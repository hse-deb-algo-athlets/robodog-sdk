"""Topic and service declarations for the Robodog contract.

Each :class:`~zenode.Topic` binds a key to its payload type, codec and delivery
semantics; both publishers and subscribers derive their behaviour from it.

Keys are relative. The deployment namespace (``[transport] namespace``, e.g.
``robodog`` or ``robodog/team-03``) is applied at runtime, so one contract
addresses a robot, a simulation and any number of isolated sandboxes. Keys
owned by external producers are declared with :meth:`zenode.Topic.absolute`
and ignore the namespace.

Topics that begin a causal chain are declared ``trace=True``, so a trace
follows the data across every node that reacts to it (see ``TRACE_RATIO``).
A :class:`~zenode.Service` cannot be a trace root — it continues the caller's
trace, or starts none — so submitting a navigation task from a script is not
the head of a chain the way publishing a message is.

The registry is introspectable::

    zenode topics --contract robodog_sdk.topics
"""

from __future__ import annotations

from zenode import Service, Topic, TopicSet

from .msgs.input import GamepadState, GamepadStatus
from .msgs.motion import (
    ActionCommand,
    EmergencyStopCommand,
    MotionGatewayStatus,
    MovementCommand,
    TiltBody,
)
from .msgs.navigation import (
    CancelAck,
    CancelRequest,
    CollisionZoneEvent,
    TaskFeedback,
    TaskGoalEnvelope,
    TaskHandle,
    TaskResult,
    TaskStatusRequest,
)
from .msgs.occupancy import CostMap
from .msgs.robot import BatteryState, MotorState, OdometryState, RobotHighState

#: Fraction of traces recorded as spans when one starts on a continuous
#: stream. Unsampled traces still carry a trace id, so ``zenode logs --trace``
#: and ``zenode trace`` work at full rate; only span recording is skipped.
#: At 0.01 an odometry stream at 20 Hz records roughly one trace every five
#: seconds.
TRACE_RATIO = 0.01

#: Maximum age of a movement command, in seconds. Older samples are dropped
#: rather than executed, which also serves as the deadman: when a producer
#: stops publishing, the gateway's watchdog falls through to zero velocity and
#: the next-ranking source takes over. Age is measured across hosts and
#: requires synchronized clocks (NTP/chrony).
COMMAND_MAX_AGE = 0.3


class MotionTopics(TopicSet):
    """Velocity control.

    Two keys and one direction of travel. Every source — teleoperation, a
    navigation skill, your node — publishes a :class:`MovementCommand` to
    ``request``. The motion gateway picks whichever fresh command has the
    highest-ranking :class:`~robodog_sdk.msgs.motion.MovementSource`, passes it
    through the collision zones, and publishes the survivor to ``move``, which
    the robot bridge is the only subscriber to.

    So ``move`` is an *output*: reading it tells you what the robot was
    actually told. Publishing there bypasses arbitration and the collision
    monitor both, and is reserved for the gateway.

    There is no handshake and nothing to acquire. Priority is carried on every
    command, which means it is re-decided on every frame: stop publishing and
    you stop being the driver, without having to say so.

    Neither key is a trace root: a command is always caused by something
    upstream, and starting a trace here would sever it from that cause. The
    emergency stop lives in :class:`SafetyTopics`.
    """

    request = Topic(
        "motion/gateway/in",
        MovementCommand,
        max_age=COMMAND_MAX_AGE,
        description="The inlet — every movement source publishes here",
    )
    move = Topic(
        "command/motion/move",
        MovementCommand,
        max_age=COMMAND_MAX_AGE,
        description="Gateway output — the robot bridge's only movement input",
    )


class PoseTopics(TopicSet):
    """Discrete actions and body orientation."""

    action = Topic("command/pose/action", ActionCommand)
    tilt_body = Topic("command/pose/tilt_body", TiltBody)


class ControlTopics(TopicSet):
    """Who is driving, and why the robot is not moving.

    One key, latched and edge-published: the gateway emits a
    :class:`~robodog_sdk.msgs.motion.MotionGatewayStatus` when the active
    source changes, when a collision zone fires or clears, or when the watchdog
    trips — not on every tick. Between edges the last value stands.

    This is the first thing to read when commands go out and nothing happens.
    ``active_source`` names who won, ``action`` and ``active_zones`` say what
    the collision monitor did to the command, and ``watchdog_tripped`` says the
    winner went quiet.
    """

    status = Topic(
        "control/status",
        MotionGatewayStatus,
        latched=True,
        description="Edge-published: the gateway's decision, and its reason",
    )


class SafetyTopics(TopicSet):
    """The safety path, in one prefix so it can be audited at a glance.

    ``zenode echo 'safety/**'`` shows the whole of it. See ADR-002, ADR-004 and
    ADR-005 in the robodog-digipro repository.
    """

    estop = Topic(
        "safety/estop",
        EmergencyStopCommand,
        latched=True,
        description="Latched: a node joining mid-stop learns that it is stopped",
    )
    collision_zone = Topic(
        "safety/collision_zone",
        CollisionZoneEvent,
        latched=True,
        description="Edge-triggered: one message on breach, one when the zone clears",
    )
    # TODO(port): safety/release_button. The ESP32 publishes an untyped JSON
    # blob and its only consumer treats any message as a trigger without
    # inspecting it, so there is no schema to declare yet.


class InputTopics(TopicSet):
    """Human input devices.

    Under ``input/`` rather than ``node/``, which zenode reserves for presence,
    health, log and trace keys.
    """

    gamepad = Topic("input/gamepad", GamepadState)
    gamepad_status = Topic("input/gamepad/status", GamepadStatus, latched=True)


class StateTopics(TopicSet):
    """Robot state, published by the Go2 bridge or the simulation.

    All latched: a subscriber joining late receives the current value rather
    than waiting for the next update.

    """

    highstate = Topic("state/highstate", RobotHighState, latched=True)
    odometry = Topic(
        "state/odometry",
        OdometryState,
        latched=True,
        trace=True,
        trace_ratio=TRACE_RATIO,
        description="Trace root: sense-decide-act chains begin at a pose",
    )
    battery = Topic("state/battery", BatteryState, latched=True)
    motor = Topic("state/motor", MotorState, latched=True)


class LocalizationTopics(TopicSet):
    """The robot's fused pose.

    One key, and exactly one producer at a time: either the MOLA SLAM stack or
    the odometry fallback node, never both (ADR-003). Consumers do not need to
    know which is running.

    MOLA's own ROS 2 output — ``lidar_odometry/pose`` and
    ``lidar_odometry/pose_quality``, CDR-encoded ``PoseStamped`` bridged by
    ``zenoh-bridge-ros2dds`` — is not part of this contract. It is internal to
    that container, carries a different wire format, and is not what a
    consumer of the robot's pose should subscribe to.
    """

    pose = Topic(
        "localization/pose",
        OdometryState,
        latched=True,
        trace=True,
        trace_ratio=TRACE_RATIO,
        description="Trace root: neither producer is a zenode node, so no context arrives",
    )


#: Key template for the per-task keys. ``{task_id}`` is the id from the
#: :class:`~robodog_sdk.msgs.navigation.TaskHandle` a submit returned.
TASK_KEY_PREFIX = "nav/task"


def task_feedback_key(task_id: str) -> str:
    """Relative key carrying :class:`TaskFeedback` for one task."""
    return f"{TASK_KEY_PREFIX}/{task_id}/feedback"


def task_result_key(task_id: str) -> str:
    """Relative key carrying the one :class:`TaskResult` for one task."""
    return f"{TASK_KEY_PREFIX}/{task_id}/result"


def task_feedback_topic(task_id: str) -> Topic[TaskFeedback]:
    """Feedback topic for one task, for ``node.subscribe``.

    :attr:`NavTopics.feedback` covers every task at once and is usually what
    you want; this narrows it to one when a process follows several tasks and
    would rather not demultiplex.
    """
    return Topic(task_feedback_key(task_id), TaskFeedback)


def task_result_topic(task_id: str) -> Topic[TaskResult]:
    """Result topic for one task, for ``node.subscribe``.

    Subscribe **before** submitting. The key is not latched and carries
    exactly one message, so a subscription declared after the task finished
    receives nothing at all — :func:`task_status_service` is the way to ask
    after the fact.
    """
    return Topic(task_result_key(task_id), TaskResult)


def task_status_service(task_id: str) -> Service[TaskStatusRequest, TaskResult]:
    """Late-poll query for one task, for ``node.call``.

    The coordinator answers with the recorded :class:`TaskResult` for a task it
    remembers. For one it does not — never submitted, or evicted from its
    bounded history — it answers ``PENDING``, so a ``PENDING`` reply means
    "unknown", not "queued".
    """
    return Service(
        f"{TASK_KEY_PREFIX}/{task_id}/status",
        request=TaskStatusRequest,
        reply=TaskResult,
    )


class NavTopics(TopicSet):
    """Task feedback, results and the cost grids the planner works on.

    The two task keys are declared with a wildcard over the task id, which is
    how a client subscribes to every task without knowing an id in advance —
    both payloads carry ``task_id``, so demultiplexing needs no key parsing.
    The producer publishes on the concrete key (:func:`task_feedback_key`,
    :func:`task_result_key`); **do not publish on a wildcard**.

    Neither is latched, and a result is published exactly once. A subscription
    declared after a task finished sees nothing; ask
    :func:`task_status_service` instead.
    """

    feedback = Topic(
        f"{TASK_KEY_PREFIX}/*/feedback",
        TaskFeedback,
        description="Subscribe-only: every running task's progress, ~10-20 Hz",
    )
    result = Topic(
        f"{TASK_KEY_PREFIX}/*/result",
        TaskResult,
        description="Subscribe-only: one terminal payload per task",
    )
    costmap_global = Topic(
        "nav/costmap/global",
        CostMap,
        latched=True,
        description="The deployment map, already inflated by the robot radius",
    )
    costmap_local = Topic(
        "nav/costmap/local",
        CostMap,
        description="Rolling body-frame window rasterized from the LiDAR",
    )


class NavServices(TopicSet):
    """Submitting and cancelling navigation tasks.

    ``submit`` starts a task and replies with its id; there is no queue, so a
    submit while another task runs is refused unless it asks to preempt (see
    :meth:`~robodog_sdk.RobotClient.navigate_to`). The skill that carries the
    goal out is named by the goal or left to the deployment default.

    Skills are named by string rather than declared here, because which ones a
    deployment registers is a property of that deployment. The stack ships
    ``global_nav`` (plans through the global map), ``waypoint_follow`` (drives
    the route it was given, without planning), ``door_traverse`` and ``dummy``.
    They do not all read the same goal fields: a planning skill may treat the
    intermediate poses of a
    :class:`~robodog_sdk.msgs.navigation.NavigateThroughPosesGoal` as advisory
    and route to the last one itself.
    """

    submit = Service(
        "nav/task/submit",
        request=TaskGoalEnvelope,
        reply=TaskHandle,
        description="Start a navigation task; replies with its id",
    )
    cancel = Service(
        "nav/task/cancel",
        request=CancelRequest,
        reply=CancelAck,
        description="Abandon a task and stop",
    )
    status = Service(
        f"{TASK_KEY_PREFIX}/*/status",
        request=TaskStatusRequest,
        reply=TaskResult,
        description="Declared per task; call it via task_status_service(task_id)",
    )


# TODO(port): sensor and controller topics. Wire formats observed on a running
# stack, so these need no guessing:
#
#   robodog/sensors/go2_camera          raw JPEG (SOI + JFIF), ~14 Hz
#                                       -> Topic(..., bytes,
#                                          codec=RawCodec(Encoding.IMAGE_JPEG),
#                                          shm=True)
#   robodog/sensors/livox/pointcloud    ROS 2 CDR PointCloud2, 10 Hz
#   robodog/sensors/livox/imu           ROS 2 CDR Imu, 200 Hz
#                                       -> bytes + a CDR codec, [livox] extra
#   nodes/joy                           JSON, ~15 Hz -> ControllerState
#
# The Livox pair is published twice: once under livox/* by
# zenoh-bridge-ros2dds and once republished under robodog/sensors/livox/*.
# Only the namespaced keys belong in this contract.
