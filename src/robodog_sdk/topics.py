"""Topic and service declarations for the Robodog contract.

Each :class:`~zenode.Topic` binds a key to its payload type, codec and delivery
semantics; both publishers and subscribers derive their behaviour from it.

Keys are relative. The deployment namespace (``[transport] namespace``, e.g.
``robodog``) is applied at runtime. Keys owned by external producers are
declared with :meth:`zenode.Topic.absolute` and ignore the namespace.

.. note::

   The stack currently hard-codes the ``robodog/`` prefix into its own key
   strings rather than deriving it from a namespace, so a deployment of this
   package must run with ``namespace = "robodog"`` to address it. Namespaced
   sandboxes are a change pending on the stack side, not on this one.

Topics that begin a causal chain are declared ``trace=True``, so a trace
follows the data across every node that reacts to it (see ``TRACE_RATIO``).
A :class:`~zenode.Service` cannot be a trace root — it continues the caller's
trace, or starts none — so submitting a navigation task from a script is not
the head of a chain the way publishing a message is.

``latched=True`` marks the topics whose value a late joiner needs. It is
delivered by zenoh-ext advanced pub/sub, which requires the *producer* to
participate; the stack's nodes publish with plain Zenoh publishers today, so on
those keys latching costs nothing and delivers nothing until they are upgraded.
Where late-join genuinely matters the stack answers a query instead:
:attr:`SafetyTopics.state`, :attr:`StateTopics.vda` and
:attr:`StateTopics.system` are backed by queryables.

The registry is introspectable::

    zenode topics --contract robodog_sdk.topics
"""

from __future__ import annotations

from zenode import Service, Topic, TopicSet

from .msgs.input import GamepadState, GamepadStatus
from .msgs.localization import MapIdentity
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
    PlannedPath,
    TaskFeedback,
    TaskGoalEnvelope,
    TaskHandle,
    TaskResult,
    TaskStatusRequest,
)
from .msgs.occupancy import CostMap, GridMap
from .msgs.robot import BatteryState, MotorState, OdometryState, RobotHighState
from .msgs.safety import ButtonEvent, SafetyState
from .msgs.system_state import SystemState, VdaFacet

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

    One key. The gateway publishes on every change — the active source, a
    collision zone firing or clearing, the watchdog tripping — and re-asserts
    the current value on a heartbeat, about once a second, so a late subscriber
    is at most one beat behind rather than waiting for a change that may never
    come. Between beats the last value stands.

    The heartbeat is what makes the age of this value meaningful: silence for
    several seconds means the gateway itself is gone, not that nothing has
    changed. :attr:`SafetyTopics.collision_zone` is the opposite — edge-only,
    where age says nothing.

    This is the first thing to read when commands go out and nothing happens.
    ``active_source`` names who won, ``action`` and ``active_zones`` say what
    the collision monitor did to the command, and ``watchdog_tripped`` says the
    winner went quiet. It answers "is anything *forwarding* my command"; for
    "may the robot move at all", read :attr:`SafetyTopics.state`.
    """

    status = Topic(
        "motion/gateway/status",
        MotionGatewayStatus,
        latched=True,
        description="The gateway's decision and its reason; on change, plus a ~1 Hz heartbeat",
    )


#: Key template for the per-source safety latches. ``{source_id}`` names one
#: safety panel.
SAFETY_SOURCE_PREFIX = "safety/source"


def safety_source_key(source_id: str) -> str:
    """Relative key carrying one safety source's own :class:`SafetyState`."""
    return f"{SAFETY_SOURCE_PREFIX}/{source_id}"


def safety_source_topic(source_id: str) -> Topic[SafetyState]:
    """Latch topic for one safety source, for ``node.subscribe``.

    :attr:`SafetyTopics.source` covers every source at once and is what the
    aggregator subscribes to; this narrows it to one, for a panel that reports
    on itself.
    """
    return Topic(safety_source_key(source_id), SafetyState, latched=True)


class SafetyTopics(TopicSet):
    """The safety path, in one prefix so it can be audited at a glance.

    ``zenode echo 'safety/**'`` shows the whole of it. See ADR-002, ADR-004 and
    ADR-005 in the robodog-digipro repository.

    :attr:`state` is the authority and the only key to make a decision on. It
    is a *level*, republished on a heartbeat as well as on every change, so a
    dropped packet costs one tick rather than the truth. Fail safe on silence:
    no fresh frame within the deadline, a lost liveliness token, or
    ``source_alive=False`` all mean stopped.

    :attr:`estop` is a mirror of that latch as an edge, kept for the consumers
    that predate the aggregator. It carries the latch and nothing else — not
    the recovery phase — so it drops one phase *before* the robot can actually
    move again. Anything deciding whether to drive wants
    ``state.motion_permitted``, not this.
    """

    state = Topic(
        "safety/state",
        SafetyState,
        latched=True,
        description="The authority: a continuous latch, heartbeat included",
    )
    source = Topic(
        f"{SAFETY_SOURCE_PREFIX}/*",
        SafetyState,
        latched=True,
        description="Subscribe-only: every panel's own latch, OR-combined by the aggregator",
    )
    release = Topic(
        "safety/release",
        ButtonEvent,
        description="Momentary: acknowledges a stop once the switch is pulled out",
    )
    cancel = Topic(
        "safety/cancel",
        ButtonEvent,
        description="Momentary: stop now, without latching the switch",
    )
    estop = Topic(
        "command/motion/estop",
        EmergencyStopCommand,
        latched=True,
        description="Legacy edge mirror of the latch — prefer `state`",
    )
    # TODO: Define correct blocked topic, LATCHED?
    collision_zone = Topic(
        "motion/collision/event",
        CollisionZoneEvent,
        latched=True,
        description="Edge-triggered: one message on breach, one when the zone clears",
    )


class InputTopics(TopicSet):
    """Human input devices.

    Both keys are absolute: the teleoperation node publishes them at the root
    of the keyspace rather than under the deployment namespace. That is where
    they are, so that is what is declared — a namespaced key here would
    subscribe to silence.
    """

    gamepad = Topic.absolute("nodes/joy", GamepadState)
    gamepad_status = Topic.absolute("nodes/controller_status", GamepadStatus, latched=True)


class StateTopics(TopicSet):
    """Robot state, published by the Go2 bridge or the simulation.

    The first four are the raw streams off the robot. :attr:`system` is the
    composite the system-state node fuses from all of them plus safety and the
    fleet runtime, and is the one to read when the question is "what is going
    on" rather than "what is this one sensor saying".
    """

    highstate = Topic("system_state/highstate", RobotHighState, latched=True)
    odometry = Topic(
        "system_state/odometry",
        OdometryState,
        latched=True,
        trace=True,
        trace_ratio=TRACE_RATIO,
        description="Trace root: sense-decide-act chains begin at a pose",
    )
    battery = Topic("system_state/battery", BatteryState, latched=True)
    motor = Topic("system_state/motor", MotorState, latched=True)
    releasebutton = Topic(
        "system_state/releasebutton",
        ButtonEvent,
        description="The release press, forwarded for the fleet bridge's wait actions",
    )
    vda = Topic(
        "system_state/vda",
        VdaFacet,
        latched=True,
        description="The fleet bridge's facet — control, order, location — for fusing",
    )
    system = Topic(
        "system_state/system",
        SystemState,
        latched=True,
        description="The composite: every facet, plus a headline derived from them",
    )


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

    :attr:`map_identity` says which map that pose is anchored to, and anything
    storing a map-frame coordinate needs it. A pose is only comparable to a
    stored one while the map is the same; nothing in a bare pose says
    otherwise, so a rebuilt map silently turns every saved coordinate into a
    confident drive to the wrong place.

    Both keys come from whatever is localising — MOLA, or the odometry
    fallback, which publishes only :attr:`pose` and no identity at all. That
    silence is the correct answer there: odometry has no map. :attr:`grid
    <MapTopics.grid>` carries the raster of the same map, and agrees with
    :attr:`map_identity` on ``map_id`` because one producer publishes both.

    :attr:`map_identity` is latched *and* re-stated on a slow heartbeat, unlike
    the grid, which is change-only. The heartbeat is what makes its age mean
    something: a consumer treats an identity older than its threshold as "no
    usable map", so silence here has to say the pose source is gone rather than
    that nothing has changed. See :meth:`~robodog_sdk.RobotClient.map_id`.
    """

    pose = Topic(
        "localization/pose",
        OdometryState,
        latched=True,
        trace=True,
        trace_ratio=TRACE_RATIO,
        description="Trace root: neither producer is a zenode node, so no context arrives",
    )
    map_identity = Topic(
        "localization/map_identity",
        MapIdentity,
        latched=True,
        description="Which map `pose` is anchored to; on change, plus a ~0.2 Hz heartbeat",
    )


class MapTopics(TopicSet):
    """The map the deployment is operating in, as SLAM built it.

    Published by the SLAM stack itself rather than relayed by a consumer, so
    the map is on the bus whenever SLAM is — not only while the navigation
    node happens to be running. It is what makes reading the map a
    subscription instead of a call to MOLA's HTTP control API.

    Latched and event-driven: it is published when a grid is rebuilt and when
    the active session changes, and never on a timer. A late joiner gets the
    current map on subscribe, so there is no window to wait through and no
    keep-alive traffic in between. That also means :attr:`~GridMap.stamp` is
    the age of the last *change*, not of the last heartbeat — an old stamp on
    this key is normal and says nothing about whether SLAM is alive. Read
    :attr:`LocalizationTopics.pose` for that.

    :attr:`grid` is the raw grid. The navigation node's inflated view of the
    same map is :attr:`NavTopics.costmap_global`, and the two agree on
    :attr:`~GridMap.map_id` — when they disagree, one of them has not caught
    up with a remap yet, and the map-frame coordinates you hold belong to
    whichever is older.
    """

    grid = Topic(
        "map/grid",
        GridMap,
        latched=True,
        description="The SLAM session's occupancy grid; on rebuild and on session change",
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
    remembers, or one carrying :attr:`TaskState.RUNNING` for a task still under
    way — so unlike the result key, a reply here is not necessarily terminal.

    For a task it does not remember — never submitted, or evicted from its
    bounded history — it answers on the Zenoh **error channel**, which surfaces
    as an exception rather than as a result. "Unknown" is not a lifecycle
    state, and the contract refuses to dress it as one.
    """
    return Service(
        f"{TASK_KEY_PREFIX}/{task_id}/status",
        request=TaskStatusRequest,
        reply=TaskResult,
    )


class NavTopics(TopicSet):
    """Task feedback, results, the planned route and the cost grids.

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
        description="Subscribe-only: every running task's progress, ~10 Hz",
    )
    result = Topic(
        f"{TASK_KEY_PREFIX}/*/result",
        TaskResult,
        description="Subscribe-only: one terminal payload per task",
    )
    path = Topic(
        "nav/path",
        PlannedPath,
        latched=True,
        description="The route a skill committed to, republished on every replan",
    )
    costmap_global = Topic(
        "nav/costmap/global",
        CostMap,
        latched=True,
        description="The active MOLA session's map, already inflated by the robot radius",
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

    Two knobs travel as query parameters on the submit rather than in the
    payload, so that the goal on the wire stays exactly the goal:
    ``?preempt=true`` displaces a running task, and ``?on_estop=hold`` keeps
    this task across an emergency stop instead of discarding it (see
    :class:`~robodog_sdk.msgs.navigation.EstopPolicy`). The coordinator also
    reads ``?client=`` and records it against the task, for logs.

    Skills are named by string rather than declared here, because which ones a
    deployment registers is a property of that deployment. The stack ships
    ``global_nav`` (plans through the global map, and the default),
    ``corridor_assist`` (plans, but hands the wheel to a reactive controller
    through the tight bits), ``waypoint_follow`` (drives the route it was
    given, without planning), ``door_traverse`` and ``dummy``.

    They do not all read the same goal fields. ``global_nav`` drives a route
    leg by leg, planning to each pose in turn, and is the only skill that
    honours ``dwell_sec``; ``waypoint_follow`` drives the polyline as one
    continuous motion, which is why a dwell has nowhere to happen. Both honour
    the requested arrival heading, and both apply it only to the final pose.

    A pub/sub adapter (``nav/simple/{goto,cancel,status}``) exists on the stack
    for clients that cannot speak this contract at all — untyped JSON, no
    services. It is deliberately not declared here: anything holding this
    package should use the services.
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


# TODO(port): sensor topics. Wire formats observed on a running stack, so these
# need no guessing:
#
#   robodog/sensors/go2_camera          raw JPEG (SOI + JFIF), ~14 Hz
#                                       -> Topic(..., bytes,
#                                          codec=RawCodec(Encoding.IMAGE_JPEG),
#                                          shm=True)
#   robodog/sensors/go2_lidar           raw, from the Go2's own scanner
#   robodog/sensors/livox/pointcloud    ROS 2 CDR PointCloud2, 10 Hz
#   robodog/sensors/livox/imu           ROS 2 CDR Imu, 200 Hz
#                                       -> bytes + a CDR codec, [livox] extra
#   robodog/sensors/realsense/*         rgb_img, depth_img, depth_data, imu,
#                                       intrinsics
#
# The Livox pair is published twice: once under livox/lidar by
# zenoh-bridge-ros2dds and once republished under robodog/sensors/livox/*. The
# nav node subscribes both so one build runs against sim and hardware; only the
# namespaced keys belong in this contract.
#
# Deliberately not ported:
#
#   robodog/diagnostic/**   A display fan-out. The diagnostic node re-publishes
#                           fields of highstate and battery one scalar per key,
#                           each wrapped in a {name, category, data} envelope.
#                           It is a view, not a source; read StateTopics.
#   robodog/liveliness/**   Liveliness token keys held by the Go2 bridge, not
#                           pub/sub topics. zenode's own presence covers this
#                           for zenode nodes, and the bridge is not one.
