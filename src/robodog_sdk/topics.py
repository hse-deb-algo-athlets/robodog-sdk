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

The registry is introspectable::

    zenode topics --contract robodog_sdk.topics
"""

from __future__ import annotations

from zenode import Service, Topic, TopicSet

from .msgs.control import ArbiterStatus, ControlGrant, ControlRelease, ControlRequest
from .msgs.input import GamepadState, GamepadStatus
from .msgs.motion import ActionCommand, EmergencyStopCommand, MovementCommand, TiltBody
from .msgs.navigation import (
    NavigationCancel,
    NavigationRequest,
    NavigationStatus,
    PlannedPath,
    ProtectiveFieldEvent,
)
from .msgs.robot import BatteryState, MotorState, OdometryState, RobotHighState

#: Fraction of traces recorded as spans when one starts on a continuous
#: stream. Unsampled traces still carry a trace id, so ``zenode logs --trace``
#: and ``zenode trace`` work at full rate; only span recording is skipped.
#: At 0.01 an odometry stream at 20 Hz records roughly one trace every five
#: seconds.
TRACE_RATIO = 0.01

#: Maximum age of a movement command, in seconds. Older samples are dropped
#: rather than executed, which also serves as the deadman: when a producer
#: stops publishing, the arbiter falls through to zero velocity. Age is
#: measured across hosts and requires synchronized clocks (NTP/chrony).
COMMAND_MAX_AGE = 0.3


class MotionTopics(TopicSet):
    """Velocity control and emergency stop.

    The lane keys are inputs to the arbiter. ``move`` is its output and the
    only movement key the robot bridge subscribes to; publishing there
    bypasses arbitration and is reserved for the arbiter itself.

    No lane is a trace root: a command is always caused by something upstream,
    and starting a trace here would sever it from that cause. The emergency
    stop lives in :class:`SafetyTopics`.
    """

    move = Topic(
        "command/motion/move",
        MovementCommand,
        max_age=COMMAND_MAX_AGE,
        description="Arbiter output — the robot bridge's only movement input",
    )
    move_teleop = Topic("command/motion/move/teleop", MovementCommand, max_age=COMMAND_MAX_AGE)
    move_nav = Topic("command/motion/move/nav", MovementCommand, max_age=COMMAND_MAX_AGE)
    move_agent = Topic(
        "command/motion/move/agent",
        MovementCommand,
        max_age=COMMAND_MAX_AGE,
        description="Lane for nodes outside the stack — student projects publish here",
    )


class PoseTopics(TopicSet):
    """Discrete actions and body orientation."""

    action = Topic("command/pose/action", ActionCommand)
    tilt_body = Topic("command/pose/tilt_body", TiltBody)


class ControlTopics(TopicSet):
    """Arbiter status and lane release. See :mod:`robodog_sdk.msgs.control`."""

    status = Topic("control/status", ArbiterStatus, latched=True)
    release = Topic("control/release", ControlRelease)


class ControlServices(TopicSet):
    acquire = Service(
        "control/acquire",
        request=ControlRequest,
        reply=ControlGrant,
        description="Ask the arbiter for a command lane",
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
    protective_field = Topic(
        "safety/protective_field",
        ProtectiveFieldEvent,
        latched=True,
        description="Edge-triggered: one message on breach, one when clear",
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


class NavTopics(TopicSet):
    """Navigation requests, execution status and planner output."""

    request = Topic(
        "nav/request",
        NavigationRequest,
        trace=True,
        description="Trace root, unsampled: one trace per navigation request",
    )
    cancel = Topic("nav/cancel", NavigationCancel)
    status = Topic("nav/status", NavigationStatus, latched=True)
    planned_path = Topic("nav/planned_path", PlannedPath, latched=True)


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
