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
from .msgs.motion import ActionCommand, EmergencyStopCommand, MovementCommand, TiltBody
from .msgs.navigation import (
    NavigationCancel,
    NavigationRequest,
    NavigationStatus,
    PlannedPath,
    ProtectiveFieldEvent,
)
from .msgs.robot import (
    BatteryState,
    ConnectionStatus,
    MotorState,
    OdometryState,
    RobotHighState,
)

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
    and starting a trace here would sever it from that cause.
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
    estop = Topic(
        "command/motion/estop",
        EmergencyStopCommand,
        latched=True,
        description="Latched: a node joining mid-stop learns it is stopped",
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
    connection = Topic("state/connection", ConnectionStatus, latched=True)


class LocalizationTopics(TopicSet):
    """Fused pose, from the MOLA SLAM stack or the odometry fallback node."""

    pose = Topic(
        "localization/pose",
        OdometryState,
        latched=True,
        trace=True,
        trace_ratio=TRACE_RATIO,
        description="Trace root: MOLA is not a zenode node, so no context arrives with it",
    )
    #: Published by the MOLA container under a key outside the namespace.
    lidar_odometry = Topic.absolute("lidar_odometry/pose", OdometryState)
    # TODO: MOLA publishes on the same topic localization/pose, maybe change?


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
    protective_field = Topic("nav/protective_field", ProtectiveFieldEvent, latched=True)


# TODO(port): sensor topics, once their payload types are ported — camera and
# image envelopes (bytes with RawCodec(Encoding.IMAGE_JPEG), shm=True for
# frames) and the Livox point cloud as
# Topic.absolute("livox/lidar", bytes, codec=CdrCodec(...)) behind the [livox]
# extra. Omitted rather than guessed: an incorrect codec is a silent wire
# incompatibility.
