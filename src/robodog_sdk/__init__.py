"""Wire contract and client for the Robodog Zenoh control system.

The package provides two things:

- The contract — :mod:`robodog_sdk.topics` and :mod:`robodog_sdk.msgs` — which
  binds every key to its payload type and delivery semantics. This is the
  supported interface between the control stack and anything built on it.
- :class:`RobotClient`, a convenience layer over that contract with no
  privileged access to it.

Names exported from this module are public API; anything else may change
without a major version.

Example::

    from zenode import Node, run, subscribe
    from robodog_sdk import RobotClient, StateTopics


    class Wanderer(Node):
        name = "wanderer"

        async def on_start(self) -> None:
            self.robot = RobotClient(self)

        @subscribe(StateTopics.odometry, mode="latest")
        async def on_pose(self, msg) -> None:
            self.log.info("at %.2f, %.2f", msg.x, msg.y)


    def cli() -> None:
        run(Wanderer)

Deployment configuration (namespace, endpoints, running the stack) is covered
in ``README.md``. ``zenode topics --contract robodog_sdk.topics`` prints the
contract.
"""

from __future__ import annotations

from importlib.metadata import version as _version

from . import frames, limits
from .client import Latest, RobotClient, StateView
from .msgs import (
    ActionCommand,
    ActionType,
    ArbiterStatus,
    BatteryLevel,
    BatteryState,
    CommandedStance,
    ConnectionStatus,
    ControlGrant,
    ControlRelease,
    ControlRequest,
    Corridor,
    EmergencyStop,
    EmergencyStopCommand,
    IMUState,
    Lane,
    MotorState,
    MovementCommand,
    MovementSource,
    NavigationCancel,
    NavigationRequest,
    NavigationSegment,
    NavigationState,
    NavigationStatus,
    OdometryState,
    PathWaypoint,
    PlannedPath,
    Pose2D,
    ProtectiveFieldEvent,
    RobotHighState,
    TiltBody,
    normalize_angle,
    quaternion_to_yaw,
)
from .topics import (
    COMMAND_MAX_AGE,
    ControlServices,
    ControlTopics,
    LocalizationTopics,
    MotionTopics,
    NavTopics,
    PoseTopics,
    StateTopics,
)

__version__ = _version("robodog-sdk")

#: Contract revision reported on a node's health heartbeat, so that a version
#: mismatch between a project and the deployed stack is visible in
#: ``zenode health`` rather than surfacing as a decoding failure.
CONTRACT_VERSION = __version__

__all__ = [
    "COMMAND_MAX_AGE",
    "CONTRACT_VERSION",
    "ActionCommand",
    "ActionType",
    "ArbiterStatus",
    "BatteryLevel",
    "BatteryState",
    "CommandedStance",
    "ConnectionStatus",
    "ControlGrant",
    "ControlRelease",
    "ControlRequest",
    "ControlServices",
    "ControlTopics",
    "Corridor",
    "EmergencyStop",
    "EmergencyStopCommand",
    "IMUState",
    "Lane",
    "Latest",
    "LocalizationTopics",
    "MotionTopics",
    "MotorState",
    "MovementCommand",
    "MovementSource",
    "NavTopics",
    "NavigationCancel",
    "NavigationRequest",
    "NavigationSegment",
    "NavigationState",
    "NavigationStatus",
    "OdometryState",
    "PathWaypoint",
    "PlannedPath",
    "Pose2D",
    "PoseTopics",
    "ProtectiveFieldEvent",
    "RobotClient",
    "RobotHighState",
    "StateTopics",
    "StateView",
    "TiltBody",
    "__version__",
    "frames",
    "limits",
    "normalize_angle",
    "quaternion_to_yaw",
]
