"""Payload schemas for every topic in the contract.

Ported from ``robodog-digipro:src/interfaces``. Not yet ported, and tracked in
the CHANGELOG: ``camera.py`` / ``image.py`` (JPEG frame envelopes),
``controller.py`` (gamepad state), and ``livox.py`` (CDR point clouds, which
land in ``robodog_sdk.contrib`` behind the ``[livox]`` extra rather than here).
"""

from __future__ import annotations

from .control import ArbiterStatus, ControlGrant, ControlRelease, ControlRequest, Lane
from .motion import (
    ActionCommand,
    ActionType,
    EmergencyStop,
    EmergencyStopCommand,
    MovementCommand,
    MovementSource,
    TiltBody,
)
from .navigation import (
    Corridor,
    NavigationCancel,
    NavigationRequest,
    NavigationSegment,
    NavigationState,
    NavigationStatus,
    PathWaypoint,
    PlannedPath,
    Pose2D,
    ProtectiveFieldEvent,
    normalize_angle,
    quaternion_to_yaw,
)
from .robot import (
    BatteryLevel,
    BatteryState,
    CommandedStance,
    ConnectionStatus,
    IMUState,
    MotorState,
    OdometryState,
    RobotHighState,
)

__all__ = [
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
    "Corridor",
    "EmergencyStop",
    "EmergencyStopCommand",
    "IMUState",
    "Lane",
    "MotorState",
    "MovementCommand",
    "MovementSource",
    "NavigationCancel",
    "NavigationRequest",
    "NavigationSegment",
    "NavigationState",
    "NavigationStatus",
    "OdometryState",
    "PathWaypoint",
    "PlannedPath",
    "Pose2D",
    "ProtectiveFieldEvent",
    "RobotHighState",
    "TiltBody",
    "normalize_angle",
    "quaternion_to_yaw",
]
