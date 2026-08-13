"""Payload schemas for every topic in the contract.

Ported from ``robodog-digipro:src/interfaces``. Not yet ported, and tracked in
the CHANGELOG: ``camera.py`` / ``image.py`` (JPEG frame envelopes) and
``livox.py`` (CDR point clouds, which land in ``robodog_sdk.contrib`` behind the
``[livox]`` extra rather than here).
"""

from __future__ import annotations

from .input import AnalogInput, Axis, Buttons, GamepadState, GamepadStatus
from .motion import (
    ActionCommand,
    ActionType,
    EmergencyStop,
    EmergencyStopCommand,
    GatewayAction,
    MotionGatewayStatus,
    MovementCommand,
    MovementSource,
    TiltBody,
)
from .navigation import (
    CancelAck,
    CancelRequest,
    CollisionZoneEvent,
    NavigateThroughPosesGoal,
    NavigateToPoseGoal,
    PathWaypoint,
    PlannedPath,
    Pose2D,
    TaskFeedback,
    TaskGoal,
    TaskGoalEnvelope,
    TaskHandle,
    TaskResult,
    TaskState,
    TaskStatusRequest,
    normalize_angle,
    quaternion_to_yaw,
)
from .occupancy import FREE, INSCRIBED, LETHAL, CostMap
from .robot import (
    BatteryLevel,
    BatteryState,
    CommandedStance,
    IMUState,
    MotorState,
    OdometryState,
    RobotHighState,
)

__all__ = [
    "FREE",
    "INSCRIBED",
    "LETHAL",
    "ActionCommand",
    "ActionType",
    "AnalogInput",
    "Axis",
    "BatteryLevel",
    "BatteryState",
    "Buttons",
    "CancelAck",
    "CancelRequest",
    "CollisionZoneEvent",
    "CommandedStance",
    "CostMap",
    "EmergencyStop",
    "EmergencyStopCommand",
    "GamepadState",
    "GamepadStatus",
    "GatewayAction",
    "IMUState",
    "MotionGatewayStatus",
    "MotorState",
    "MovementCommand",
    "MovementSource",
    "NavigateThroughPosesGoal",
    "NavigateToPoseGoal",
    "OdometryState",
    "PathWaypoint",
    "PlannedPath",
    "Pose2D",
    "RobotHighState",
    "TaskFeedback",
    "TaskGoal",
    "TaskGoalEnvelope",
    "TaskHandle",
    "TaskResult",
    "TaskState",
    "TaskStatusRequest",
    "TiltBody",
    "normalize_angle",
    "quaternion_to_yaw",
]
