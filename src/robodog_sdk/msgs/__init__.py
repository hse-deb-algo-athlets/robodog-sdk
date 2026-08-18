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
    EstopPolicy,
    NavActivity,
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
from .safety import ButtonEvent, EstopPhase, SafetyState
from .system_state import (
    GO2_MODE_TO_POSTURE,
    ControlMode,
    Headline,
    Location,
    OrderActivity,
    Posture,
    SystemState,
    VdaFacet,
)

__all__ = [
    "FREE",
    "GO2_MODE_TO_POSTURE",
    "INSCRIBED",
    "LETHAL",
    "ActionCommand",
    "ActionType",
    "AnalogInput",
    "Axis",
    "BatteryLevel",
    "BatteryState",
    "ButtonEvent",
    "Buttons",
    "CancelAck",
    "CancelRequest",
    "CollisionZoneEvent",
    "CommandedStance",
    "ControlMode",
    "CostMap",
    "EmergencyStop",
    "EmergencyStopCommand",
    "EstopPhase",
    "EstopPolicy",
    "GamepadState",
    "GamepadStatus",
    "GatewayAction",
    "Headline",
    "IMUState",
    "Location",
    "MotionGatewayStatus",
    "MotorState",
    "MovementCommand",
    "MovementSource",
    "NavActivity",
    "NavigateThroughPosesGoal",
    "NavigateToPoseGoal",
    "OdometryState",
    "OrderActivity",
    "PathWaypoint",
    "PlannedPath",
    "Pose2D",
    "Posture",
    "RobotHighState",
    "SafetyState",
    "SystemState",
    "TaskFeedback",
    "TaskGoal",
    "TaskGoalEnvelope",
    "TaskHandle",
    "TaskResult",
    "TaskState",
    "TaskStatusRequest",
    "TiltBody",
    "VdaFacet",
    "normalize_angle",
    "quaternion_to_yaw",
]
