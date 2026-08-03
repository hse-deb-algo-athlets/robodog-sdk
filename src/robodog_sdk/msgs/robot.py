"""Robot state payloads.

The ``from_raw()`` parsers of the original schemas are not included: they
decode Unitree WebRTC frames, which concern only the Go2 bridge. This module
describes what appears on the bus, not how a producer populates it.
"""

from __future__ import annotations

from datetime import UTC, datetime
from enum import StrEnum

from pydantic import BaseModel, Field


def _utcnow() -> datetime:
    return datetime.now(UTC)


class CommandedStance(StrEnum):
    UNDEFINED = "undefined"
    STANDING = "standing"
    LYING_DOWN = "lying_down"
    SITTING = "sitting"


class ConnectionStatus(BaseModel):
    """State of the link between the bridge and the robot."""

    connected: bool = False
    motion_mode: str = "unknown"
    stance: CommandedStance = CommandedStance.UNDEFINED


class IMUState(BaseModel):
    quaternion: list[float] = Field(default_factory=list)
    gyroscope: list[float] = Field(default_factory=list)
    accelerometer: list[float] = Field(default_factory=list)
    rpy: list[float] = Field(default_factory=list)


class RobotHighState(BaseModel):
    """High-level robot state from the Go2 sport mode."""

    imu_state: IMUState = Field(default_factory=IMUState)
    mode: int = 0
    velocity: list[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
    yaw_speed: float = 0.0
    position: list[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
    body_height: float = 0.0
    foot_force: list[int] = Field(default_factory=list)
    timestamp: datetime = Field(default_factory=_utcnow)


class BatteryLevel(StrEnum):
    good = "good"
    low = "low"
    critical = "critical"


class BatteryState(BaseModel):
    soc: int = Field(default=0, ge=0, le=100)
    level: BatteryLevel = BatteryLevel.good
    voltage: float = 0.0
    current: float = 0.0  # A — sign follows the BMS convention
    temperature: float = 0.0  # °C — hottest BMS cell
    timestamp: datetime = Field(default_factory=_utcnow)


class MotorState(BaseModel):
    temperatures: list[float] = Field(default_factory=list)  # °C, one per motor
    timestamp: datetime = Field(default_factory=_utcnow)


class OdometryState(BaseModel):
    """Robot pose from an odometry or SLAM source."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    quaternion: list[float] = Field(
        default_factory=lambda: [0.0, 0.0, 0.0, 1.0],
        description="Orientation as [qx, qy, qz, qw]",
    )
    timestamp: datetime = Field(default_factory=_utcnow)
