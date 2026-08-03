"""Motion and pose command payloads.

Velocity and tilt fields carry the capability envelope from
:mod:`robodog_sdk.limits` as field constraints, so an out-of-range command is
rejected in the sending process rather than on the robot.
"""

from __future__ import annotations

from datetime import UTC, datetime
from enum import StrEnum

from pydantic import BaseModel, Field

from ..limits import (
    MAX_BODY_YAW_DEG,
    MAX_LATERAL_MS,
    MAX_LINEAR_MS,
    MAX_TILT_DEG,
    MAX_YAW_RATE_DEG,
)


def _utcnow() -> datetime:
    return datetime.now(UTC)


class MovementSource(StrEnum):
    """Origin of a movement command.

    Provenance only. Priority is determined by the lane a command is published
    to (see :class:`~robodog_sdk.topics.MotionTopics`), never by this field.
    """

    controller = "controller"
    autonomous = "autonomous"
    planner = "planner"


class MovementCommand(BaseModel):
    """Velocity command in the body frame.

    Attributes:
        x: Forward velocity, m/s.
        y: Lateral velocity, m/s, positive left.
        z_deg: Yaw rate, deg/s, positive counter-clockwise. Degrees here and
            radians elsewhere in the contract is inherited from the Go2 API.
    """

    x: float = Field(default=0.0, ge=-MAX_LINEAR_MS, le=MAX_LINEAR_MS)
    y: float = Field(default=0.0, ge=-MAX_LATERAL_MS, le=MAX_LATERAL_MS)
    z_deg: float = Field(default=0.0, ge=-MAX_YAW_RATE_DEG, le=MAX_YAW_RATE_DEG)
    source: MovementSource = MovementSource.controller
    timestamp: datetime = Field(default_factory=_utcnow)

    def is_zero(self) -> bool:
        return self.x == 0.0 and self.y == 0.0 and self.z_deg == 0.0

    def scale(self, factor: float) -> MovementCommand:
        """Return a copy with all velocities multiplied by ``factor``.

        The result is validated against the capability envelope. Constructed
        rather than copied, since ``model_copy(update=…)`` skips validation in
        Pydantic v2 and would allow a scaled command out of range.

        Raises:
            pydantic.ValidationError: The scaled value exceeds the envelope.
        """
        return MovementCommand(
            x=self.x * factor,
            y=self.y * factor,
            z_deg=self.z_deg * factor,
            source=self.source,
            timestamp=self.timestamp,
        )


class TiltBody(BaseModel):
    """Body orientation while standing (Euler angles in degrees)."""

    pitch_deg: float = Field(default=0.0, ge=-MAX_TILT_DEG, le=MAX_TILT_DEG)
    roll_deg: float = Field(default=0.0, ge=-MAX_TILT_DEG, le=MAX_TILT_DEG)
    yaw_deg: float = Field(default=0.0, ge=-MAX_BODY_YAW_DEG, le=MAX_BODY_YAW_DEG)

    def is_zero(self) -> bool:
        return self.pitch_deg == 0.0 and self.roll_deg == 0.0 and self.yaw_deg == 0.0


class ActionType(StrEnum):
    stand_up = "stand_up"
    lie_down = "lie_down"
    sit_down = "sit_down"
    hello = "hello"
    dance1 = "dance1"
    wiggle_hips = "wiggle_hips"
    stretch = "stretch"
    stop_move = "stop_move"
    balance_stand = "balance_stand"


class ActionCommand(BaseModel):
    """A discrete action trigger (emote, stance change, etc.)."""

    action: ActionType
    timestamp: datetime = Field(default_factory=_utcnow)


class EmergencyStop(StrEnum):
    stop = "stop"
    release = "release"


class EmergencyStopCommand(BaseModel):
    """Emergency stop command.

    The topic carries both directions, so both values exist here.
    :class:`~robodog_sdk.RobotClient` exposes only ``stop``: an e-stop is
    cleared at the physical button.
    """

    command: EmergencyStop
    timestamp: datetime = Field(default_factory=_utcnow)
