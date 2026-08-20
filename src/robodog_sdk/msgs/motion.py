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
    """Who is asking the robot to move — and, because of that, how loudly.

    This field *is* the arbitration. Every source publishes to one inlet
    (:attr:`~robodog_sdk.topics.MotionTopics.request`) and the motion gateway
    forwards whichever fresh command has the highest-ranking source, so
    claiming a source you are not is how you take the wheel from someone who
    is.

    Declaration order defines the ranking; :attr:`priority` is derived from it.

    - ``autonomous`` — anything acting on its own behalf. The default, and the
      right one for a node you are writing.
    - ``planner`` — a navigation skill following a path.
    - ``assisted_teleop`` — human intent shaped by a skill rather than sent raw.
    - ``controller`` — a human on the gamepad, who outranks everything.
    """

    autonomous = "autonomous"
    planner = "planner"
    assisted_teleop = "assisted_teleop"
    controller = "controller"

    @property
    def priority(self) -> int:
        """Rank of this source. **Higher wins**, unlike an index.

        The inversion is worth reading twice: ``autonomous`` is 0 and loses to
        everything, ``controller`` is 3 and loses to nothing.
        """
        return list(MovementSource).index(self)

    def outranks(self, other: MovementSource) -> bool:
        """Whether a fresh command from this source displaces one from ``other``."""
        return self.priority > other.priority


class MovementCommand(BaseModel):
    """Velocity command in the body frame.

    ``source`` defaults to :attr:`MovementSource.autonomous`, the lowest rank.
    A command claims the wheel by naming a higher source, so the default is the
    one that cannot take the robot away from a human by accident. Say
    ``source=MovementSource.controller`` only if you *are* the gamepad.

    Attributes:
        x: Forward velocity, m/s.
        y: Lateral velocity, m/s, positive left.
        z_deg: Yaw rate, deg/s, positive counter-clockwise. Degrees here and
            radians elsewhere in the contract is inherited from the Go2 API.
    """

    x: float = Field(default=0.0, ge=-MAX_LINEAR_MS, le=MAX_LINEAR_MS)
    y: float = Field(default=0.0, ge=-MAX_LATERAL_MS, le=MAX_LATERAL_MS)
    z_deg: float = Field(default=0.0, ge=-MAX_YAW_RATE_DEG, le=MAX_YAW_RATE_DEG)
    source: MovementSource = MovementSource.autonomous
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


class GatewayAction(StrEnum):
    """What the gateway did to the command it forwarded.

    ``pass_through`` — nothing in the way; the command went out unchanged.
    ``limit`` — a limit zone capped one or more velocity components.
    ``slowdown`` — a slowdown zone scaled the whole command by a factor < 1.
    ``stop`` — a stop zone, a stale LiDAR scan, or the watchdog intervened.

    ``stop`` does not always mean zero. A breached stop zone is *directional*:
    the gateway strips only the velocity heading into the obstacle, leaving
    motion away from it and rotation intact, so the robot can still reverse or
    turn out of the zone rather than being trapped in it. Only an obstacle that
    surrounds the robot, or a stale scan or tripped watchdog, collapses the
    command entirely. Read ``active_zones`` for which zones are breached, not
    this, if the question is what the robot may still do.
    """

    pass_through = "pass_through"
    limit = "limit"
    slowdown = "slowdown"
    stop = "stop"


class MotionGatewayStatus(BaseModel):
    """Who is driving, and what the gateway is doing about it.

    Published on every state change rather than every tick, so it is an edge
    stream: latched, and unchanged between edges. This is the key to read when
    a command is being sent and the robot is not moving — it names the reason.

    Attributes:
        active_source: Source whose command is being forwarded. ``None`` means
            nobody has sent a fresh command, which is idle rather than stopped.
        action: What the collision monitor did to that command.
        active_zones: Names of the zones currently breached, if any.
        watchdog_tripped: The active source went silent and the gateway is
            holding the robot at zero until it speaks again.
        reason: Free text for the last transition, e.g. which source preempted
            which. Diagnostic — do not branch on it.
    """

    active_source: MovementSource | None = None
    action: GatewayAction = GatewayAction.pass_through
    active_zones: list[str] = Field(default_factory=list)
    watchdog_tripped: bool = False
    reason: str | None = None
    timestamp: datetime = Field(default_factory=_utcnow)

    @property
    def moving_allowed(self) -> bool:
        """Whether a command sent right now would reach the robot intact."""
        return not self.watchdog_tripped and self.action is not GatewayAction.stop
