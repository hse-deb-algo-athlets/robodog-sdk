"""Control-lane arbitration payloads.

The arbiter is the single writer to the robot's movement input and resolves
priority between the e-stop, teleoperation, navigation and agent lanes
(ADR-010). It runs in robodog-stack; only its contract is defined here.
"""

from __future__ import annotations

from datetime import UTC, datetime
from enum import StrEnum

from pydantic import BaseModel, Field


def _utcnow() -> datetime:
    return datetime.now(UTC)


class Lane(StrEnum):
    """Command lanes, in descending order of authority.

    Declaration order defines priority; :attr:`priority` is derived from it.
    """

    estop = "estop"
    teleop = "teleop"
    nav = "nav"
    agent = "agent"

    @property
    def priority(self) -> int:
        """Rank of this lane, lowest first. ``estop`` is 0."""
        return list(Lane).index(self)


class ControlRequest(BaseModel):
    """A request to the arbiter for a command lane.

    ``ttl`` bounds the grant so that a holder which stops running releases the
    lane; a running holder renews by requesting again.
    """

    node: str  # requesting node name, as it appears in presence
    lane: Lane = Lane.agent
    ttl: float = Field(default=30.0, gt=0.0)  # s
    reason: str = ""


class ControlGrant(BaseModel):
    """The arbiter's reply to a :class:`ControlRequest`."""

    granted: bool
    lane: Lane
    holder: str | None = None  # who holds it, granted or not
    expires_at: datetime | None = None
    detail: str = ""


class ControlRelease(BaseModel):
    """Release a lane before its TTL expires."""

    node: str
    lane: Lane = Lane.agent


class ArbiterStatus(BaseModel):
    """The lane currently in control, published for observability."""

    active_lane: Lane | None = None
    holder: str | None = None
    #: True while the arbiter emits zero velocity because the active lane has
    #: gone silent.
    idle_stop: bool = False
    timestamp: datetime = Field(default_factory=_utcnow)
