"""The safety latch — the one payload that decides whether the robot may move.

Published by the safety aggregator on ``safety/state`` as a *continuous
level*: on every change **and** at a fixed heartbeat. Deliberately not an event
("a stop happened"), because that model loses the truth on a single dropped
message — a level re-asserts itself within one tick and a reconnecting consumer
resyncs without asking.

Consumers **fail safe on silence**. No fresh frame within the deadline, a lost
liveliness token, or ``source_alive=False`` all mean the same thing as a
pressed button: stopped. Every default here is chosen so that an unpopulated
:class:`SafetyState` reads as stopped with no live source — nothing accidentally
permits motion by forgetting to fill a field in.

Read :attr:`SafetyState.motion_permitted`, not ``estop``. The two differ for a
whole phase of the recovery, and the difference is a robot lying on the floor.
"""

from __future__ import annotations

from datetime import UTC, datetime
from enum import StrEnum

from pydantic import BaseModel, Field, computed_field


def _utcnow() -> datetime:
    return datetime.now(UTC)


class EstopPhase(StrEnum):
    """Where the e-stop latch is in its clear/latch cycle.

    :attr:`STOPPED` and :attr:`AWAITING_RELEASE` are the *button* cycle: a
    human pressed the switch and a human has to acknowledge it.
    :attr:`SOURCE_LOST` is the other reason motion is denied — the safety
    source stopped talking — and is kept apart on purpose. It engages the very
    same stop, but it is not a pressed button, it clears itself the moment
    frames resume, and telling an operator that the emergency stop is pressed
    over a dropped heartbeat sends them hunting a switch nobody touched.
    """

    CLEAR = "clear"  # not engaged
    STOPPED = "stopped"  # button latched
    AWAITING_RELEASE = "awaiting_release"  # pulled out, awaiting the release press
    #: Acknowledged, robot coming back up — motion still denied. See
    #: :attr:`SafetyState.motion_permitted`.
    RELEASING = "releasing"
    #: No live safety source (missing, stale, or liveliness dropped). A
    #: fail-safe stop, *not* a button press: it needs no release and clears
    #: itself as soon as the source is live again.
    SOURCE_LOST = "source_lost"


class SafetyState(BaseModel):
    """The continuous safety latch, on ``safety/state``.

    ``motion_permitted`` is derived and serialized, so a consumer reading the
    wire sees it too. The fail-safe reasoning about *silence* still belongs to
    the consumer — a payload cannot tell you it stopped arriving.
    """

    #: The latch. Stays ``True`` through :attr:`EstopPhase.STOPPED` and
    #: :attr:`EstopPhase.AWAITING_RELEASE`, and clears once the release is
    #: confirmed. Fail-safe default: engaged.
    #:
    #: Deliberately ``False`` in :attr:`EstopPhase.RELEASING`. This is the
    #: level the robot bridge obeys, and the bridge's recovery — standing back
    #: up — is what *starts* on the falling edge. Holding it engaged there
    #: would mean the robot never gets up, so never reports ready, so never
    #: leaves ``RELEASING``. Motion during that window is denied by
    #: :attr:`motion_permitted` instead.
    estop: bool = True
    #: Whether the upstream safety source is delivering fresh frames.
    #: Fail-safe default: no source seen yet.
    source_alive: bool = False
    phase: EstopPhase = EstopPhase.STOPPED
    #: Monotonic counter from the source, so a consumer detects loss and
    #: reordering rather than trusting arrival order.
    seq: int = 0
    timestamp: datetime = Field(default_factory=_utcnow)

    @computed_field  # type: ignore[prop-decorator]
    @property
    def motion_permitted(self) -> bool:
        """May the robot be driven right now?

        Needs a live source, the latch clear, *and* the recovery finished.
        :attr:`EstopPhase.RELEASING` is the gap this closes: the stop is
        acknowledged and ``estop`` has already dropped so the robot can stand
        back up, but it is still on the floor. Read this, not ``estop``, to
        decide whether to drive.
        """
        return self.source_alive and not self.estop and self.phase is not EstopPhase.RELEASING


class ButtonEvent(BaseModel):
    """A momentary press on a safety panel — release (Freigabe) or cancel
    (Abbrechen).

    Unlike :class:`SafetyState` this is an *event*, not a level: emitted once
    per press, and possibly re-sent a few times for reliability over a lossy
    link. Deduplicate on ``(source_id, seq)`` — a burst of re-sends of one
    press must not read as several presses.
    """

    #: Which panel sent it. Matches the ``{source_id}`` of that panel's
    #: per-source latch key, so an event can be traced back to its source.
    source_id: str
    #: Monotonic counter shared with this source's :class:`SafetyState`
    #: stream, which is what makes the deduplication above possible.
    seq: int = 0
    timestamp: datetime = Field(default_factory=_utcnow)
