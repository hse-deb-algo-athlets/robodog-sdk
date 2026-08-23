"""What the poses on ``localization/pose`` are expressed against.

A map-frame coordinate only means something while the map is the same one.
Anything persisted against that frame — a saved spot, a landmark, a patrol
route — has to record which map it was recorded in and refuse coordinates
belonging to a different one. Without that, a rebuilt or re-sessioned map turns
every stored coordinate into a confident drive to nowhere, with nothing in the
data to say so.
"""

from __future__ import annotations

from datetime import UTC, datetime

from pydantic import BaseModel, Field


def _utcnow() -> datetime:
    return datetime.now(UTC)


class MapIdentity(BaseModel):
    """Which map the current pose stream is anchored to, if any.

    Published by the localization source itself — it is the only thing that
    knows what frame it is emitting poses in. Latched, so a late joiner has it
    on subscribe, and re-stated on a slow heartbeat, so :attr:`stamp` separates
    "nothing has changed" from "the source is gone".

    Every default is "I do not know where I am", so a payload that is only
    partly understood cannot read as a valid map to drive stored coordinates
    against.
    """

    #: Identifier of the active map. Changes when the map does, which is the
    #: whole point — a consumer compares it against what it stored.
    #:
    #: ``None`` is a real and common answer: the odometry fallback has no map,
    #: and SLAM that is down or unreachable has none either. Read it as "do not
    #: trust stored map-frame coordinates", never as "unchanged". Never reuse
    #: an id across two different maps.
    map_id: str | None = None
    #: What is producing the pose, e.g. ``"mola"`` or ``"odometry"``.
    source: str = "unknown"
    #: The source's own state string, verbatim.
    state: str | None = None
    #: Source-reported localization quality, if it reports one.
    quality: float | None = None
    #: Whether the source answered at all. ``False`` alongside a populated
    #: ``map_id`` means the last map known, not a current one.
    #:
    #: Together with ``map_id`` this separates two answers that must not be
    #: conflated: ``reachable=True`` with ``map_id=None`` is a producer saying
    #: it has no map — a settled fact that waiting will not change — whereas
    #: nothing arriving at all is a producer that is absent, which a consumer
    #: sees as a stale :attr:`stamp` rather than as this payload.
    reachable: bool = False
    stamp: datetime = Field(default_factory=_utcnow)
