"""The composite robot state — one message instead of four half-answers.

Fuses otherwise-disjoint sources into a single typed payload on
``system_state/system``, so no consumer has to string-parse or guess:

- **Nav activity** — what the running navigation skill is doing
  (:class:`~robodog_sdk.msgs.navigation.NavActivity`, from the feedback key).
- **Control mode** — who owns the robot, derived from the safety latch, the
  gamepad and the fleet runtime.
- **Order activity** — the VDA5050 order lifecycle.
- **Posture** — the Go2's own body state, mapped from its raw mode integer.

These facets are **orthogonal**, which is the whole reason they are kept
separate. A robot can be ``control=AUTO`` with ``order=IDLE`` — looking
entirely healthy — and ``posture=LYING`` at the same moment. A flat enum cannot
say that without lying about one of them. So the facets stay, and two
convenience fields are *derived* for consumers that need one answer:
:attr:`SystemState.headline` and :attr:`SystemState.ready_to_move`.

Staleness is the producer's job, not this model's: when a source goes quiet the
aggregator degrades that facet to its safe value, so the derived headline stays
honest rather than showing the last thing that was true.
"""

from __future__ import annotations

from datetime import UTC, datetime
from enum import StrEnum

from pydantic import BaseModel, Field, computed_field

from .navigation import NavActivity
from .safety import EstopPhase


def _utcnow() -> datetime:
    return datetime.now(UTC)


class Posture(StrEnum):
    """The Go2's physical body state, mapped from its raw mode integer."""

    UNKNOWN = "unknown"
    STANDING = "standing"
    LYING = "lying"
    DAMPING = "damping"  # motors soft / limp
    MOVING = "moving"

    @classmethod
    def from_go2_mode(cls, mode: int) -> Posture:
        """Map the Go2's raw ``mode`` integer to a posture.

        Unmapped integers — reserve slots, joint lock — fall through to
        :attr:`UNKNOWN`, which is the conservative answer: it keeps
        :attr:`SystemState.ready_to_move` ``False``.
        """
        return GO2_MODE_TO_POSTURE.get(mode, cls.UNKNOWN)


#: Go2 ``SportModeState.mode`` → :class:`Posture`, from the Unitree enum::
#:
#:   mode  name             posture      mode  name             posture
#:   ────  ───────────────  ──────────   ────  ───────────────  ──────────
#:    0    idle/def. stand  STANDING       7   damping          DAMPING
#:    1    balanceStand     STANDING       8   recoveryStand    STANDING
#:    2    pose             STANDING       9   reserve          (→ UNKNOWN)
#:    3    locomotion       MOVING        10   sit              LYING
#:    4    reserve          (→ UNKNOWN)   11   frontFlip        MOVING
#:    5    lieDown          LYING         12   frontJump        MOVING
#:    6    jointLock        (→ UNKNOWN)   13   frontPounc       MOVING
#:
#: ``locomotion`` is 3, not 2 — the Go1 SDK differs here and the mistake is
#: easy to inherit. 4, 6 and 9 are left unmapped on purpose: a reserve slot or
#: a joint lock must not read as ready to move. The values are
#: firmware-dependent; verify against the hardware before trusting them.
GO2_MODE_TO_POSTURE: dict[int, Posture] = {
    0: Posture.STANDING,  # idle / default stand
    1: Posture.STANDING,  # balanceStand
    2: Posture.STANDING,  # pose — standing, adjusting body pose
    3: Posture.MOVING,  # locomotion — this is "driving"
    5: Posture.LYING,  # lieDown
    7: Posture.DAMPING,  # damping — motors soft / limp
    8: Posture.STANDING,  # recoveryStand — getting back up
    10: Posture.LYING,  # sit — down, and not drive-ready
    11: Posture.MOVING,  # frontFlip
    12: Posture.MOVING,  # frontJump
    13: Posture.MOVING,  # frontPounc
}


class ControlMode(StrEnum):
    """Who currently owns the robot. Declared highest priority first."""

    ESTOP = "estop"  # emergency stop engaged — overrides everything
    MANUAL = "manual"  # a human is driving with the gamepad
    AUTO = "auto"  # autonomous / order execution


class OrderActivity(StrEnum):
    """The VDA5050 order lifecycle, as an operator sees it."""

    IDLE = "idle"  # no active order
    EXECUTING = "executing"  # driving an order
    WAITING_RELEASE = "waiting_release"  # parked at a node, awaiting the button
    STOPPING = "stopping"  # a cancelOrder is in progress


class Headline(StrEnum):
    """The single state to show at a glance.

    A priority reduction of the facets, for consumers that have one line to
    spend — a display, an LED, a status hero. Exactly one is ever current; see
    :meth:`SystemState.headline` for the order and why it runs that way.
    """

    ESTOP = "estop"  # the emergency stop is pressed
    #: No live safety source. Stops just as hard, but is not a pressed button:
    #: nobody has to go and find it, and it needs no release.
    SOURCE_LOST = "source_lost"
    #: Acknowledged; the robot is standing back up and accepts nothing yet.
    RECOVERING = "recovering"
    MANUAL = "manual"  # a human is driving on the gamepad
    LYING = "lying"  # on the floor — lying, sitting or damped
    BLOCKED = "blocked"  # the navigation skill gave up
    STALLED = "stalled"  # transiently stopped in front of an obstacle
    RELEASE = "release"  # parked, waiting for the release button
    DRIVING = "driving"  # actively moving
    AUTO = "auto"  # autonomous, standing by, nothing to do


class Location(BaseModel):
    """Where the robot is on the fleet map.

    Exactly one of the two is set while an order is active: ``node`` when
    parked *at* one, ``edge`` when driving between two.
    """

    node: str | None = None
    edge: str | None = None


class SystemState(BaseModel):
    """The composite state, on :attr:`~robodog_sdk.topics.StateTopics.system`.

    :attr:`headline` and :attr:`ready_to_move` are derived from the facets and
    emitted in the serialized payload. They are recomputed on parse, so the
    facets remain the single source of truth and a stale derived value cannot
    survive a round trip.
    """

    control: ControlMode = ControlMode.AUTO
    posture: Posture = Posture.UNKNOWN
    order: OrderActivity = OrderActivity.IDLE
    nav: NavActivity = NavActivity.NONE
    location: Location | None = None
    #: Where the e-stop latch is, straight from the safety state.
    #: :attr:`control` collapses every engaged phase to
    #: :attr:`ControlMode.ESTOP`; this keeps *why* — a pressed button, a lost
    #: source, or the recovery window after a release — which are three very
    #: different things to put in front of an operator.
    safety_phase: EstopPhase = EstopPhase.CLEAR
    timestamp: datetime = Field(default_factory=_utcnow)

    @computed_field  # type: ignore[prop-decorator]
    @property
    def ready_to_move(self) -> bool:
        """Whether the robot is in a body state that will execute motion.

        Conservative: an :attr:`Posture.UNKNOWN` posture counts as not ready.
        Authorization is deliberately excluded — this answers "can it", not
        "may it". :attr:`EstopPhase.RELEASING` reads as not ready even though
        the latch is already down, because the robot is mid-stand-up.
        """
        return (
            self.posture in (Posture.STANDING, Posture.MOVING)
            and self.control is not ControlMode.ESTOP
            and self.safety_phase is not EstopPhase.RELEASING
        )

    @computed_field  # type: ignore[prop-decorator]
    @property
    def headline(self) -> Headline:
        """The facets reduced to one value, highest priority first.

        The order follows what most needs an operator's eye: safety, then who
        holds control, then a body state that blocks motion, then navigation
        trouble, then the ordinary driving and waiting progression. First match
        wins, so exactly one headline is ever shown.
        """
        if self.control is ControlMode.ESTOP:
            # The same stop, a different story: a lost source is not a switch
            # anybody pressed, and no release will clear it.
            if self.safety_phase is EstopPhase.SOURCE_LOST:
                return Headline.SOURCE_LOST
            return Headline.ESTOP
        # Ahead of MANUAL and LYING: mid-recovery the body *is* lying, and
        # saying so hides the reason it cannot be driven yet.
        if self.safety_phase is EstopPhase.RELEASING:
            return Headline.RECOVERING
        if self.control is ControlMode.MANUAL:
            return Headline.MANUAL
        # Any non-upright state that blocks motion reads the same to an
        # operator. The facet keeps them distinct for machine consumers.
        if self.posture in (Posture.LYING, Posture.DAMPING):
            return Headline.LYING
        if self.nav is NavActivity.BLOCKED:
            return Headline.BLOCKED
        if self.nav in (NavActivity.STALLED, NavActivity.RETREATING):
            return Headline.STALLED
        if self.order is OrderActivity.WAITING_RELEASE:
            return Headline.RELEASE
        if self._is_driving:
            return Headline.DRIVING
        return Headline.AUTO

    @property
    def _is_driving(self) -> bool:
        """Actively moving: a steering skill, an order under way, or the body
        itself reporting motion."""
        return (
            self.nav in (NavActivity.CRUISING, NavActivity.ALIGNING)
            or self.order in (OrderActivity.EXECUTING, OrderActivity.STOPPING)
            or self.posture is Posture.MOVING
        )


class VdaFacet(BaseModel):
    """The control, order and location slice the VDA5050 bridge owns.

    Published on :attr:`~robodog_sdk.topics.StateTopics.vda` for the
    system-state node to fuse into :class:`SystemState`. The e-stop is
    deliberately **absent**: the safety node is its sole authority, so this
    facet only ever distinguishes :attr:`ControlMode.AUTO` from
    :attr:`ControlMode.MANUAL`. Order lifecycle and map position stay here
    because that knowledge lives in the bridge and nowhere else.
    """

    control: ControlMode = ControlMode.AUTO
    order: OrderActivity = OrderActivity.IDLE
    location: Location | None = None
    timestamp: datetime = Field(default_factory=_utcnow)
