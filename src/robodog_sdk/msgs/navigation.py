"""Navigation payloads — the task (action) contract.

Navigation is a *task* with a lifetime, not a stream of requests. A client
submits a :data:`TaskGoal` to the ``nav/task/submit`` service and receives a
:class:`TaskHandle` carrying a ``task_id``. From then on everything about that
task is addressed by its id: :class:`TaskFeedback` while it runs,
:class:`TaskResult` once, when it finishes, and a status query for anyone who
arrives too late to have heard either.

The work itself is done by a **skill** — a named strategy inside the navigation
node (``global_nav``, ``corridor_assist``, ``waypoint_follow``,
``door_traverse``, ``dummy``). A goal either names one or leaves the choice to
the deployment default.

Three fields describe "what is happening" at different altitudes and are worth
keeping apart. :class:`TaskState` is the *lifecycle* — running, or one of four
endings. :class:`NavActivity` is the skill's transient sub-state — cruising,
aligning, stalled. ``active_skill`` is merely which skill is driving.

:class:`Pose2D` retains its vector arithmetic, which belongs to the type rather
than to any one consumer. The planner helpers that required numpy are not
included, keeping the contract free of numeric dependencies; the occupancy grid
the planner works on is in :mod:`robodog_sdk.msgs.occupancy`.
"""

from __future__ import annotations

import math
from datetime import UTC, datetime
from enum import StrEnum
from typing import Annotated, Literal

from pydantic import BaseModel, Field, RootModel


def _utcnow() -> datetime:
    return datetime.now(UTC)


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Yaw, the rotation about z, from an ``(qx, qy, qz, qw)`` quaternion.

    Part of the contract so that every consumer of orientation derives the
    same value.
    """
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


class Pose2D(BaseModel):
    """A planar pose in the ``map`` frame. Angles in radians."""

    x: float
    y: float
    theta: float

    def __add__(self, other: Pose2D) -> Pose2D:
        return Pose2D(
            x=self.x + other.x,
            y=self.y + other.y,
            theta=normalize_angle(self.theta + other.theta),
        )

    def __sub__(self, other: Pose2D) -> Pose2D:
        return Pose2D(
            x=self.x - other.x,
            y=self.y - other.y,
            theta=normalize_angle(self.theta - other.theta),
        )

    def __abs__(self) -> float:
        """Euclidean distance from the origin (ignores theta)."""
        return math.hypot(self.x, self.y)

    def __mul__(self, scalar: float) -> Pose2D:
        return Pose2D(x=self.x * scalar, y=self.y * scalar, theta=self.theta * scalar)

    def __rmul__(self, scalar: float) -> Pose2D:
        return self.__mul__(scalar)

    @property
    def distance(self) -> float:
        """Same as ``abs()``, but more readable in context."""
        return abs(self)

    @property
    def bearing(self) -> float:
        """Angle from the origin to this point."""
        return math.atan2(self.y, self.x)

    def distance_to(self, other: Pose2D) -> float:
        return abs(self - other)

    def bearing_to(self, other: Pose2D) -> float:
        return (other - self).bearing


# ---------------------------------------------------------------- lifecycle


class TaskState(StrEnum):
    """Where a navigation task is in its life.

    :attr:`RUNNING` is what streams on the feedback key; the four terminal
    values are reported once, on the result key. There is no queued state —
    a submit starts the skill immediately, so a task is always either running
    or finished.

    A status query for a task the coordinator has no record of — never
    submitted, or aged out of its bounded history — is answered on the Zenoh
    **error channel**, not with a placeholder value here. "Unknown" is not a
    lifecycle state, and giving it one invites a client to treat a forgotten
    task as one about to start.
    """

    RUNNING = "running"
    #: Terminal — the goal was reached.
    SUCCEEDED = "succeeded"
    #: Terminal — the skill could not carry the goal out (no pose source, no
    #: map, an unhandled error).
    FAILED = "failed"
    #: Terminal — a client cancelled, or another submit preempted it.
    CANCELED = "canceled"
    #: Terminal — the skill gave up in the face of the world: an obstacle that
    #: did not clear, no plan through the map, no forward progress. Distinct
    #: from :attr:`FAILED` in that nothing is wrong with the robot; retrying
    #: later, or from somewhere else, may well work.
    BLOCKED = "blocked"

    @property
    def is_terminal(self) -> bool:
        """Whether the task has finished, successfully or not.

        :meth:`~robodog_sdk.RobotClient.navigate_to` completes on these states
        and returns the result that carried one.
        """
        return self in (
            TaskState.SUCCEEDED,
            TaskState.FAILED,
            TaskState.CANCELED,
            TaskState.BLOCKED,
        )


class NavActivity(StrEnum):
    """What a running task's skill is *currently* doing.

    Orthogonal to :class:`TaskState`: ``state`` stays :attr:`TaskState.RUNNING`
    for the whole of a task's life while ``activity`` swings between these as
    the skill drives, aligns, stalls in front of an obstacle or backs off.

    :attr:`STALLED` and :attr:`RETREATING` are *transient* — the skill is still
    trying. :attr:`BLOCKED` here mirrors a give-up in the streamed feedback;
    the authoritative verdict is always the :class:`TaskResult` carrying
    :attr:`TaskState.BLOCKED`.
    """

    NONE = "none"  # not steering / between phases
    CRUISING = "cruising"  # following the path normally
    ALIGNING = "aligning"  # rotating in place toward the path or goal heading
    STALLED = "stalled"  # stopped in front of an obstacle, still trying
    RETREATING = "retreating"  # backing off to the last passed waypoint
    PAUSED = "paused"  # held by an external stop (e-stop), not by the world
    MANUAL = "manual"  # held because a human took over on the gamepad
    BLOCKED = "blocked"  # skill gave up — see the terminal TaskResult


class EstopPolicy(StrEnum):
    """What an e-stop does to a task — chosen by the client that submitted it.

    Travels as the ``?on_estop=`` parameter on the submit rather than in the
    payload: whoever submitted a task is the only one who knows whether it
    survives a stop, so nav does not guess.

    :attr:`CANCEL` is the default because most routes have no owner watching
    them. A goal sent from a script or a map UI is a one-shot instruction, and
    silently resuming it minutes after a human walked over and hit the button
    is not what anyone meant by it. Pass :attr:`HOLD` only if this process
    owns the mission and handles the recovery itself.
    """

    #: Discard the task when the e-stop engages. It will not resume.
    CANCEL = "cancel"
    #: Hold the task; it resumes once motion is permitted again.
    HOLD = "hold"


# -------------------------------------------------------------------- goals


class NavigateToPoseGoal(BaseModel):
    """Drive to a single target pose."""

    kind: Literal["navigate_to_pose"] = "navigate_to_pose"
    target: Pose2D
    #: m/s. ``None`` uses the skill's configured cruise speed.
    max_speed: float | None = None
    #: m — how close to ``target`` counts as arrived.
    arrival_deviation: float = 0.15
    #: rad — heading tolerance, only meaningful with ``orientation_at_target``.
    arrival_orientation_deviation: float = 0.1
    #: rad. ``None`` leaves the final heading a don't-care.
    orientation_at_target: float | None = None
    #: Skill to run this goal. ``None`` uses the deployment's default skill.
    skill: str | None = None


class NavigateThroughPosesGoal(BaseModel):
    """Drive through an ordered list of poses; the last one is the target.

    The intermediate poses are a *route*, not a suggestion: this is the shape a
    fleet manager (VDA5050) uses when it has already decided which way to go.
    Skills that plan for themselves may only honour the final pose — see the
    note on skills in :mod:`robodog_sdk.topics`.
    """

    kind: Literal["navigate_through_poses"] = "navigate_through_poses"
    poses: list[Pose2D] = Field(min_length=1)
    max_speed: float | None = None
    arrival_deviation: float = 0.15
    arrival_orientation_deviation: float = 0.1
    final_orientation: float | None = None
    skill: str | None = None
    #: Half-width in metres of the released corridor around the ``poses``
    #: polyline, for the case where a human takes the gamepad mid-route. When
    #: they let go the skill measures how far off the line the robot ended up:
    #: inside the corridor it resumes, beyond it the task ends
    #: :attr:`TaskState.BLOCKED` so the client can decide what happens next.
    #: ``None`` disables the check and the skill simply resumes from wherever
    #: it is — which is the right answer for a one-off local goal, and the
    #: wrong one for a fleet order whose route was chosen for a reason.
    corridor_deviation_m: float | None = None


#: What ``nav/task/submit`` accepts, discriminated on ``kind``.
TaskGoal = Annotated[
    NavigateToPoseGoal | NavigateThroughPosesGoal,
    Field(discriminator="kind"),
]


class TaskGoalEnvelope(RootModel[TaskGoal]):
    """A :data:`TaskGoal` as a concrete model, for the submit service.

    :data:`TaskGoal` is a discriminated union rather than a class, which a
    codec cannot be derived from. The envelope adds nothing to the wire — a
    root model serializes as its content — and exists so the service
    declaration has a type to name.
    """

    @property
    def goal(self) -> NavigateToPoseGoal | NavigateThroughPosesGoal:
        """The goal itself. Alias for ``root``, which reads poorly in context."""
        return self.root


# ------------------------------------------------------- submit and cancel


class TaskHandle(BaseModel):
    """Reply to ``nav/task/submit``.

    ``accepted=False`` means nothing was started: another task is running and
    the submit did not ask to preempt it, the named skill does not exist, or
    the goal did not parse. ``task_id`` is empty in that case — there is no
    task to refer to.
    """

    task_id: str
    accepted: bool
    reason: str | None = None


class CancelRequest(BaseModel):
    """Ask the coordinator to abandon a task and stop."""

    task_id: str


class CancelAck(BaseModel):
    """Reply to ``nav/task/cancel``.

    ``canceled=False`` covers both "already finished" and "never heard of it";
    ``reason`` distinguishes them in prose, not in a field.
    """

    task_id: str
    canceled: bool
    reason: str | None = None


class TaskStatusRequest(BaseModel):
    """Request payload for the per-task status query.

    The task is identified by the key the query goes to
    (:func:`~robodog_sdk.topics.task_status_service`), which is also all the
    coordinator reads. This carries the id for symmetry and for logs.
    """

    task_id: str = ""


# ---------------------------------------------------- feedback and results


class TaskFeedback(BaseModel):
    """Progress, published on the task's feedback key while it runs.

    Roughly 10 Hz, driven by the skill's own control loop, so the rate is the
    skill's and not a guarantee. Not latched: subscribe before submitting, or
    accept that the first samples are missed.

    ``state`` is narrowed to :attr:`TaskState.RUNNING`. Feedback streams only
    while the task is alive, so that is the only lifecycle value this key can
    carry, and the narrowing is what makes it impossible for a feedback frame
    and the :class:`TaskResult` to disagree — a stray terminal value on the
    wire is rejected at validation rather than believed.
    """

    task_id: str
    state: Literal[TaskState.RUNNING] = TaskState.RUNNING
    current_pose: Pose2D | None = None
    distance_to_goal: float | None = None
    #: Which skill is driving — a plain name, nothing appended. Read
    #: :attr:`activity` for what it is doing and :attr:`note` for the prose.
    active_skill: str | None = None
    #: The skill's transient sub-state. Machine-readable, unlike :attr:`note`.
    activity: NavActivity = NavActivity.CRUISING
    #: Optional human-readable detail for the current :attr:`activity`, e.g.
    #: ``"stalled 3.1s"``. Never parse this — branch on ``activity`` instead.
    note: str | None = None
    #: Pure-pursuit carrot point (x/y only). Diagnostic; safe to ignore.
    #: ``None`` whenever the skill is not actively steering.
    lookahead_point: Pose2D | None = None
    #: Route progress on a multi-waypoint goal: the 0-based index of the most
    #: recently *reached* original waypoint — ``goal.poses[i]`` of a
    #: :class:`NavigateThroughPosesGoal`. ``None`` until the first is reached,
    #: and for skills that do not track segments at all.
    current_segment_index: int | None = None
    #: How many waypoints the route has, alongside
    #: :attr:`current_segment_index`. This pair is what lets a fleet bridge
    #: report each node as it is passed instead of only at the end of the run.
    total_segments: int | None = None
    timestamp: datetime = Field(default_factory=_utcnow)


class TaskResult(BaseModel):
    """The one terminal payload for a task.

    On the result key ``state`` is always terminal, and the message is
    published exactly once. The status query reuses this model to answer late
    lookups, and there it may also carry :attr:`TaskState.RUNNING` — the task
    is still going. So a subscriber of the result key may assume terminal; a
    caller of :func:`~robodog_sdk.topics.task_status_service` may not.

    A lookup for a task the coordinator has no record of is answered on the
    Zenoh error channel and surfaces as an exception, never as a result with a
    placeholder state.
    """

    task_id: str
    state: TaskState
    message: str | None = None
    timestamp: datetime = Field(default_factory=_utcnow)

    @property
    def succeeded(self) -> bool:
        return self.state is TaskState.SUCCEEDED


# ---------------------------------------------------------- planner output


class PathWaypoint(BaseModel):
    """A single waypoint on a planned path."""

    pose: Pose2D
    speed: float  # m/s at this waypoint
    is_segment_boundary: bool = False
    must_stop: bool = False
    allowed_deviation: float = 0.15  # m
    allowed_orientation_deviation: float = 0.1  # rad
    #: Which controller is meant to drive this waypoint. Normally unset — the
    #: whole path belongs to :attr:`PlannedPath.skill`. ``corridor_assist``
    #: fills it in per waypoint so one plan can say "reactive from here to
    #: there, planned either side of it", which is also what lets a viewer
    #: colour the route before it is driven rather than after.
    skill: str | None = None


class PlannedPath(BaseModel):
    """Planner output: a dense, ordered waypoint list for the tracker.

    Published on :attr:`~robodog_sdk.topics.NavTopics.path` once per plan, and
    again on a replan or a retreat. Visualisation only — nothing in the control
    loop subscribes to it, and a consumer that misses one has missed a picture,
    not a command.
    """

    task_id: str
    waypoints: list[PathWaypoint]
    #: Which skill produced this path. A path is single-skill by construction;
    #: ``corridor_assist`` republishes as it hands the wheel between planned
    #: and reactive driving, so a subscriber sees the mode change as a new
    #: path rather than having to infer it.
    skill: str | None = None


# ------------------------------------------------------------------ safety


class CollisionZoneEvent(BaseModel):
    """An obstacle entered or left one of the motion gateway's zones.

    Published on transitions only — once on breach, once when the zone clears —
    rather than on every detection cycle. Zones are named by the deployment
    (``"stop"``, ``"slowdown"``, ...); what each one does to a command is the
    gateway's business, and a consumer should react to the name it configured
    rather than assume the set.
    """

    active: bool
    zone_name: str
    distance_m: float | None = None
    timestamp: datetime = Field(default_factory=_utcnow)
