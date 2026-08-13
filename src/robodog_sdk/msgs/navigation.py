"""Navigation payloads — the task (action) contract.

Navigation is a *task* with a lifetime, not a stream of requests. A client
submits a :data:`TaskGoal` to the ``nav/task/submit`` service and receives a
:class:`TaskHandle` carrying a ``task_id``. From then on everything about that
task is addressed by its id: :class:`TaskFeedback` while it runs,
:class:`TaskResult` once, when it finishes, and a status query for anyone who
arrives too late to have heard either.

The work itself is done by a **skill** — a named strategy inside the navigation
node (``global_nav``, ``waypoint_follow``, ``door_traverse``, ``dummy``). A goal
either names one or leaves the choice to the deployment default.

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

    The four terminal values are reported once, on the task's result key.
    :attr:`RUNNING` is what streams on the feedback key in between.
    """

    #: Submitted, not yet running. The coordinator starts a task immediately,
    #: so this is only ever seen as the answer to a status query for a task id
    #: it does not know — see :class:`TaskResult`.
    PENDING = "pending"
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
    (:func:`~robodog_sdk.topics.task_status_service`), so this carries the id
    only for symmetry and for logs.
    """

    task_id: str = ""


# ---------------------------------------------------- feedback and results


class TaskFeedback(BaseModel):
    """Progress, published on the task's feedback key while it runs.

    Roughly 10-20 Hz, driven by the skill's own control loop, so the rate is
    the skill's and not a guarantee. Not latched: subscribe before submitting,
    or accept that the first samples are missed.

    ``state`` is :attr:`TaskState.RUNNING` for the whole of a task's life —
    the terminal state arrives on the result key, not here.
    """

    task_id: str
    state: TaskState
    current_pose: Pose2D | None = None
    distance_to_goal: float | None = None
    #: Skill running this task. Skills may append a colon-separated note about
    #: what they are doing (``"waypoint_follow:stalled 3.1s"``), so compare on
    #: the part before the first colon.
    active_skill: str | None = None
    #: Pure-pursuit carrot point (x/y only). Diagnostic; safe to ignore.
    #: ``None`` whenever the skill is not actively steering.
    lookahead_point: Pose2D | None = None
    timestamp: datetime = Field(default_factory=_utcnow)


class TaskResult(BaseModel):
    """The one terminal payload for a task.

    Published on the task's result key, and returned by the status query for
    as long as the coordinator remembers the task. A status query for a task
    it does not remember — never submitted, or evicted from its history —
    answers :attr:`TaskState.PENDING`, so treat that as "unknown", never as
    "about to start".
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


class PlannedPath(BaseModel):
    """Planner output: a dense, ordered waypoint list for the tracker.

    Internal to the navigation node — planner and tracker are in one process
    and no key carries this today. It is in the contract because it is the
    interchange type between the two halves of a skill, and a project writing
    its own planner or tracker needs to name it.
    """

    task_id: str
    waypoints: list[PathWaypoint]


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
