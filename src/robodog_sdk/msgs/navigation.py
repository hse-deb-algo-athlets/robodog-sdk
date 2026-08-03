"""Navigation payloads.

:class:`Pose2D` retains its vector arithmetic, which belongs to the type rather
than to any one consumer. The planner helpers that required numpy are not
included, keeping the contract free of numeric dependencies.
"""

from __future__ import annotations

import math
from datetime import UTC, datetime
from enum import StrEnum

from pydantic import BaseModel, Field


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


class Corridor(BaseModel):
    """Lateral bounds for a segment, as deviation from its straight line."""

    left_width: float  # m, positive = left of travel direction
    right_width: float  # m, positive = right of travel direction


class NavigationSegment(BaseModel):
    """A single segment to traverse.

    Corresponds to one VDA5050 edge and its end node; the planner itself has
    no VDA5050 dependency.
    """

    target: Pose2D
    max_speed: float | None = None  # m/s, None = deployment default
    corridor: Corridor | None = None
    allowed_deviation: float = 0.15  # m — how close counts as "arrived"
    allowed_orientation_deviation: float = 0.1  # rad
    must_stop: bool = True
    orientation_at_target: float | None = None  # rad
    rotation_allowed_on_segment: bool = True


class NavigationRequest(BaseModel):
    """A navigation request: one segment to reach a point, several for a path."""

    request_id: str  # correlates with NavigationStatus
    segments: list[NavigationSegment] = Field(min_length=1)
    lookahead_segments: int = 1


class NavigationCancel(BaseModel):
    """Abandon a navigation request and stop.

    A distinct message rather than an empty :class:`NavigationRequest`, which
    is invalid and would be indistinguishable from a malformed request.
    """

    #: The request to cancel; ``None`` cancels whatever is running.
    request_id: str | None = None
    reason: str = ""
    timestamp: datetime = Field(default_factory=_utcnow)


class PathWaypoint(BaseModel):
    """A single waypoint on the planned path."""

    pose: Pose2D
    speed: float  # m/s at this waypoint
    is_segment_boundary: bool = False
    must_stop: bool = False
    allowed_deviation: float = 0.15  # m
    allowed_orientation_deviation: float = 0.1  # rad


class PlannedPath(BaseModel):
    """Planner output: a dense, ordered waypoint list for the executor."""

    request_id: str
    waypoints: list[PathWaypoint]


class NavigationState(StrEnum):
    IDLE = "idle"
    FOLLOWING = "following"
    ARRIVED_SEGMENT = "arrived_segment"
    ARRIVED_FINAL = "arrived_final"
    BLOCKED = "blocked"
    FAILED = "failed"

    @property
    def is_terminal(self) -> bool:
        """Whether the request has finished, successfully or not.

        :meth:`~robodog_sdk.RobotClient.navigate_to` completes on these
        states and returns which one was reached.
        """
        return self in (
            NavigationState.ARRIVED_FINAL,
            NavigationState.BLOCKED,
            NavigationState.FAILED,
        )


class NavigationStatus(BaseModel):
    timestamp: datetime = Field(default_factory=_utcnow)
    state: NavigationState
    current_pose: Pose2D | None = None
    distance_to_target: float | None = None
    distance_to_final: float | None = None
    current_segment_index: int | None = None
    request_id: str | None = None
    #: Pure-pursuit carrot point (x/y only). Diagnostic; safe to ignore.
    lookahead_point: Pose2D | None = None


class ProtectiveFieldEvent(BaseModel):
    """An obstacle entered or left the protective (stop) field.

    Published on transitions only — once on breach, once when the field
    clears — rather than on every detection cycle.
    """

    active: bool
    distance_m: float | None = None
    source: str = "obstacle_detector"
    timestamp: datetime = Field(default_factory=_utcnow)
