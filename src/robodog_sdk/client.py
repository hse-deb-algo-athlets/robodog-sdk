"""High-level client for the Robodog contract.

:class:`RobotClient` publishes and subscribes on the keys declared in
:mod:`robodog_sdk.topics`, with the same payload types and the same arbiter
priority as any other node. It holds no privileged channel; using the contract
directly is equivalent::

    cmd = publish(MotionTopics.move_agent)

Scope (ADR-010): message construction, lane handshake, latched state, and
request correlation. No behaviours, no retry or reconnection policy, no caching
that masks staleness.
"""

from __future__ import annotations

import asyncio
import contextlib
import time
import uuid
from collections.abc import AsyncIterator
from typing import TYPE_CHECKING, Generic, TypeVar

from zenode import Node, TransportConfig

from .msgs.control import ControlRelease, ControlRequest, Lane
from .msgs.motion import (
    ActionCommand,
    ActionType,
    EmergencyStop,
    EmergencyStopCommand,
    MovementCommand,
    MovementSource,
    TiltBody,
)
from .msgs.navigation import (
    NavigationCancel,
    NavigationRequest,
    NavigationSegment,
    NavigationState,
    NavigationStatus,
    Pose2D,
)
from .msgs.robot import BatteryState, ConnectionStatus, OdometryState, RobotHighState
from .topics import ControlServices, ControlTopics, MotionTopics, NavTopics, PoseTopics, StateTopics

if TYPE_CHECKING:
    from types import TracebackType

T = TypeVar("T")

#: Default republish rate for :meth:`RobotClient.driving`, in Hz. Well inside
#: ``COMMAND_MAX_AGE``, so a dropped sample does not interrupt motion.
DRIVE_RATE_HZ = 10.0


class Latest(Generic[T]):
    """Last value received on a topic, with the time since it arrived.

    Age is exposed rather than folded into the value: "never received" and
    "received 40 seconds ago" are different conditions and need different
    handling.
    """

    __slots__ = ("_at", "value")

    def __init__(self) -> None:
        self.value: T | None = None
        self._at: float | None = None

    def update(self, value: T) -> None:
        """Record a newly received value. Called by :class:`RobotClient`."""
        self.value = value
        self._at = time.monotonic()

    @property
    def age(self) -> float:
        """Seconds since the last message, or ``inf`` if none has arrived."""
        return float("inf") if self._at is None else time.monotonic() - self._at

    def fresh(self, within: float = 1.0) -> bool:
        """Whether a value has arrived within the last ``within`` seconds."""
        return self.value is not None and self.age <= within

    def __bool__(self) -> bool:
        return self.value is not None


class StateView:
    """Robot state as last received by this process.

    Every field is a :class:`Latest`; all are backed by latched topics, so they
    are populated shortly after the client is created rather than on the next
    publish.
    """

    def __init__(self) -> None:
        self.odometry: Latest[OdometryState] = Latest()
        self.localization: Latest[OdometryState] = Latest()
        self.battery: Latest[BatteryState] = Latest()
        self.highstate: Latest[RobotHighState] = Latest()
        self.connection: Latest[ConnectionStatus] = Latest()
        self.nav: Latest[NavigationStatus] = Latest()


class RobotClient:
    """Command the robot and read its state over the contract.

    Create it in a node's ``on_start``, once the session exists::

        class Wanderer(Node):
            name = "wanderer"

            async def on_start(self) -> None:
                self.robot = RobotClient(self)

    For scripts and notebooks, :meth:`connect` provides a client that owns its
    own session::

        async with RobotClient.connect() as robot:
            async with robot.control(), robot.driving(x=0.3):
                await asyncio.sleep(2)

    Args:
        node: The node whose session, publishers and subscriptions are used.
        source: Provenance stamped on outgoing movement commands. Diagnostic
            only — priority follows the lane, not this field.
    """

    def __init__(self, node: Node, *, source: MovementSource = MovementSource.autonomous) -> None:
        self._node = node
        self._source = source
        self.state = StateView()

        self._move = node.publisher(MotionTopics.move_agent)
        self._estop = node.publisher(MotionTopics.estop)
        self._action = node.publisher(PoseTopics.action)
        self._tilt = node.publisher(PoseTopics.tilt_body)
        self._nav_request = node.publisher(NavTopics.request)
        self._nav_cancel = node.publisher(NavTopics.cancel)
        self._control_release = node.publisher(ControlTopics.release)

        node.subscribe(StateTopics.odometry, self.state.odometry.update, mode="latest")
        node.subscribe(StateTopics.battery, self.state.battery.update, mode="latest")
        node.subscribe(StateTopics.highstate, self.state.highstate.update, mode="latest")
        node.subscribe(StateTopics.connection, self.state.connection.update, mode="latest")
        node.subscribe(NavTopics.status, self._on_nav_status, mode="latest")

        self._nav_waiters: dict[str, asyncio.Future[NavigationState]] = {}

    # ------------------------------------------------------------ standalone

    @classmethod
    @contextlib.asynccontextmanager
    async def connect(
        cls,
        *,
        transport: TransportConfig | None = None,
        name: str = "robot-client",
    ) -> AsyncIterator[RobotClient]:
        """Yield a client backed by a session of its own.

        For scripts and notebooks. Within a node, construct
        ``RobotClient(node)`` instead; this would open a second session in the
        same process.

        Args:
            transport: Transport configuration. Defaults to the resolved
                deployment configuration.
            name: Node name used for presence and for lane requests.
        """

        class _ClientNode(Node):
            pass

        _ClientNode.name = name
        node = _ClientNode(transport=transport)
        await node.start()
        try:
            yield cls(node)
        finally:
            await node.shutdown()

    # --------------------------------------------------------------- control

    @contextlib.asynccontextmanager
    async def control(
        self, *, lane: Lane = Lane.agent, ttl: float = 30.0, reason: str = ""
    ) -> AsyncIterator[None]:
        """Hold a command lane for the duration of the block.

        Commands sent without holding the lane are still delivered, but rank
        below the current holder. The lane is released on exit, after a stop.

        Args:
            lane: Lane to request. Defaults to :attr:`Lane.agent`.
            ttl: Seconds the grant remains valid if not renewed.
            reason: Free text recorded by the arbiter.

        Raises:
            PermissionError: The lane is held by another node.
        """
        grant = await self._node.call(
            ControlServices.acquire,
            ControlRequest(node=self._node.name, lane=lane, ttl=ttl, reason=reason),
        )
        if not grant.granted:
            raise PermissionError(
                f"lane {lane.value!r} not granted"
                + (f" — held by {grant.holder!r}" if grant.holder else "")
                + (f": {grant.detail}" if grant.detail else "")
            )
        try:
            yield
        finally:
            self.halt()
            self._control_release.put(ControlRelease(node=self._node.name, lane=lane))

    # -------------------------------------------------------------- commands

    def move(self, x: float = 0.0, y: float = 0.0, z_deg: float = 0.0) -> None:
        """Send a single velocity command, in the body frame.

        The command expires after ``COMMAND_MAX_AGE``. Use :meth:`driving` for
        sustained motion.

        Args:
            x: Forward velocity, m/s.
            y: Lateral velocity, m/s, positive left.
            z_deg: Yaw rate, deg/s, positive counter-clockwise.

        Raises:
            pydantic.ValidationError: A value lies outside
                :mod:`robodog_sdk.limits`.
        """
        self._move.put(MovementCommand(x=x, y=y, z_deg=z_deg, source=self._source))

    def halt(self) -> None:
        """Command zero velocity. See :meth:`emergency_stop` for the e-stop."""
        self._move.put(MovementCommand(source=self._source))

    @contextlib.asynccontextmanager
    async def driving(
        self, x: float = 0.0, y: float = 0.0, z_deg: float = 0.0, *, rate_hz: float = DRIVE_RATE_HZ
    ) -> AsyncIterator[None]:
        """Maintain a velocity for the duration of the block, then stop.

        Republishes at ``rate_hz`` because a single command expires after
        ``COMMAND_MAX_AGE``. The pump is cancelled and a zero velocity sent on
        exit, including on exception.

        Args:
            x: Forward velocity, m/s.
            y: Lateral velocity, m/s, positive left.
            z_deg: Yaw rate, deg/s, positive counter-clockwise.
            rate_hz: Republish rate.
        """
        period = 1.0 / rate_hz

        async def _pump() -> None:
            while True:
                self.move(x, y, z_deg)
                await asyncio.sleep(period)

        task = self._node.spawn(_pump(), name="driving")
        try:
            yield
        finally:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task
            self.halt()

    def action(self, action: ActionType) -> None:
        """Trigger a discrete action, such as ``stand_up`` or ``sit_down``."""
        self._action.put(ActionCommand(action=action))

    def tilt(self, pitch_deg: float = 0.0, roll_deg: float = 0.0, yaw_deg: float = 0.0) -> None:
        """Set body orientation while standing, in degrees."""
        self._tilt.put(TiltBody(pitch_deg=pitch_deg, roll_deg=roll_deg, yaw_deg=yaw_deg))

    def emergency_stop(self) -> None:
        """Trigger the emergency stop.

        There is no counterpart: an e-stop is cleared at the physical button,
        not through this API.
        """
        self._estop.put(EmergencyStopCommand(command=EmergencyStop.stop))

    # ------------------------------------------------------------ navigation

    async def navigate_to(
        self,
        x: float,
        y: float,
        heading: float | None = None,
        *,
        timeout: float | None = None,
        max_speed: float | None = None,
    ) -> NavigationState:
        """Request navigation to a point and wait for the outcome.

        Args:
            x: Target x in the ``map`` frame, m.
            y: Target y in the ``map`` frame, m.
            heading: Required orientation on arrival, rad. ``None`` leaves it
                unconstrained.
            timeout: Seconds to wait for a terminal state. ``None`` waits
                indefinitely.
            max_speed: Segment speed limit, m/s. ``None`` uses the deployment
                default.

        Returns:
            The terminal state: ``ARRIVED_FINAL``, ``BLOCKED`` or ``FAILED``.

        Raises:
            TimeoutError: No terminal state arrived within ``timeout``.

        Cancelling the awaiting task stops the wait, not the robot; call
        :meth:`cancel_navigation` to stop navigating.
        """
        request_id = uuid.uuid4().hex
        loop = asyncio.get_running_loop()
        waiter: asyncio.Future[NavigationState] = loop.create_future()
        self._nav_waiters[request_id] = waiter

        self._nav_request.put(
            NavigationRequest(
                request_id=request_id,
                segments=[
                    NavigationSegment(
                        target=Pose2D(x=x, y=y, theta=heading or 0.0),
                        orientation_at_target=heading,
                        max_speed=max_speed,
                    )
                ],
            )
        )
        try:
            return await asyncio.wait_for(waiter, timeout)
        finally:
            self._nav_waiters.pop(request_id, None)

    def cancel_navigation(self, request_id: str = "", reason: str = "") -> None:
        """Abandon a navigation request and stop.

        Args:
            request_id: Request to cancel. Empty cancels whatever is running.
            reason: Free text recorded by the navigation node.
        """
        self._nav_cancel.put(NavigationCancel(request_id=request_id or None, reason=reason))

    def _on_nav_status(self, status: NavigationStatus) -> None:
        self.state.nav.update(status)
        if status.request_id is None or not status.state.is_terminal:
            return
        waiter = self._nav_waiters.get(status.request_id)
        if waiter is not None and not waiter.done():
            waiter.set_result(status.state)

    # ------------------------------------------------------------- readiness

    async def wait_until_ready(self, *nodes: str, timeout: float = 10.0) -> None:
        """Wait until the named nodes hold presence tokens.

        Args:
            nodes: Node names to wait for. Defaults to ``arbiter``.
            timeout: Seconds to wait.

        Raises:
            TimeoutError: A node was still absent when the timeout elapsed.
        """
        await self._node.wait_for_nodes(list(nodes) or ["arbiter"], timeout=timeout)

    # ------------------------------------------------------------- lifecycle

    async def __aenter__(self) -> RobotClient:
        return self

    async def __aexit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.halt()
