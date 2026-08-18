"""High-level client for the Robodog contract.

:class:`RobotClient` publishes and subscribes on the keys declared in
:mod:`robodog_sdk.topics`, with the same payload types and the same gateway
priority as any other node. It holds no privileged channel; using the contract
directly is equivalent::

    cmd = publish(MotionTopics.request)

Scope (ADR-010): message construction, latched state, and task correlation. No
behaviours, no retry or reconnection policy, no caching that masks staleness.
"""

from __future__ import annotations

import asyncio
import contextlib
import time
from collections.abc import AsyncIterator, Sequence
from typing import TYPE_CHECKING, Generic, TypeVar

from zenode import Node, ServiceError, ServiceTimeout, TransportConfig
from zenode.service import call_service

from .msgs.motion import (
    ActionCommand,
    ActionType,
    MotionGatewayStatus,
    MovementCommand,
    MovementSource,
    TiltBody,
)
from .msgs.navigation import (
    CancelAck,
    CancelRequest,
    EstopPolicy,
    NavigateThroughPosesGoal,
    NavigateToPoseGoal,
    Pose2D,
    TaskFeedback,
    TaskGoal,
    TaskGoalEnvelope,
    TaskHandle,
    TaskResult,
    TaskStatusRequest,
)
from .msgs.robot import BatteryState, OdometryState, RobotHighState
from .msgs.safety import ButtonEvent, SafetyState
from .msgs.system_state import SystemState
from .topics import (
    ControlTopics,
    LocalizationTopics,
    MotionTopics,
    NavServices,
    NavTopics,
    PoseTopics,
    SafetyTopics,
    StateTopics,
    task_status_service,
)

if TYPE_CHECKING:
    from types import TracebackType

T = TypeVar("T")

#: Default republish rate for :meth:`RobotClient.driving`, in Hz. Well inside
#: ``COMMAND_MAX_AGE``, so a dropped sample does not interrupt motion.
DRIVE_RATE_HZ = 10.0

#: Seconds to wait for the navigation coordinator to answer a submit or a
#: cancel. Generous, because a preempting submit answers only once the
#: displaced task has actually stopped.
NAV_CALL_TIMEOUT = 6.0

#: How many finished tasks the client keeps results for, so that
#: :meth:`RobotClient.wait_for_task` works when called after the fact.
NAV_RESULT_HISTORY = 32

#: Task id :meth:`RobotClient.wait_for_nav` asks after. It is a question the
#: coordinator can only answer if it is running, and one no real task can
#: collide with — ids are hex, and this is not.
_NAV_PROBE_ID = "robodog-sdk-liveness-probe"

#: Per-attempt timeout for that probe. Short: an answer either comes back
#: promptly or the node is not there, and a long wait only delays the retry.
_NAV_PROBE_TIMEOUT = 1.0


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

    Every field is a :class:`Latest`, and age is exposed rather than hidden
    because most of these are only as true as they are recent. The robot state
    streams continuously, so a stale value means the producer stopped;
    navigation feedback exists only while a task runs, so its age is what tells
    you whether one does; the gateway and safety values are edge-published, so
    their age is the time since the last *change* and says nothing about
    liveness.
    """

    def __init__(self) -> None:
        #: Raw odometry off the robot — where it thinks it has driven to.
        self.odometry: Latest[OdometryState] = Latest()
        #: The fused pose, from SLAM or the odometry fallback. This is the one
        #: navigation goals are expressed in; :attr:`odometry` drifts.
        self.localization: Latest[OdometryState] = Latest()
        self.battery: Latest[BatteryState] = Latest()
        self.highstate: Latest[RobotHighState] = Latest()
        #: Most recent :class:`TaskFeedback`, from whichever task published it.
        self.nav: Latest[TaskFeedback] = Latest()
        #: The gateway's last decision: who is driving, and what was done to
        #: their command. Edge-published, so its age is the time since the last
        #: change, not the time since the last tick.
        self.gateway: Latest[MotionGatewayStatus] = Latest()
        #: The safety latch — the authority on whether the robot may move at
        #: all. Republished on a heartbeat as well as on change, so unlike the
        #: gateway status its age *is* meaningful: see
        #: :attr:`RobotClient.motion_permitted`.
        self.safety: Latest[SafetyState] = Latest()
        #: The composite state: nav activity, control mode, order and posture
        #: in one payload, plus the headline derived from them.
        self.system: Latest[SystemState] = Latest()


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
            async with robot.driving(x=0.3):
                await asyncio.sleep(2)

    Args:
        node: The node whose session, publishers and subscriptions are used.
        source: Stamped on every outgoing movement command, and therefore what
            decides whether this client's commands beat anyone else's. The
            default loses to teleoperation and to navigation, which is what a
            node you are writing should do; raise it only if this process
            genuinely is the thing the higher rank names.
    """

    def __init__(self, node: Node, *, source: MovementSource = MovementSource.autonomous) -> None:
        self._node = node
        self._source = source
        self.state = StateView()

        self._move = node.publisher(MotionTopics.request)
        self._cancel_button = node.publisher(SafetyTopics.cancel)
        self._action = node.publisher(PoseTopics.action)
        self._tilt = node.publisher(PoseTopics.tilt_body)
        self._cancel_seq = 0

        node.subscribe(StateTopics.odometry, self.state.odometry.update, mode="latest")
        node.subscribe(LocalizationTopics.pose, self.state.localization.update, mode="latest")
        node.subscribe(StateTopics.battery, self.state.battery.update, mode="latest")
        node.subscribe(StateTopics.highstate, self.state.highstate.update, mode="latest")
        node.subscribe(StateTopics.system, self.state.system.update, mode="latest")
        node.subscribe(ControlTopics.status, self.state.gateway.update, mode="latest")
        node.subscribe(SafetyTopics.state, self.state.safety.update, mode="latest")
        # Both wildcards, and both declared here rather than per task: a result
        # is published once and is not latched, so a subscription declared
        # after the submit returns can miss a task that failed immediately.
        node.subscribe(NavTopics.feedback, self._on_task_feedback, mode="latest")
        node.subscribe(NavTopics.result, self._on_task_result)

        self._task_waiters: dict[str, asyncio.Future[TaskResult]] = {}
        self._task_results: dict[str, TaskResult] = {}

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
            name: Node name used for presence and in service calls.
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

    @property
    def source(self) -> MovementSource:
        """The rank this client's commands are sent at."""
        return self._source

    @property
    def driving_now(self) -> bool:
        """Whether the gateway is currently forwarding *this* client's source.

        ``False`` also when no status has arrived at all, so it answers "am I
        demonstrably the driver", not "is anyone else". Compare
        :attr:`preempted_by`.
        """
        status = self.state.gateway.value
        return status is not None and status.active_source is self._source

    @property
    def preempted_by(self) -> MovementSource | None:
        """The source that is out-ranking this client, if any.

        ``None`` when nothing is, which covers both "we are driving" and
        "nobody is driving". There is nothing to do about a preemption except
        notice it: the gateway re-decides on every frame, so a client that
        keeps publishing resumes automatically once the higher source falls
        silent.
        """
        status = self.state.gateway.value
        if status is None or status.active_source is None:
            return None
        active = status.active_source
        return active if active.outranks(self._source) else None

    @property
    def blocked_by_zone(self) -> list[str]:
        """Names of the collision zones currently holding the robot back.

        Empty when nothing is breached — which is not the same as "the robot
        will move": see :attr:`StateView.gateway` for the watchdog.
        """
        status = self.state.gateway.value
        return list(status.active_zones) if status is not None else []

    def motion_permitted(self, *, within: float = 2.0) -> bool:
        """Whether the safety authority currently permits motion.

        Fails safe on silence, which is the whole point of asking: a latch that
        stopped arriving is treated exactly like one that says stopped, so a
        crashed safety node or a severed link cannot read as permission. That
        is why this is a method taking a freshness window rather than a
        property — the answer depends on *when* the last frame arrived, not
        only on what it said.

        Args:
            within: Seconds. A latch older than this counts as absent. The
                safety node republishes on a heartbeat, so a value well above
                that interval still catches a genuine outage.
        """
        latch = self.state.safety.value
        if latch is None or not self.state.safety.fresh(within=within):
            return False
        return latch.motion_permitted

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
        """Command zero velocity from this client, and nothing more.

        It stops *this* client's contribution: a higher-ranking source keeps
        driving, and a navigation task keeps running. :meth:`emergency_stop`
        is the one that stops everything.
        """
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
        """Stop the robot now, and abandon whatever it was doing.

        Publishes the cancel button event that the safety node, the navigation
        coordinator and the fleet bridge each subscribe to on their own: the
        robot is commanded to zero, the running navigation task is cancelled,
        and any order runtime is wiped. Each is done by its owner, so none of
        it depends on this process staying alive afterwards.

        This is the *software* stop and there is no counterpart to it, because
        it does not latch. Only the physical switch latches — engaging it is
        what puts :attr:`StateView.safety` into a stopped phase, and only the
        release press on the panel clears that. Software can stop the robot;
        it cannot pretend to be the button.
        """
        self._cancel_seq += 1
        self._cancel_button.put(ButtonEvent(source_id=self._node.name, seq=self._cancel_seq))

    # ------------------------------------------------------------ navigation

    async def navigate_to(
        self,
        x: float,
        y: float,
        heading: float | None = None,
        *,
        timeout: float | None = None,
        max_speed: float | None = None,
        skill: str | None = None,
        preempt: bool = False,
        on_estop: EstopPolicy = EstopPolicy.CANCEL,
    ) -> TaskResult:
        """Navigate to a point and wait for the task to finish.

        Args:
            x: Target x in the ``map`` frame, m.
            y: Target y in the ``map`` frame, m.
            heading: Requested orientation on arrival, rad. ``None`` leaves the
                final heading a don't-care. Requested, not guaranteed: a skill
                configured to arrive on position alone ignores it rather than
                turning on the spot at the goal.
            timeout: Seconds to wait for a terminal state. ``None`` waits
                indefinitely, which is what a skill that waits out an obstacle
                rather than giving up requires.
            max_speed: Speed limit for the task, m/s. ``None`` uses the
                skill's configured cruise speed.
            skill: Skill to run the goal. ``None`` uses the deployment default.
            preempt: Displace a task that is already running. Without it a
                submit made while the robot is navigating is refused.
            on_estop: What an emergency stop does to this task. The default
                discards it; :attr:`EstopPolicy.HOLD` keeps it and resumes
                once motion is permitted again, which is only right if this
                call is still there to see the outcome.

        Returns:
            The terminal :class:`TaskResult`. Check ``result.succeeded``, or
            ``result.state`` for which of the four terminal states it reached
            and ``result.message`` for why.

        Raises:
            PermissionError: The coordinator refused the goal — another task is
                running and ``preempt`` was not set, or the skill is unknown.
            TimeoutError: No terminal state arrived within ``timeout``.
            zenode.ServiceTimeout: Nothing answered the submit. Usually the
                navigation node is not running, or the namespace is wrong.

        Cancelling the awaiting task stops the wait, not the robot. Call
        :meth:`cancel_task` to stop navigating.
        """
        goal = NavigateToPoseGoal(
            target=Pose2D(x=x, y=y, theta=heading or 0.0),
            orientation_at_target=heading,
            max_speed=max_speed,
            skill=skill,
        )
        return await self.run_goal(goal, timeout=timeout, preempt=preempt, on_estop=on_estop)

    async def navigate_through(
        self,
        poses: Sequence[Pose2D] | Sequence[tuple[float, float]],
        *,
        final_heading: float | None = None,
        timeout: float | None = None,
        max_speed: float | None = None,
        skill: str | None = None,
        preempt: bool = False,
        on_estop: EstopPolicy = EstopPolicy.CANCEL,
        corridor_deviation_m: float | None = None,
    ) -> TaskResult:
        """Navigate a route through several poses and wait for the outcome.

        Args:
            poses: The route, in order; the last one is the target. Accepts
                :class:`Pose2D` or plain ``(x, y)`` pairs.
            final_heading: Requested orientation at the last pose, rad.
            timeout: As :meth:`navigate_to`.
            max_speed: As :meth:`navigate_to`.
            skill: As :meth:`navigate_to`. A skill that plans for itself may
                route to the last pose directly instead of following the
                intermediate ones — pass ``"waypoint_follow"`` when the route
                is the point.
            preempt: As :meth:`navigate_to`.
            on_estop: As :meth:`navigate_to`.
            corridor_deviation_m: Half-width in metres of the corridor around
                the route that a human may drive the robot out of and still
                have it resume. ``None`` lets it resume from anywhere, which
                is fine for a route that was a convenience and wrong for one
                that was a decision.
        """
        goal = NavigateThroughPosesGoal(
            poses=[
                p if isinstance(p, Pose2D) else Pose2D(x=p[0], y=p[1], theta=0.0) for p in poses
            ],
            final_orientation=final_heading,
            max_speed=max_speed,
            skill=skill,
            corridor_deviation_m=corridor_deviation_m,
        )
        return await self.run_goal(goal, timeout=timeout, preempt=preempt, on_estop=on_estop)

    async def run_goal(
        self,
        goal: TaskGoal,
        *,
        timeout: float | None = None,
        preempt: bool = False,
        on_estop: EstopPolicy = EstopPolicy.CANCEL,
    ) -> TaskResult:
        """Submit a goal and wait for its result. Submit plus wait, in one call.

        Raises:
            PermissionError: The goal was not accepted.
            TimeoutError: No terminal state arrived within ``timeout``.
        """
        handle = await self.submit(goal, preempt=preempt, on_estop=on_estop)
        if not handle.accepted:
            raise PermissionError(
                f"navigation goal not accepted: {handle.reason or 'no reason given'}"
            )
        return await self.wait_for_task(handle.task_id, timeout=timeout)

    async def submit(
        self,
        goal: TaskGoal,
        *,
        preempt: bool = False,
        on_estop: EstopPolicy = EstopPolicy.CANCEL,
    ) -> TaskHandle:
        """Submit a goal without waiting for it to finish.

        For fire-and-forget navigation, and for the case where the caller wants
        to watch :attr:`state.nav <StateView.nav>` rather than block. Pair it
        with :meth:`wait_for_task` when you want the result later: the client
        subscribes to results for its whole lifetime, so waiting after the fact
        is safe.

        Args:
            goal: What to do.
            preempt: Displace a task that is already running.
            on_estop: What an emergency stop does to this task.

        Returns:
            The coordinator's :class:`TaskHandle`. ``accepted=False`` is an
            ordinary answer here, not an exception — unlike :meth:`run_goal`.

        Raises:
            zenode.ServiceTimeout: Nothing answered on the submit key.
        """
        # Both knobs travel as query parameters rather than in the payload, so
        # the goal on the wire stays exactly the goal. zenode has no API for
        # query parameters, hence the selector built by hand here. The client
        # name goes along for the coordinator's logs.
        params = [f"client={self._node.name}", f"on_estop={on_estop.value}"]
        if preempt:
            params.append("preempt=true")
        key = NavServices.submit.resolve(self._node.namespace) + "?" + "&".join(params)
        return await call_service(
            self._node.session,
            NavServices.submit,
            key,
            TaskGoalEnvelope(goal),
            timeout=NAV_CALL_TIMEOUT,
            node=self._node.name,
        )

    async def wait_for_task(self, task_id: str, *, timeout: float | None = None) -> TaskResult:
        """Wait for a task to reach a terminal state.

        Returns immediately if the result has already arrived. Safe to call
        long after the submit — results are kept for the last
        ``NAV_RESULT_HISTORY`` tasks — and safe to call from several places at
        once for the same task.

        Raises:
            TimeoutError: No terminal state arrived within ``timeout``.
        """
        recorded = self._task_results.get(task_id)
        if recorded is not None:
            return recorded
        loop = asyncio.get_running_loop()
        waiter: asyncio.Future[TaskResult] = loop.create_future()
        self._task_waiters[task_id] = waiter
        try:
            return await asyncio.wait_for(waiter, timeout)
        finally:
            self._task_waiters.pop(task_id, None)

    async def cancel_task(self, task_id: str) -> CancelAck:
        """Abandon a task and stop.

        ``canceled=False`` in the reply means the coordinator did nothing,
        which is the normal answer for a task that had already finished.

        Raises:
            zenode.ServiceTimeout: Nothing answered on the cancel key.
        """
        return await self._node.call(
            NavServices.cancel, CancelRequest(task_id=task_id), timeout=NAV_CALL_TIMEOUT
        )

    async def task_status(self, task_id: str) -> TaskResult:
        """Ask the coordinator what became of a task.

        For a process that did not watch the task itself. The answer is
        terminal for a task that finished and :attr:`TaskState.RUNNING` for one
        still under way, so — unlike a payload from the result key — check
        ``state.is_terminal`` before treating it as an outcome.

        A task the coordinator has no record of, whether it was never submitted
        or has since aged out of its bounded history, is answered on the Zenoh
        error channel and raises. "Unknown" is not a lifecycle state, and this
        call will not invent one for you.

        Raises:
            zenode.ServiceError: The coordinator has no record of this task.
            zenode.ServiceTimeout: Nothing answered on the status key. Note
                that this *subclasses* ``ServiceError``, so catching the latter
                alone cannot tell "nav says it never heard of it" from "nav is
                not running". Catch this one first when the difference matters.
        """
        return await self._node.call(
            task_status_service(task_id),
            TaskStatusRequest(task_id=task_id),
            timeout=NAV_CALL_TIMEOUT,
        )

    @property
    def navigating(self) -> bool:
        """Whether navigation feedback has arrived recently.

        A heuristic, and the only one available: feedback is published by the
        running skill, so silence means either no task or a task whose skill
        has stopped publishing. It is not a substitute for the result.
        """
        return self.state.nav.fresh(within=1.0)

    def _on_task_feedback(self, feedback: TaskFeedback) -> None:
        self.state.nav.update(feedback)

    def _on_task_result(self, result: TaskResult) -> None:
        if not result.state.is_terminal:
            # A non-terminal payload on the result key is a producer bug; keep
            # it out of the history so a later wait does not resolve on it.
            return
        self._task_results[result.task_id] = result
        while len(self._task_results) > NAV_RESULT_HISTORY:
            self._task_results.pop(next(iter(self._task_results)))
        waiter = self._task_waiters.get(result.task_id)
        if waiter is not None and not waiter.done():
            waiter.set_result(result)

    # ------------------------------------------------------------- readiness

    async def wait_until_ready(self, *nodes: str, timeout: float = 10.0) -> None:
        """Wait until the named nodes hold presence tokens.

        .. warning::

           **No node of the control stack holds one.** Presence is a zenode
           liveliness token at ``<ns>/node/<name>``, and the stack's nodes are
           plain Zenoh applications — nav, the motion gateway, the safety
           aggregator and the robot bridge are all invisible to this. Waiting
           for ``"nav"`` here does not wait; it times out after ``timeout`` and
           raises, whether or not nav is running.

        So this is for waiting on *peers* — other nodes built on zenode, yours
        or a teammate's. To wait for the navigation coordinator, use
        :meth:`wait_for_nav`, which asks it a question instead of looking for a
        token it does not hold. For the rest of the stack the honest signal is
        the data itself: :attr:`state.safety <StateView.safety>` arriving means
        the safety node is up, and a pose means the bridge is.

        There is no default node name: which peers your behaviour needs is your
        behaviour's business, and waiting for the wrong one is worse than not
        waiting.

        Args:
            nodes: Node names to wait for. At least one.
            timeout: Seconds to wait.

        Raises:
            ValueError: No node names were given.
            TimeoutError: A node was still absent when the timeout elapsed.
        """
        if not nodes:
            raise ValueError("wait_until_ready() needs at least one node name")
        await self._node.wait_for_nodes(list(nodes), timeout=timeout)

    async def wait_for_nav(self, *, timeout: float = 10.0, poll: float = 0.5) -> None:
        """Wait until the navigation coordinator is answering.

        The coordinator holds no presence token, so this asks it something
        instead: the status of a task that cannot exist. A running coordinator
        answers that on the error channel — it has no such task — and that
        refusal *is* the liveness signal, because only a running one can
        produce it. Silence means nothing is serving the key.

        Args:
            timeout: Seconds to keep asking for.
            poll: Seconds between attempts.

        Raises:
            TimeoutError: Nothing answered the status key within ``timeout``.
        """
        deadline = time.monotonic() + timeout
        while True:
            try:
                await self._node.call(
                    task_status_service(_NAV_PROBE_ID),
                    TaskStatusRequest(task_id=_NAV_PROBE_ID),
                    timeout=_NAV_PROBE_TIMEOUT,
                )
            except ServiceTimeout:
                pass  # nobody is serving the key yet
            except ServiceError:
                # "no such task" — which only a live coordinator says. This
                # clause must stay below ServiceTimeout, which subclasses it:
                # the other order catches silence too and reports a coordinator
                # that is not there as one that is.
                return
            else:
                return  # answered with a result, which is stranger but still up
            if time.monotonic() >= deadline:
                raise TimeoutError(f"navigation coordinator did not answer within {timeout}s")
            await asyncio.sleep(poll)

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
