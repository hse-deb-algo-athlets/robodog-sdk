"""Test double for the control stack.

``zenode.testing`` provides an in-process session; this module provides the
counterparty — state on the latched topics, a navigation coordinator that
accepts tasks, and a record of the commands a node under test issued::

    from zenode.testing import harness
    from robodog_sdk.testing import FakeStack

    async with harness() as h:
        stack = await h.start_node(FakeStack)
        await h.start_node(FakeNav)          # only if the node navigates
        await h.start_node(MyAgent)

Nothing here integrates motion or reacts to a command: :class:`FakeNav`
finishes a task without the robot having moved a metre, and :class:`FakeStack`
records commands without arbitrating between them. Use these to test how a node
responds to state and to outcomes; use the MuJoCo simulation to test whether the
robot reaches a target.
"""

from __future__ import annotations

import asyncio
import contextlib
import uuid

from zenode import Node, publish, serve, subscribe

from .msgs.localization import MapIdentity
from .msgs.motion import (
    GatewayAction,
    MotionGatewayStatus,
    MovementCommand,
    MovementSource,
)
from .msgs.navigation import (
    CancelAck,
    CancelRequest,
    NavActivity,
    NavigateThroughPosesGoal,
    NavigateToPoseGoal,
    Pose2D,
    TaskFeedback,
    TaskGoalEnvelope,
    TaskHandle,
    TaskResult,
    TaskState,
    TaskStatusRequest,
)
from .msgs.robot import BatteryLevel, BatteryState, OdometryState
from .msgs.safety import ButtonEvent, EstopPhase, SafetyState
from .msgs.system_state import ControlMode, Posture, SystemState
from .topics import (
    ControlTopics,
    LocalizationTopics,
    MotionTopics,
    NavServices,
    SafetyTopics,
    StateTopics,
    task_feedback_topic,
    task_result_topic,
)


class FakeNav(Node):
    """Stands in for the navigation coordinator: accepts tasks, finishes them.

    One task at a time, as in the real coordinator. A submit while a task is
    running cancels that task first — the double cannot see the ``preempt``
    query parameter that decides this in the stack, and for the same reason it
    cannot see ``on_estop`` either, so it always preempts and never holds.
    Set :attr:`accept` to ``False`` to make the next submit be refused instead,
    which is how a test covers the "refused" branch.

    By default a task succeeds after :attr:`duration` seconds of feedback. Set
    :attr:`result_state` to finish somewhere else, or :attr:`auto_finish` to
    ``False`` and drive the ending yourself::

        nav.auto_finish = False
        nav.activity = NavActivity.STALLED
        ...
        nav.finish(TaskState.BLOCKED, "door did not open")
    """

    name = "nav"
    health_interval = None

    #: Every goal that was accepted, in order.
    goals: list[NavigateToPoseGoal | NavigateThroughPosesGoal]
    #: Whether the next submit is accepted at all.
    accept: bool
    #: Terminal state an auto-finished task reaches.
    result_state: TaskState
    #: Seconds a task runs before it auto-finishes.
    duration: float
    #: Whether a task ends on its own.
    auto_finish: bool
    #: Feedback publish period, seconds.
    feedback_period: float
    #: Sub-state stamped on every feedback frame. Assign to it mid-task to
    #: make the node under test see the skill stall or back off.
    activity: NavActivity

    async def on_start(self) -> None:
        self.goals = []
        self.accept = True
        self.result_state = TaskState.SUCCEEDED
        self.duration = 0.05
        self.auto_finish = True
        self.feedback_period = 0.02
        self.activity = NavActivity.CRUISING
        self._active_id: str | None = None
        self._finished: asyncio.Event = asyncio.Event()
        self._ending: TaskResult | None = None
        self._results: dict[str, TaskResult] = {}

    @property
    def active_task_id(self) -> str | None:
        """Id of the task currently running, if any."""
        return self._active_id

    @property
    def last_goal(self) -> NavigateToPoseGoal | NavigateThroughPosesGoal | None:
        return self.goals[-1] if self.goals else None

    def finish(self, state: TaskState = TaskState.SUCCEEDED, message: str = "") -> None:
        """End the running task in ``state``. No-op when nothing is running."""
        if self._active_id is None:
            return
        self._ending = TaskResult(task_id=self._active_id, state=state, message=message or None)
        self._finished.set()

    @serve(NavServices.submit)
    async def on_submit(self, request: TaskGoalEnvelope) -> TaskHandle:
        if not self.accept:
            return TaskHandle(task_id="", accepted=False, reason="fake nav is refusing tasks")
        if self._active_id is not None:
            self.finish(TaskState.CANCELED, "preempted by new submit")
            await asyncio.sleep(0)  # let the runner observe it
        task_id = uuid.uuid4().hex
        self.goals.append(request.goal)
        self._active_id = task_id
        self._ending = None
        self._finished = asyncio.Event()
        self.spawn(self._run(task_id, request.goal), name=f"task-{task_id[:8]}")
        return TaskHandle(task_id=task_id, accepted=True)

    @serve(NavServices.cancel)
    async def on_cancel(self, request: CancelRequest) -> CancelAck:
        if request.task_id != self._active_id or self._active_id is None:
            return CancelAck(
                task_id=request.task_id, canceled=False, reason="unknown or finished task"
            )
        self.finish(TaskState.CANCELED, "canceled by client")
        return CancelAck(task_id=request.task_id, canceled=True)

    @serve(NavServices.status)
    async def on_status(self, request: TaskStatusRequest) -> TaskResult:
        """Answer a late poll, and raise for a task this double never ran.

        The real coordinator reads the task id off the key and answers an
        unknown one on the Zenoh error channel. Raising here produces the same
        error reply, so a test sees the exception a client will really get; the
        id is taken from the payload because a served handler is not given the
        key it was called on.
        """
        recorded = self._results.get(request.task_id)
        if recorded is not None:
            return recorded
        if request.task_id == self._active_id:
            return TaskResult(
                task_id=request.task_id,
                state=TaskState.RUNNING,
                message="no terminal result recorded yet",
            )
        raise LookupError(f"no task '{request.task_id}' on record")

    async def _run(self, task_id: str, goal: object) -> None:
        feedback = self.publisher(task_feedback_topic(task_id))
        result = self.publisher(task_result_topic(task_id))
        target = _goal_target(goal)
        total = _goal_segments(goal)
        elapsed = 0.0
        while not self._finished.is_set():
            feedback.put(
                TaskFeedback(
                    task_id=task_id,
                    current_pose=target,
                    distance_to_goal=0.0,
                    active_skill="fake",
                    activity=self.activity,
                    total_segments=total,
                )
            )
            if self.auto_finish and elapsed >= self.duration:
                self.finish(self.result_state, "fake nav finished the task")
                break
            with contextlib.suppress(TimeoutError):
                await asyncio.wait_for(self._finished.wait(), self.feedback_period)
            elapsed += self.feedback_period
        ending = self._ending or TaskResult(task_id=task_id, state=self.result_state)
        self._active_id = None
        self._results[task_id] = ending
        result.put(ending)


def _goal_target(goal: object) -> Pose2D | None:
    if isinstance(goal, NavigateToPoseGoal):
        return goal.target
    if isinstance(goal, NavigateThroughPosesGoal):
        return goal.poses[-1]
    return None


def _goal_segments(goal: object) -> int | None:
    return len(goal.poses) if isinstance(goal, NavigateThroughPosesGoal) else None


class FakeStack(Node):
    """Publishes latched robot state and records commands sent to the gateway."""

    name = "fake-stack"
    health_interval = None

    odometry = publish(StateTopics.odometry)
    localization = publish(LocalizationTopics.pose)
    map_identity = publish(LocalizationTopics.map_identity)
    battery = publish(StateTopics.battery)
    system = publish(StateTopics.system)
    gateway = publish(ControlTopics.status)
    safety = publish(SafetyTopics.state)

    #: Every ``MovementCommand`` seen on the gateway inlet, in order and from
    #: every source. The double does not arbitrate — it records.
    commands: list[MovementCommand]
    #: Every cancel button event seen, in order. This is what
    #: :meth:`~robodog_sdk.RobotClient.emergency_stop` sends; the double
    #: records it rather than acting on it, so a test can assert that a node
    #: asked for the stop without the double having to model the recovery.
    cancels: list[ButtonEvent]

    async def on_start(self) -> None:
        self.commands = []
        self.cancels = []
        self.odometry.put(OdometryState())
        self.localization.put(OdometryState())
        self.map_identity.put(MapIdentity(map_id="fake-map", source="fake-stack", reachable=True))
        self.battery.put(BatteryState(soc=87, level=BatteryLevel.good, voltage=28.4))
        self.gateway.put(MotionGatewayStatus(active_source=None))
        self.system.put(SystemState(posture=Posture.STANDING))
        # Explicitly permissive. The defaults on SafetyState are fail-safe —
        # stopped, no live source — so a double that published one unmodified
        # would leave every node under test refusing to move.
        self.safety.put(SafetyState(estop=False, source_alive=True, phase=EstopPhase.CLEAR))

    @subscribe(MotionTopics.request)
    async def on_movement_request(self, msg: MovementCommand) -> None:
        self.commands.append(msg)

    @subscribe(SafetyTopics.cancel)
    async def on_cancel_button(self, msg: ButtonEvent) -> None:
        self.cancels.append(msg)

    def set_pose(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        """Publish a new pose, on both the odometry and localization keys.

        The transition is instantaneous, and the two agree exactly — which the
        real ones will not, since odometry drifts and the fused pose is what
        corrects it. Test a node's handling of that disagreement by driving
        :attr:`localization` directly.
        """
        pose = OdometryState(x=x, y=y, z=z)
        self.odometry.put(pose)
        self.localization.put(pose)

    def set_battery(self, soc: int, level: BatteryLevel = BatteryLevel.good) -> None:
        """Publish a new battery state."""
        self.battery.put(BatteryState(soc=soc, level=level))

    def set_map(self, map_id: str | None, *, reachable: bool = True) -> None:
        """Publish a map identity — which map poses are anchored to.

        ``map_id=None`` is the case worth testing: it is what a node sees when
        SLAM is down or the odometry fallback is driving, and a node that
        treats it as "unchanged" will happily drive to a stored coordinate
        that no longer means anything::

            stack.set_map("hall-b")
            ...
            stack.set_map(None)     # SLAM went away
            stack.set_map("hall-c") # somebody loaded a different map
        """
        self.map_identity.put(MapIdentity(map_id=map_id, source="fake-stack", reachable=reachable))

    def set_safety(
        self,
        *,
        estop: bool = False,
        source_alive: bool = True,
        phase: EstopPhase = EstopPhase.CLEAR,
    ) -> None:
        """Publish a safety latch — whether the robot may move, and why not.

        The parameters are independent on purpose, because in the stack they
        are: ``phase`` carries the story an operator needs while ``estop`` and
        ``source_alive`` decide the answer. To make a node see a pressed
        button::

            stack.set_safety(estop=True, phase=EstopPhase.STOPPED)

        and to make it see the safety source disappear, which stops just as
        hard for an entirely different reason::

            stack.set_safety(source_alive=False, phase=EstopPhase.SOURCE_LOST)
        """
        self.safety.put(SafetyState(estop=estop, source_alive=source_alive, phase=phase))

    def set_system(
        self,
        *,
        control: ControlMode = ControlMode.AUTO,
        posture: Posture = Posture.STANDING,
        nav: NavActivity = NavActivity.NONE,
        safety_phase: EstopPhase = EstopPhase.CLEAR,
    ) -> None:
        """Publish a composite state. ``headline`` is derived, not passed in."""
        self.system.put(
            SystemState(control=control, posture=posture, nav=nav, safety_phase=safety_phase)
        )

    def set_driver(
        self,
        source: MovementSource | None,
        *,
        action: GatewayAction = GatewayAction.pass_through,
        zones: list[str] | None = None,
        watchdog_tripped: bool = False,
        reason: str | None = None,
    ) -> None:
        """Publish a gateway status — who is driving and what happened to it.

        This is how a test says "a human just took the gamepad" or "the robot
        is standing in a stop zone", neither of which the double can work out
        from the commands it receives::

            stack.set_driver(MovementSource.controller, reason="human took over")
            stack.set_driver(MovementSource.planner, action=GatewayAction.stop,
                             zones=["front"])
        """
        self.gateway.put(
            MotionGatewayStatus(
                active_source=source,
                action=action,
                active_zones=zones or [],
                watchdog_tripped=watchdog_tripped,
                reason=reason,
            )
        )

    @property
    def last_command(self) -> MovementCommand | None:
        """The most recent command on the inlet, if any."""
        return self.commands[-1] if self.commands else None

    @property
    def stopped(self) -> bool:
        """Whether the most recent command was zero velocity."""
        last = self.last_command
        return last is not None and last.is_zero()
