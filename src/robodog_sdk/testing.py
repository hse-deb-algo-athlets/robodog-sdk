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

from .msgs.motion import (
    GatewayAction,
    MotionGatewayStatus,
    MovementCommand,
    MovementSource,
)
from .msgs.navigation import (
    CancelAck,
    CancelRequest,
    NavigateThroughPosesGoal,
    NavigateToPoseGoal,
    Pose2D,
    TaskFeedback,
    TaskGoalEnvelope,
    TaskHandle,
    TaskResult,
    TaskState,
)
from .msgs.robot import BatteryLevel, BatteryState, OdometryState
from .topics import (
    ControlTopics,
    MotionTopics,
    NavServices,
    StateTopics,
    task_feedback_topic,
    task_result_topic,
)


class FakeNav(Node):
    """Stands in for the navigation coordinator: accepts tasks, finishes them.

    One task at a time, as in the real coordinator. A submit while a task is
    running cancels that task first — the double cannot see the ``preempt``
    query parameter that decides this in the stack, so it always preempts.
    Set :attr:`accept` to ``False`` to make the next submit be refused instead,
    which is how a test covers the "refused" branch.

    By default a task succeeds after :attr:`duration` seconds of feedback. Set
    :attr:`result_state` to finish somewhere else, or :attr:`auto_finish` to
    ``False`` and drive the ending yourself::

        nav.auto_finish = False
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

    async def on_start(self) -> None:
        self.goals = []
        self.accept = True
        self.result_state = TaskState.SUCCEEDED
        self.duration = 0.05
        self.auto_finish = True
        self.feedback_period = 0.02
        self._active_id: str | None = None
        self._finished: asyncio.Event = asyncio.Event()
        self._ending: TaskResult | None = None

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

    async def _run(self, task_id: str, goal: object) -> None:
        feedback = self.publisher(task_feedback_topic(task_id))
        result = self.publisher(task_result_topic(task_id))
        target = _goal_target(goal)
        elapsed = 0.0
        while not self._finished.is_set():
            feedback.put(
                TaskFeedback(
                    task_id=task_id,
                    state=TaskState.RUNNING,
                    current_pose=target,
                    distance_to_goal=0.0,
                    active_skill="fake",
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
        result.put(ending)


def _goal_target(goal: object) -> Pose2D | None:
    if isinstance(goal, NavigateToPoseGoal):
        return goal.target
    if isinstance(goal, NavigateThroughPosesGoal):
        return goal.poses[-1]
    return None


class FakeStack(Node):
    """Publishes latched robot state and records commands sent to the gateway."""

    name = "fake-stack"
    health_interval = None

    odometry = publish(StateTopics.odometry)
    battery = publish(StateTopics.battery)
    gateway = publish(ControlTopics.status)

    #: Every ``MovementCommand`` seen on the gateway inlet, in order and from
    #: every source. The double does not arbitrate — it records.
    commands: list[MovementCommand]

    async def on_start(self) -> None:
        self.commands = []
        self.odometry.put(OdometryState())
        self.battery.put(BatteryState(soc=87, level=BatteryLevel.good, voltage=28.4))
        self.gateway.put(MotionGatewayStatus(active_source=None))

    @subscribe(MotionTopics.request)
    async def on_movement_request(self, msg: MovementCommand) -> None:
        self.commands.append(msg)

    def set_pose(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        """Publish a new pose. The transition is instantaneous."""
        self.odometry.put(OdometryState(x=x, y=y, z=z))

    def set_battery(self, soc: int, level: BatteryLevel = BatteryLevel.good) -> None:
        """Publish a new battery state."""
        self.battery.put(BatteryState(soc=soc, level=level))

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
