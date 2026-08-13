"""RobotClient against an in-process session: no router, robot or simulation.

Also the reference for testing a node of your own: start ``FakeStack``, start
the node, and assert on the commands it issued.
"""

from __future__ import annotations

import asyncio

import pytest
from zenode import Node
from zenode.testing import harness

from robodog_sdk import (
    ActionType,
    GatewayAction,
    MotionTopics,
    MovementSource,
    NavigateThroughPosesGoal,
    NavigateToPoseGoal,
    Pose2D,
    RobotClient,
    SafetyTopics,
    TaskState,
)
from robodog_sdk.testing import FakeNav, FakeStack

pytestmark = pytest.mark.integration


class _Agent(Node):
    """Minimal host node, providing the session the client uses."""

    name = "test-agent"
    health_interval = None
    robot: RobotClient

    async def on_start(self) -> None:
        self.robot = RobotClient(self)


async def _agent_with_stack(h) -> tuple[_Agent, FakeStack]:
    stack = await h.start_node(FakeStack)
    agent = await h.start_node(_Agent)
    return agent, stack


async def _agent_with_nav(h) -> tuple[_Agent, FakeNav]:
    nav = await h.start_node(FakeNav)
    agent = await h.start_node(_Agent)
    return agent, nav


async def test_move_reaches_the_gateway_inlet_at_the_lowest_rank() -> None:
    async with harness() as h:
        agent, _ = await _agent_with_stack(h)
        out = h.collect(MotionTopics.request)

        agent.robot.move(x=0.4, z_deg=15.0)

        cmd = await out.next()
        assert (cmd.x, cmd.z_deg) == (0.4, 15.0)
        assert cmd.source is MovementSource.autonomous, "a client must not outrank a human"


def test_client_has_nothing_to_acquire() -> None:
    """There is no handshake to perform, so there is no method to call."""
    assert not hasattr(RobotClient, "control")


async def test_state_view_sees_latched_state() -> None:
    async with harness() as h:
        agent, stack = await _agent_with_stack(h)

        stack.set_pose(x=1.5, y=-2.0)
        await asyncio.sleep(0.2)

        assert agent.robot.state.odometry.value is not None
        assert agent.robot.state.odometry.value.x == 1.5
        assert agent.robot.state.odometry.fresh(within=2.0)


async def test_state_view_reports_missing_data_as_infinite_age() -> None:
    async with harness() as h:
        agent = await h.start_node(_Agent)  # no stack: nothing ever publishes

        assert agent.robot.state.battery.value is None
        assert agent.robot.state.battery.age == float("inf")
        assert not agent.robot.state.battery


async def test_driving_republishes_then_stops() -> None:
    async with harness() as h:
        agent, stack = await _agent_with_stack(h)

        async with agent.robot.driving(x=0.2, rate_hz=20.0):
            await asyncio.sleep(0.3)

        await asyncio.sleep(0.1)
        assert len(stack.commands) > 3, "driving() should republish, not fire once"
        assert stack.stopped, "leaving driving() must leave the robot stopped"


async def test_preemption_is_visible_without_a_handshake() -> None:
    async with harness() as h:
        agent, stack = await _agent_with_stack(h)

        stack.set_driver(MovementSource.autonomous)
        await asyncio.sleep(0.2)
        assert agent.robot.driving_now
        assert agent.robot.preempted_by is None

        stack.set_driver(MovementSource.controller, reason="human took over")
        await asyncio.sleep(0.2)
        assert agent.robot.preempted_by is MovementSource.controller
        assert not agent.robot.driving_now


async def test_a_stop_zone_is_reported_by_name() -> None:
    async with harness() as h:
        agent, stack = await _agent_with_stack(h)

        stack.set_driver(MovementSource.autonomous, action=GatewayAction.stop, zones=["front"])
        await asyncio.sleep(0.2)

        assert agent.robot.blocked_by_zone == ["front"]
        status = agent.robot.state.gateway.value
        assert status is not None and not status.moving_allowed


async def test_action_and_estop_are_published() -> None:
    async with harness() as h:
        agent, _ = await _agent_with_stack(h)
        actions = h.collect(SafetyTopics.estop)

        agent.robot.action(ActionType.hello)
        agent.robot.emergency_stop()

        assert (await actions.next()).command.value == "stop"


def test_client_has_no_estop_release() -> None:
    """An e-stop is cleared at the physical button, not through the client."""
    assert not hasattr(RobotClient, "release_emergency_stop")
    assert not hasattr(RobotClient, "clear_estop")


# ------------------------------------------------------------------ navigation


async def test_navigate_to_returns_the_terminal_result() -> None:
    async with harness() as h:
        agent, nav = await _agent_with_nav(h)

        result = await agent.robot.navigate_to(1.0, 2.0, timeout=5.0)

        assert result.state is TaskState.SUCCEEDED
        assert result.succeeded
        goal = nav.last_goal
        assert isinstance(goal, NavigateToPoseGoal)
        assert (goal.target.x, goal.target.y) == (1.0, 2.0)


async def test_navigate_to_reports_a_blocked_task_rather_than_raising() -> None:
    """BLOCKED is an outcome, not an error: the caller decides what it means."""
    async with harness() as h:
        agent, nav = await _agent_with_nav(h)
        nav.result_state = TaskState.BLOCKED

        result = await agent.robot.navigate_to(1.0, 0.0, timeout=5.0)

        assert result.state is TaskState.BLOCKED
        assert not result.succeeded


async def test_a_refused_goal_raises() -> None:
    async with harness() as h:
        agent, nav = await _agent_with_nav(h)
        nav.accept = False

        with pytest.raises(PermissionError):
            await agent.robot.navigate_to(1.0, 0.0, timeout=5.0)


async def test_feedback_lands_in_the_state_view() -> None:
    async with harness() as h:
        agent, nav = await _agent_with_nav(h)
        nav.auto_finish = False

        handle = await agent.robot.submit(
            NavigateThroughPosesGoal(poses=[Pose2D(x=1.0, y=0.0, theta=0.0)])
        )
        await asyncio.sleep(0.2)

        assert agent.robot.navigating
        feedback = agent.robot.state.nav.value
        assert feedback is not None
        assert feedback.task_id == handle.task_id
        assert feedback.state is TaskState.RUNNING

        nav.finish(TaskState.SUCCEEDED)
        assert (await agent.robot.wait_for_task(handle.task_id, timeout=5.0)).succeeded


async def test_cancel_ends_the_task_as_canceled() -> None:
    async with harness() as h:
        agent, nav = await _agent_with_nav(h)
        nav.auto_finish = False

        handle = await agent.robot.submit(
            NavigateThroughPosesGoal(poses=[Pose2D(x=5.0, y=0.0, theta=0.0)])
        )
        ack = await agent.robot.cancel_task(handle.task_id)

        assert ack.canceled
        result = await agent.robot.wait_for_task(handle.task_id, timeout=5.0)
        assert result.state is TaskState.CANCELED


async def test_wait_for_task_works_after_the_result_already_arrived() -> None:
    """The result key carries one message and is not latched, so the client
    subscribes for its whole life rather than per task."""
    async with harness() as h:
        agent, _ = await _agent_with_nav(h)

        handle = await agent.robot.submit(
            NavigateThroughPosesGoal(poses=[Pose2D(x=1.0, y=0.0, theta=0.0)])
        )
        await asyncio.sleep(0.4)  # the task finishes while nobody is waiting

        result = await agent.robot.wait_for_task(handle.task_id, timeout=1.0)
        assert result.succeeded


async def test_wait_for_task_times_out_without_stopping_the_robot() -> None:
    async with harness() as h:
        agent, nav = await _agent_with_nav(h)
        nav.auto_finish = False

        handle = await agent.robot.submit(
            NavigateThroughPosesGoal(poses=[Pose2D(x=9.0, y=0.0, theta=0.0)])
        )
        with pytest.raises(TimeoutError):
            await agent.robot.wait_for_task(handle.task_id, timeout=0.2)

        assert nav.active_task_id == handle.task_id, "the task keeps running"
