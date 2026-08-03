"""RobotClient against an in-process session: no router, robot or simulation.

Also the reference for testing a node of your own: start ``FakeStack``, start
the node, and assert on the commands it issued.
"""

from __future__ import annotations

import asyncio

import pytest
from zenode import Node
from zenode.testing import harness

from robodog_sdk import ActionType, Lane, MotionTopics, RobotClient
from robodog_sdk.testing import FakeStack

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


async def test_move_reaches_the_agent_lane() -> None:
    async with harness() as h:
        agent, _ = await _agent_with_stack(h)
        out = h.collect(MotionTopics.move_agent)

        agent.robot.move(x=0.4, z_deg=15.0)

        cmd = await out.next()
        assert (cmd.x, cmd.z_deg) == (0.4, 15.0)


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


async def test_control_acquires_and_releases_the_lane() -> None:
    async with harness() as h:
        agent, stack = await _agent_with_stack(h)

        async with agent.robot.control(reason="test"):
            pass

        await asyncio.sleep(0.1)
        assert [r.lane for r in stack.granted] == [Lane.agent]
        assert stack.stopped, "releasing the lane must stop the robot"


async def test_action_and_estop_are_published() -> None:
    async with harness() as h:
        agent, _ = await _agent_with_stack(h)
        actions = h.collect(MotionTopics.estop)

        agent.robot.action(ActionType.hello)
        agent.robot.emergency_stop()

        assert (await actions.next()).command.value == "stop"


def test_client_has_no_estop_release() -> None:
    """An e-stop is cleared at the physical button, not through the client."""
    assert not hasattr(RobotClient, "release_emergency_stop")
    assert not hasattr(RobotClient, "clear_estop")
