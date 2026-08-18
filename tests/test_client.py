"""RobotClient against an in-process session: no router, robot or simulation.

Also the reference for testing a node of your own: start ``FakeStack``, start
the node, and assert on the commands it issued.
"""

from __future__ import annotations

import asyncio

import pytest
from zenode import Node
from zenode.errors import ServiceError
from zenode.testing import harness

from robodog_sdk import (
    ActionType,
    ControlMode,
    EstopPhase,
    EstopPolicy,
    GatewayAction,
    Headline,
    MotionTopics,
    MovementSource,
    NavActivity,
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
        cancels = h.collect(SafetyTopics.cancel)

        agent.robot.action(ActionType.hello)
        agent.robot.emergency_stop()

        event = await cancels.next()
        assert event.source_id == "test-agent", "the stop is attributable to who asked for it"
        assert event.seq == 1


async def test_repeated_stops_are_distinguishable_from_a_resend() -> None:
    """Consumers deduplicate on ``(source_id, seq)``, so two deliberate stops
    must not look like one press delivered twice."""
    async with harness() as h:
        agent, stack = await _agent_with_stack(h)

        agent.robot.emergency_stop()
        agent.robot.emergency_stop()
        await asyncio.sleep(0.2)

        assert [e.seq for e in stack.cancels] == [1, 2]


def test_client_has_no_estop_release() -> None:
    """An e-stop is cleared at the physical button, not through the client."""
    assert not hasattr(RobotClient, "release_emergency_stop")
    assert not hasattr(RobotClient, "clear_estop")


async def test_motion_permitted_fails_safe_on_silence() -> None:
    """A latch that stopped arriving must read exactly like one that says
    stopped — otherwise a dead safety node reads as permission."""
    async with harness() as h:
        agent, stack = await _agent_with_stack(h)
        await asyncio.sleep(0.2)

        assert agent.robot.motion_permitted()

        stack.set_safety(estop=True, phase=EstopPhase.STOPPED)
        await asyncio.sleep(0.2)
        assert not agent.robot.motion_permitted()

        stack.set_safety()  # permitted again, but ask with no freshness budget
        await asyncio.sleep(0.2)
        assert not agent.robot.motion_permitted(within=0.0)


async def test_motion_is_denied_through_the_recovery_window() -> None:
    """``estop`` is already down in RELEASING so the robot can stand back up.
    It cannot be driven yet, and only ``motion_permitted`` says so."""
    async with harness() as h:
        agent, stack = await _agent_with_stack(h)

        stack.set_safety(estop=False, phase=EstopPhase.RELEASING)
        await asyncio.sleep(0.2)

        latch = agent.robot.state.safety.value
        assert latch is not None and not latch.estop
        assert not agent.robot.motion_permitted()


async def test_no_safety_source_is_not_a_pressed_button() -> None:
    """Both deny motion; the composite keeps them apart so an operator is not
    sent hunting for a switch nobody touched."""
    async with harness() as h:
        agent, stack = await _agent_with_stack(h)

        stack.set_safety(source_alive=False, phase=EstopPhase.SOURCE_LOST)
        stack.set_system(control=ControlMode.ESTOP, safety_phase=EstopPhase.SOURCE_LOST)
        await asyncio.sleep(0.2)

        assert not agent.robot.motion_permitted()
        system = agent.robot.state.system.value
        assert system is not None
        assert system.headline is Headline.SOURCE_LOST
        assert not system.ready_to_move


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


async def test_feedback_carries_the_skill_sub_state() -> None:
    """``state`` stays RUNNING for the whole task; ``activity`` is what moves."""
    async with harness() as h:
        agent, nav = await _agent_with_nav(h)
        nav.auto_finish = False
        nav.activity = NavActivity.STALLED

        handle = await agent.robot.submit(
            NavigateToPoseGoal(target=Pose2D(x=1.0, y=0.0, theta=0.0))
        )
        await asyncio.sleep(0.2)

        feedback = agent.robot.state.nav.value
        assert feedback is not None
        assert feedback.state is TaskState.RUNNING
        assert feedback.activity is NavActivity.STALLED

        nav.finish(TaskState.BLOCKED, "obstacle did not clear")
        result = await agent.robot.wait_for_task(handle.task_id, timeout=5.0)
        assert result.state is TaskState.BLOCKED


async def test_route_progress_is_reported_per_waypoint() -> None:
    async with harness() as h:
        agent, nav = await _agent_with_nav(h)
        nav.auto_finish = False

        await agent.robot.submit(
            NavigateThroughPosesGoal(poses=[Pose2D(x=x, y=0.0, theta=0.0) for x in (1.0, 2.0, 3.0)])
        )
        await asyncio.sleep(0.2)

        feedback = agent.robot.state.nav.value
        assert feedback is not None and feedback.total_segments == 3


async def test_status_of_a_forgotten_task_raises_rather_than_inventing_a_state() -> None:
    """ "Unknown" is answered on the error channel. A client that got a state
    back instead would have no way to tell it from a real one."""
    async with harness() as h:
        agent, _ = await _agent_with_nav(h)

        with pytest.raises(ServiceError):
            await agent.robot.task_status("a-task-that-never-existed")


async def test_status_of_a_running_task_is_not_terminal() -> None:
    """Unlike the result key, the status query can answer RUNNING — so a
    caller has to check before treating the answer as an outcome."""
    async with harness() as h:
        agent, nav = await _agent_with_nav(h)
        nav.auto_finish = False

        handle = await agent.robot.submit(
            NavigateToPoseGoal(target=Pose2D(x=1.0, y=0.0, theta=0.0))
        )
        status = await agent.robot.task_status(handle.task_id)

        assert status.state is TaskState.RUNNING
        assert not status.state.is_terminal

        nav.finish(TaskState.SUCCEEDED)
        await agent.robot.wait_for_task(handle.task_id, timeout=5.0)
        assert (await agent.robot.task_status(handle.task_id)).succeeded


async def test_submit_knobs_travel_as_query_parameters(monkeypatch) -> None:
    """preempt and on_estop are selector parameters, not payload fields: the
    goal on the wire has to stay exactly the goal."""
    seen: dict[str, str] = {}

    async def _capture(session, service, key, request, **kwargs):
        seen["key"] = key
        seen["wire"] = service.request_codec.encode(request).decode()
        from robodog_sdk import TaskHandle

        return TaskHandle(task_id="t1", accepted=True)

    async with harness() as h:
        agent, _ = await _agent_with_nav(h)
        monkeypatch.setattr("robodog_sdk.client.call_service", _capture)

        await agent.robot.submit(
            NavigateToPoseGoal(target=Pose2D(x=1.0, y=0.0, theta=0.0)),
            preempt=True,
            on_estop=EstopPolicy.HOLD,
        )

        selector = seen["key"]
        assert selector.split("?")[0].endswith("nav/task/submit")
        assert "preempt=true" in selector
        assert "on_estop=hold" in selector
        assert "client=test-agent" in selector
        assert "on_estop" not in seen["wire"], "the policy must not leak into the goal"


async def test_a_submit_holds_nothing_unless_it_says_so(monkeypatch) -> None:
    """The default discards the task on a stop. A route nobody is watching
    must not resume itself minutes after someone hit the button."""
    seen: dict[str, str] = {}

    async def _capture(session, service, key, request, **kwargs):
        seen["key"] = key
        from robodog_sdk import TaskHandle

        return TaskHandle(task_id="t1", accepted=True)

    async with harness() as h:
        agent, _ = await _agent_with_nav(h)
        monkeypatch.setattr("robodog_sdk.client.call_service", _capture)

        await agent.robot.submit(NavigateToPoseGoal(target=Pose2D(x=1.0, y=0.0, theta=0.0)))

        assert "on_estop=cancel" in seen["key"]


async def test_a_route_carries_its_corridor() -> None:
    async with harness() as h:
        agent, nav = await _agent_with_nav(h)

        await agent.robot.navigate_through(
            [(1.0, 0.0), (2.0, 0.0)], corridor_deviation_m=0.75, timeout=5.0
        )

        goal = nav.last_goal
        assert isinstance(goal, NavigateThroughPosesGoal)
        assert goal.corridor_deviation_m == 0.75


async def test_the_fused_pose_reaches_the_state_view() -> None:
    """Goals are expressed in the fused pose's frame, so a client that only
    saw raw odometry would be reasoning in the wrong one."""
    async with harness() as h:
        agent, stack = await _agent_with_stack(h)

        stack.set_pose(x=3.0, y=-1.0)
        await asyncio.sleep(0.2)

        assert agent.robot.state.localization.value is not None
        assert agent.robot.state.localization.value.x == 3.0


async def test_wait_for_nav_returns_when_the_coordinator_answers() -> None:
    """The coordinator holds no presence token, so liveness is a question it
    answers, not a token it holds."""
    async with harness() as h:
        agent, _ = await _agent_with_nav(h)

        await agent.robot.wait_for_nav(timeout=5.0)


async def test_wait_for_nav_times_out_when_nothing_serves_the_key() -> None:
    """Silence must not read as an answer. ``ServiceTimeout`` subclasses
    ``ServiceError``, so the "no such task" refusal that signals liveness is
    one catch away from swallowing the case where nav is simply absent."""
    async with harness() as h:
        agent = await h.start_node(_Agent)  # no FakeNav

        with pytest.raises(TimeoutError):
            await agent.robot.wait_for_nav(timeout=1.0, poll=0.2)


async def test_stack_nodes_are_not_waitable_by_presence() -> None:
    """The double is a zenode node and so has presence; the real nav node does
    not, which is the whole reason wait_for_nav exists."""
    async with harness() as h:
        agent, _ = await _agent_with_nav(h)

        await agent.robot.wait_until_ready("nav", timeout=5.0)  # the double, not the stack
        with pytest.raises(ValueError):
            await agent.robot.wait_until_ready()
