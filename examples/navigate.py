"""Navigate a route, and react to how it ends.

    uv run python examples/navigate.py

Navigation is a task, not a command: the submit returns an id and the outcome
arrives later, on that id's result key. This example shows the two halves of
that separately —

- ``navigate_to`` submits and waits, which is the short form and enough for
  most scripts;
- the second leg submits without waiting, watches ``state.nav`` while the task
  runs, and collects the result afterwards.

A task can end in four ways and only one of them is arrival. ``BLOCKED`` in
particular is not an error: the skill met the world and stopped. Deciding what
to do about it — wait, route around, ask a human — is the caller's job, which
is why the client returns the result rather than raising.
"""

from __future__ import annotations

import asyncio

from zenode import Node, NodeConfig, run

from robodog_sdk import (
    NavigateThroughPosesGoal,
    Pose2D,
    RobotClient,
    TaskResult,
    TaskState,
)


class RouteConfig(NodeConfig):
    """Configuration, loaded from ``[node.navigate]``."""

    #: Seconds to wait for each leg. None would wait forever, which is right
    #: for a skill that waits out an obstacle and wrong for an example.
    leg_timeout: float = 120.0


class Navigate(Node):
    name = "navigate"
    config: RouteConfig

    robot: RobotClient

    async def on_start(self) -> None:
        self.robot = RobotClient(self)
        self.spawn(self._run_route(), name="route")

    async def _run_route(self) -> None:
        await self.robot.wait_until_ready("nav", timeout=5.0)

        # Leg one: submit and wait. Anything that is not SUCCEEDED ends the run
        # — there is no point driving leg two from somewhere we did not reach.
        first = await self.robot.navigate_to(2.0, 0.0, timeout=self.config.leg_timeout)
        self._report("leg 1", first)
        if not first.succeeded:
            self.stop()
            return

        # Leg two: the same thing taken apart, so the wait is ours to spend.
        handle = await self.robot.submit(self._leg_two_goal())
        if not handle.accepted:
            self.log.error("leg 2 refused: %s", handle.reason)
            self.stop()
            return

        watcher = self.spawn(self._watch(), name="watch")
        try:
            second = await self.robot.wait_for_task(handle.task_id, timeout=self.config.leg_timeout)
        except TimeoutError:
            # The task is still running: the wait timed out, not the robot.
            # Stopping it is a separate decision, and an explicit one.
            self.log.warning("leg 2 outran its timeout — cancelling")
            await self.robot.cancel_task(handle.task_id)
            second = await self.robot.wait_for_task(handle.task_id, timeout=5.0)
        finally:
            watcher.cancel()

        self._report("leg 2", second)
        self.stop()

    @staticmethod
    def _leg_two_goal() -> NavigateThroughPosesGoal:
        # waypoint_follow drives the route it is given. A planning skill is
        # free to treat the intermediate poses as advisory and cut the corner,
        # which is not what a route is for.
        #
        # The corridor is the other half of meaning it: if a human takes the
        # gamepad mid-leg and lets go more than half a metre off this line,
        # the task ends BLOCKED rather than resuming from wherever it now is.
        return NavigateThroughPosesGoal(
            poses=[Pose2D(x=2.0, y=1.5, theta=0.0), Pose2D(x=0.0, y=1.5, theta=0.0)],
            skill="waypoint_follow",
            corridor_deviation_m=0.5,
        )

    async def _watch(self) -> None:
        """Log progress while a task runs. Feedback is not latched, so a gap
        here means the skill stopped publishing, not that it finished.

        ``state`` is RUNNING for the whole leg and says nothing useful here;
        ``activity`` is the field that moves. A stall is not a failure — the
        skill is still trying — so this reports it and keeps waiting.
        """
        while True:
            feedback = self.robot.state.nav.value
            if feedback is not None and self.robot.navigating:
                where = ""
                if feedback.current_segment_index is not None:
                    where = f" [{feedback.current_segment_index + 1}/{feedback.total_segments}]"
                self.log.info(
                    "%s: %.2f m to go — %s%s%s",
                    feedback.task_id[:8],
                    feedback.distance_to_goal or float("nan"),
                    feedback.activity.value,
                    f" ({feedback.note})" if feedback.note else "",
                    where,
                )
            await asyncio.sleep(1.0)

    def _report(self, leg: str, result: TaskResult) -> None:
        if result.state is TaskState.SUCCEEDED:
            self.log.info("%s arrived", leg)
        elif result.state is TaskState.BLOCKED:
            self.log.warning("%s blocked: %s", leg, result.message)
        else:
            self.log.error("%s ended %s: %s", leg, result.state.value, result.message)


def cli() -> None:
    run(Navigate)


if __name__ == "__main__":
    cli()
