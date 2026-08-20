"""Navigation as a task: submit a goal, then wait for its outcome.

    uv run python examples/navigate.py

Two stages, one per call style. ``navigate_to`` submits and waits; ``submit``
returns a handle, leaving the wait to the caller.

A task ends in one of four states and only ``SUCCEEDED`` is arrival.
``BLOCKED`` is an outcome, not an error, so the client returns it rather than
raising.
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

    #: Seconds to wait for each stage before giving up on it.
    stage_timeout: float = 120.0


class Navigate(Node):
    name = "navigate"
    config: RouteConfig

    robot: RobotClient

    async def on_start(self) -> None:
        self.robot = RobotClient(self)
        self.spawn(self._run_route(), name="route")

    async def _run_route(self) -> None:
        # Not wait_until_ready("nav"): the coordinator holds no zenode presence
        # token, so that call times out even when nav is running.
        try:
            await self.robot.wait_for_nav(timeout=5.0)
        except TimeoutError:
            self.log.error("no navigation coordinator answered — is the nav node running?")
            self.stop()
            return

        # Stage one: submit and wait. Stage two starts from where this ends, so
        # anything other than SUCCEEDED ends the run.
        first = await self.robot.navigate_to(0, 0.0, timeout=self.config.stage_timeout)
        self._report("stage 1", first)
        if not first.succeeded:
            self.stop()
            return

        # Stage two: submit and wait separately, so the wait is ours to spend.
        handle = await self.robot.submit(self._stage_two_goal())
        if not handle.accepted:
            self.log.error("stage 2 refused: %s", handle.reason)
            self.stop()
            return

        watcher = self.spawn(self._watch(), name="watch")
        try:
            second = await self.robot.wait_for_task(
                handle.task_id, timeout=self.config.stage_timeout
            )
        except TimeoutError:
            # The wait timed out, not the task: cancelling is a separate call.
            self.log.warning("stage 2 outran its timeout — cancelling")
            await self.robot.cancel_task(handle.task_id)
            second = await self.robot.wait_for_task(handle.task_id, timeout=5.0)
        finally:
            watcher.cancel()

        self._report("stage 2", second)
        self.stop()

    @staticmethod
    def _stage_two_goal() -> NavigateThroughPosesGoal:
        # waypoint_follow drives the route as given; a planning skill may treat
        # the intermediate poses as advisory and cut the corner. The corridor
        # bounds a manual takeover: handed back more than 0.5 m off this line,
        # the task ends BLOCKED instead of resuming.
        return NavigateThroughPosesGoal(
            poses=[Pose2D(x=1, y=1, theta=0.0), Pose2D(x=0.0, y=1.5, theta=0.0)],
            skill="waypoint_follow",
            corridor_deviation_m=0.5,
        )

    async def _watch(self) -> None:
        """Log task progress.

        ``state`` is RUNNING throughout; ``activity`` is the field that moves.
        A stall is transient, so this reports it and keeps waiting.
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

    def _report(self, stage: str, result: TaskResult) -> None:
        if result.state is TaskState.SUCCEEDED:
            self.log.info("%s arrived", stage)
        elif result.state is TaskState.BLOCKED:
            self.log.warning("%s blocked: %s", stage, result.message)
        else:
            self.log.error("%s ended %s: %s", stage, result.state.value, result.message)


def cli() -> None:
    run(Navigate)


if __name__ == "__main__":
    cli()
