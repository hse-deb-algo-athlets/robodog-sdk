"""Drive forward a fixed distance, using ``RobotClient``.

    uv run python examples/client_drive.py

Equivalent to ``contract_drive.py``. The client supplies ``driving()``
(republishes at 10 Hz, stops on exit), ``state`` (latched pose and its age)
and ``preempted_by`` (who outranks us at the gateway).

Distance is measured against ``state.odometry`` rather than
``state.localization``: the fused pose jumps when SLAM corrects, which reads
here as having driven backwards.

The drive runs as a background task because decorated bindings are wired only
after ``on_start`` returns.
"""

from __future__ import annotations

import asyncio
import math

from zenode import Node, NodeConfig, run, subscribe

from robodog_sdk import CollisionZoneEvent, RobotClient, SafetyTopics


class DriveConfig(NodeConfig):
    """Configuration, loaded from ``[node.client-drive]``."""

    speed: float = 0.3  # m/s
    distance: float = 2.0  # m


class ClientDrive(Node):
    name = "client-drive"
    config: DriveConfig

    #: Acquired in on_start: the client needs the node's session.
    robot: RobotClient
    _blocked: bool

    async def on_start(self) -> None:
        self.robot = RobotClient(self)
        self._blocked = False
        self.spawn(self._run_drive(), name="drive")

    @subscribe(SafetyTopics.collision_zone)
    async def on_collision_zone(self, msg: CollisionZoneEvent) -> None:
        self._blocked = msg.active

    async def _run_drive(self) -> None:
        origin = await self._first_pose()
        self.log.info("drive started at %.2f, %.2f", origin[0], origin[1])

        async with self.robot.driving(x=self.config.speed):
            while not self._blocked and self._travelled_from(origin) < self.config.distance:
                if (driver := self.robot.preempted_by) is not None:
                    # The gateway re-decides every frame, so continuing to
                    # publish would resume the drive once they let go. Giving
                    # up is this example's choice, not the contract's.
                    self.log.info("yielding to %s", driver.value)
                    break
                await asyncio.sleep(0.1)

        self.log.info(
            "drive finished after %.2f m%s",
            self._travelled_from(origin),
            " (blocked)" if self._blocked else "",
        )
        self.stop()

    async def _first_pose(self) -> tuple[float, float]:
        """Return the starting pose. Latched, so this normally returns at once."""
        while (pose := self.robot.state.odometry.value) is None:
            await asyncio.sleep(0.05)
        return pose.x, pose.y

    def _travelled_from(self, origin: tuple[float, float]) -> float:
        pose = self.robot.state.odometry.value
        if pose is None or self.robot.state.odometry.age > 1.0:
            # Stale pose: report no progress rather than a figure we cannot
            # support. The drive continues; the distance stops advancing.
            return 0.0
        return math.hypot(pose.x - origin[0], pose.y - origin[1])


def cli() -> None:
    run(ClientDrive)


if __name__ == "__main__":
    cli()
