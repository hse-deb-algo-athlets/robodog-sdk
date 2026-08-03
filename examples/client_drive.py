"""Drive forward a fixed distance, using RobotClient.

    uv run python examples/client_drive.py

Equivalent to ``contract_drive.py``. The client supplies:

- ``driving()``, which republishes at 10 Hz and sends zero velocity on exit,
  removing the timer and the ``on_stop`` handler.
- ``control()``, which acquires and releases the agent lane.
- ``state``, which carries the latched pose and its age, removing the
  subscription and the first-message bookkeeping.

The behaviour runs as a background task rather than inline in ``on_start``:
decorated bindings are wired after ``on_start`` returns, so blocking there
would leave the handlers inactive.
"""

from __future__ import annotations

import asyncio
import math

from zenode import Node, NodeConfig, run, subscribe

from robodog_sdk import NavTopics, ProtectiveFieldEvent, RobotClient


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

    @subscribe(NavTopics.protective_field)
    async def on_protective_field(self, msg: ProtectiveFieldEvent) -> None:
        self._blocked = msg.active

    async def _run_drive(self) -> None:
        origin = await self._first_pose()
        self.log.info("drive started at %.2f, %.2f", origin[0], origin[1])

        async with (
            self.robot.control(reason="example drive"),
            self.robot.driving(x=self.config.speed),
        ):
            while not self._blocked and self._travelled_from(origin) < self.config.distance:
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
            # Stale pose: report no progress rather than an unfounded figure.
            # The drive continues; the distance simply stops advancing.
            return 0.0
        return math.hypot(pose.x - origin[0], pose.y - origin[1])


def cli() -> None:
    run(ClientDrive)


if __name__ == "__main__":
    cli()
