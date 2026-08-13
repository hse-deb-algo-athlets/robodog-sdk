"""Drive forward a fixed distance, using the contract directly.

    uv run python examples/contract_drive.py

Equivalent to ``client_drive.py``, written against ``Topic`` declarations
rather than :class:`~robodog_sdk.RobotClient`. Compare the two to see what the
client provides.

Handled explicitly here:

- Republishing. A command expires after ``COMMAND_MAX_AGE``, so sustained
  motion requires a timer rather than a single ``put()``.
- Stopping on shutdown. ``on_stop`` publishes zero velocity; without it the
  robot continues until the deadman elapses.
- The command's rank. ``source=autonomous`` is what makes the gateway hand the
  robot to a human the moment one touches the gamepad, and it is carried on
  every frame rather than agreed once.
- Trace continuity across the timer. ``state/odometry`` is a trace root, but a
  timer body runs outside any trace, so the pose's context is captured in the
  handler and restored before publishing.
"""

from __future__ import annotations

import math

from zenode import Node, NodeConfig, every, publish, run, subscribe, trace

from robodog_sdk import (
    CollisionZoneEvent,
    MotionTopics,
    MovementCommand,
    MovementSource,
    OdometryState,
    SafetyTopics,
    StateTopics,
)


class DriveConfig(NodeConfig):
    """Configuration, loaded from ``[node.contract-drive]``."""

    speed: float = 0.3  # m/s
    distance: float = 2.0  # m
    rate_hz: float = 10.0  # republish rate; must be well inside COMMAND_MAX_AGE


class ContractDrive(Node):
    name = "contract-drive"
    config: DriveConfig

    #: Materialized before ``on_start`` and usable from it. The gateway inlet,
    #: not ``move``: that key is the gateway's output, and publishing there
    #: would bypass both arbitration and the collision zones.
    cmd = publish(MotionTopics.request)

    # Declared here, assigned in on_start: plain state needs no session, but
    # declaring it keeps the node's attributes discoverable in one place.
    _origin: tuple[float, float] | None
    _travelled: float
    _blocked: bool
    #: Trace context of the pose the current distance is derived from.
    _pose_trace: str | None

    async def on_start(self) -> None:
        self._origin = None
        self._travelled = 0.0
        self._blocked = False
        self._pose_trace = None

    @subscribe(StateTopics.odometry, mode="latest")
    async def on_odometry(self, msg: OdometryState) -> None:
        """Track distance travelled. ``mode="latest"``: only the newest pose matters."""
        if self._origin is None:
            self._origin = (msg.x, msg.y)
            self.log.info("drive started at %.2f, %.2f", msg.x, msg.y)
        self._travelled = math.hypot(msg.x - self._origin[0], msg.y - self._origin[1])
        self._pose_trace = trace.current()

    @subscribe(SafetyTopics.collision_zone)
    async def on_collision_zone(self, msg: CollisionZoneEvent) -> None:
        """Handle a collision-zone transition: one message per edge."""
        self._blocked = msg.active
        self.log.warning("collision zone %s", "breached" if msg.active else "clear")

    @every("rate_hz", unit="hz")
    async def tick(self) -> None:
        if self._origin is None:
            return  # no pose yet — publishing would be driving blind
        # A timer is caused by the clock, not by a message, so it runs outside
        # any trace. Restoring the pose's context keeps the command linked to
        # the measurement it was derived from.
        with trace.using(self._pose_trace):
            if self._blocked or self._travelled >= self.config.distance:
                self.cmd.put(MovementCommand(source=MovementSource.autonomous))
                return
            self.cmd.put(MovementCommand(x=self.config.speed, source=MovementSource.autonomous))

    async def on_stop(self) -> None:
        """Stop the robot. Runs before the session closes, so this is delivered."""
        self.cmd.put(MovementCommand(source=MovementSource.autonomous))


def cli() -> None:
    run(ContractDrive)


if __name__ == "__main__":
    cli()
