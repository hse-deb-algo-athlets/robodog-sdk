"""Test double for the control stack.

``zenode.testing`` provides an in-process session; this module provides the
counterparty — state on the latched topics, an arbiter that grants lanes, and
a record of the commands a node under test issued::

    from zenode.testing import harness
    from robodog_sdk.testing import FakeStack

    async with harness() as h:
        stack = await h.start_node(FakeStack)
        await h.start_node(MyAgent)

Nothing here integrates motion or reacts to a command. Use it to test how a
node responds to state; use the MuJoCo simulation to test whether the robot
reaches a target.
"""

from __future__ import annotations

from zenode import Node, publish, serve, subscribe

from .msgs.control import ArbiterStatus, ControlGrant, ControlRequest
from .msgs.motion import MovementCommand
from .msgs.robot import BatteryLevel, BatteryState, ConnectionStatus, OdometryState
from .topics import ControlServices, ControlTopics, MotionTopics, StateTopics


class FakeStack(Node):
    """Publishes latched robot state and grants every lane request."""

    name = "fake-stack"
    health_interval = None

    odometry = publish(StateTopics.odometry)
    battery = publish(StateTopics.battery)
    connection = publish(StateTopics.connection)
    arbiter = publish(ControlTopics.status)

    #: Every ``MovementCommand`` received on the agent lane, in order.
    commands: list[MovementCommand]
    #: Every lane request received, in order.
    granted: list[ControlRequest]

    async def on_start(self) -> None:
        self.commands = []
        self.granted = []
        self.odometry.put(OdometryState())
        self.battery.put(BatteryState(soc=87, level=BatteryLevel.good, voltage=28.4))
        self.connection.put(ConnectionStatus(connected=True, motion_mode="normal"))
        self.arbiter.put(ArbiterStatus(active_lane=None))

    @subscribe(MotionTopics.move_agent)
    async def on_agent_command(self, msg: MovementCommand) -> None:
        self.commands.append(msg)

    @serve(ControlServices.acquire)
    async def on_acquire(self, request: ControlRequest) -> ControlGrant:
        self.granted.append(request)
        return ControlGrant(granted=True, lane=request.lane, holder=request.node)

    def set_pose(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        """Publish a new pose. The transition is instantaneous."""
        self.odometry.put(OdometryState(x=x, y=y, z=z))

    def set_battery(self, soc: int, level: BatteryLevel = BatteryLevel.good) -> None:
        """Publish a new battery state."""
        self.battery.put(BatteryState(soc=soc, level=level))

    @property
    def last_command(self) -> MovementCommand | None:
        """The most recent command on the agent lane, if any."""
        return self.commands[-1] if self.commands else None

    @property
    def stopped(self) -> bool:
        """Whether the most recent command was zero velocity."""
        last = self.last_command
        return last is not None and last.is_zero()
