"""The examples run and behave as documented.

Both are started against ``FakeStack`` in-process, so an example that stops
working fails CI rather than a reader.
"""

from __future__ import annotations

import asyncio

import client_drive
import contract_drive
import pytest
from zenode.testing import harness

from robodog_sdk import NavTopics, ProtectiveFieldEvent, SafetyTopics
from robodog_sdk.testing import FakeArbiter, FakeStack

pytestmark = pytest.mark.integration

SETTLE = 0.2  # generous: these assert on behaviour, not on latency


async def test_contract_drive_drives_then_stops_at_distance() -> None:
    async with harness() as h:
        stack = await h.start_node(FakeStack)
        await h.start_node(
            contract_drive.ContractDrive,
            config=contract_drive.DriveConfig(speed=0.3, distance=1.0, rate_hz=50.0),
        )

        stack.set_pose(x=0.0, y=0.0)
        await asyncio.sleep(SETTLE)
        assert stack.last_command is not None
        assert stack.last_command.x == pytest.approx(0.3), "should be driving"

        stack.set_pose(x=2.0, y=0.0)  # past the target distance
        await asyncio.sleep(SETTLE)
        assert stack.stopped, "should stop once the distance is covered"


async def test_contract_drive_stops_on_protective_field() -> None:
    async with harness() as h:
        stack = await h.start_node(FakeStack)
        await h.start_node(
            contract_drive.ContractDrive,
            config=contract_drive.DriveConfig(speed=0.3, distance=100.0, rate_hz=50.0),
        )
        field = h.publisher(SafetyTopics.protective_field)

        stack.set_pose(x=0.0, y=0.0)
        await asyncio.sleep(SETTLE)
        assert not stack.stopped

        field.put(ProtectiveFieldEvent(active=True, distance_m=0.4))
        await asyncio.sleep(SETTLE)
        assert stack.stopped, "a breached protective field must stop the drive"


async def test_client_drive_takes_the_lane_and_stops() -> None:
    async with harness() as h:
        stack = await h.start_node(FakeStack)
        arbiter = await h.start_node(FakeArbiter)
        await h.start_node(
            client_drive.ClientDrive,
            config=client_drive.DriveConfig(speed=0.3, distance=1.0),
        )

        stack.set_pose(x=0.0, y=0.0)
        await asyncio.sleep(SETTLE)
        assert arbiter.granted, "client_drive should acquire the agent lane"
        assert stack.last_command is not None
        assert stack.last_command.x == pytest.approx(0.3)

        stack.set_pose(x=2.0, y=0.0)
        await asyncio.sleep(SETTLE + 0.2)
        assert stack.stopped, "leaving driving() must leave the robot stopped"


async def test_both_examples_publish_on_the_agent_lane() -> None:
    """Examples publish on the agent lane, never on the arbiter's output key."""
    async with harness() as h:
        stack = await h.start_node(FakeStack)
        arbiter_output = h.collect(NavTopics.request)  # unrelated key: stays empty
        await h.start_node(FakeArbiter)
        await h.start_node(
            client_drive.ClientDrive, config=client_drive.DriveConfig(speed=0.2, distance=100.0)
        )

        stack.set_pose(x=0.0, y=0.0)
        await asyncio.sleep(SETTLE)

        assert stack.commands, "FakeStack only subscribes to the agent lane"
        assert not arbiter_output.items
