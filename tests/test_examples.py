"""The examples run and behave as documented.

Each is started against the doubles in ``robodog_sdk.testing`` in-process, so
an example that stops working fails CI rather than a reader.
"""

from __future__ import annotations

import asyncio

import client_drive
import contract_drive
import navigate
import pytest
from zenode.testing import harness

from robodog_sdk import (
    CollisionZoneEvent,
    MotionTopics,
    MovementSource,
    SafetyTopics,
    TaskState,
)
from robodog_sdk.testing import FakeNav, FakeStack

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


async def test_contract_drive_stops_on_collision_zone() -> None:
    async with harness() as h:
        stack = await h.start_node(FakeStack)
        await h.start_node(
            contract_drive.ContractDrive,
            config=contract_drive.DriveConfig(speed=0.3, distance=100.0, rate_hz=50.0),
        )
        field = h.publisher(SafetyTopics.collision_zone)

        stack.set_pose(x=0.0, y=0.0)
        await asyncio.sleep(SETTLE)
        assert not stack.stopped

        field.put(CollisionZoneEvent(active=True, zone_name="stop", distance_m=0.4))
        await asyncio.sleep(SETTLE)
        assert stack.stopped, "a breached collision zone must stop the drive"


async def test_client_drive_drives_then_stops() -> None:
    async with harness() as h:
        stack = await h.start_node(FakeStack)
        await h.start_node(
            client_drive.ClientDrive,
            config=client_drive.DriveConfig(speed=0.3, distance=1.0),
        )

        stack.set_pose(x=0.0, y=0.0)
        await asyncio.sleep(SETTLE)
        assert stack.last_command is not None
        assert stack.last_command.x == pytest.approx(0.3)

        stack.set_pose(x=2.0, y=0.0)
        await asyncio.sleep(SETTLE + 0.2)
        assert stack.stopped, "leaving driving() must leave the robot stopped"


async def test_examples_publish_on_the_inlet_never_the_gateway_output() -> None:
    async with harness() as h:
        stack = await h.start_node(FakeStack)
        gateway_output = h.collect(MotionTopics.move)  # the gateway's key: stays empty
        await h.start_node(
            client_drive.ClientDrive, config=client_drive.DriveConfig(speed=0.2, distance=100.0)
        )

        stack.set_pose(x=0.0, y=0.0)
        await asyncio.sleep(SETTLE)

        assert stack.commands, "FakeStack only subscribes to the inlet"
        assert not gateway_output.items, "publishing the gateway's output bypasses it"
        assert all(c.source is MovementSource.autonomous for c in stack.commands)


async def test_navigate_runs_both_stages_and_stops() -> None:
    async with harness() as h:
        await h.start_node(FakeStack)
        nav = await h.start_node(FakeNav)
        await h.start_node(navigate.Navigate, config=navigate.RouteConfig(stage_timeout=5.0))

        for _ in range(50):
            if len(nav.goals) >= 2:
                break
            await asyncio.sleep(0.05)

        assert len(nav.goals) == 2, "both stages should have been submitted"
        assert nav.goals[1].skill == "waypoint_follow"


async def test_navigate_abandons_the_route_when_the_first_stage_blocks() -> None:
    async with harness() as h:
        await h.start_node(FakeStack)
        nav = await h.start_node(FakeNav)
        nav.result_state = TaskState.BLOCKED
        await h.start_node(navigate.Navigate, config=navigate.RouteConfig(stage_timeout=5.0))

        await asyncio.sleep(SETTLE + 0.3)
        assert len(nav.goals) == 1, "a blocked first stage must not start the second"
