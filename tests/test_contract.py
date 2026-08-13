"""Contract-wide invariants.

These tests iterate the topic registry, so they cover every topic currently
declared and every topic added later without being modified.
"""

from __future__ import annotations

import pytest
from zenode import registered_services, registered_topics

import robodog_sdk  # noqa: F401  — importing registers the TopicSets
from robodog_sdk import (
    MovementCommand,
    MovementSource,
    NavigateThroughPosesGoal,
    NavigateToPoseGoal,
    Pose2D,
    TaskGoalEnvelope,
    TaskState,
)
from robodog_sdk.topics import (
    ControlTopics,
    MotionTopics,
    NavServices,
    NavTopics,
    SafetyTopics,
    task_feedback_key,
    task_result_key,
    task_status_service,
)

TOPICS = [(entry.owner + "." + entry.attr, topic) for entry, topic in registered_topics()]
SERVICES = [(entry.owner + "." + entry.attr, svc) for entry, svc in registered_services()]


def _example(model: type) -> object:
    """Build an instance with every default accepted, or skip if it can't be."""
    try:
        return model()
    except Exception:  # required fields — those get their own targeted tests
        pytest.skip(f"{model.__name__} has required fields")


def test_contract_is_not_empty() -> None:
    assert TOPICS, "no topics registered — did the TopicSet import get dropped?"


@pytest.mark.parametrize(("name", "topic"), TOPICS, ids=[n for n, _ in TOPICS])
def test_roundtrip(name: str, topic) -> None:
    """Encoding and decoding returns an equal message, for every topic."""
    if topic.schema is bytes:
        payload = b"\x00\x01\x02"
        assert topic.codec.decode(topic.codec.encode(payload)) == payload
        return
    sample = _example(topic.schema)
    assert topic.codec.decode(topic.codec.encode(sample)) == sample


@pytest.mark.parametrize(("name", "topic"), TOPICS, ids=[n for n, _ in TOPICS])
def test_keys_are_namespaced(name: str, topic) -> None:
    """Relative keys take the deployment namespace; absolute keys do not.

    A relative key that ignored the namespace would be visible across
    sandboxes sharing a network.
    """
    resolved = topic.resolve("robodog")
    if topic.is_absolute:
        assert resolved == topic.key
    else:
        assert resolved == f"robodog/{topic.key}"


@pytest.mark.parametrize(("name", "svc"), SERVICES, ids=[n for n, _ in SERVICES])
def test_service_roundtrip(name: str, svc) -> None:
    request = _example(svc.request)
    reply = _example(svc.reply)
    assert svc.request_codec.decode(svc.request_codec.encode(request)) == request
    assert svc.reply_codec.decode(svc.reply_codec.encode(reply)) == reply


def test_command_topics_are_perishable() -> None:
    """Both movement keys expire, which is what makes the deadman work."""
    for topic in (MotionTopics.request, MotionTopics.move):
        assert topic.max_age is not None, f"{topic.key} would drive on stale commands"


def test_movement_priority_is_ordered_lowest_first() -> None:
    """Higher priority wins, and a human wins over everything."""
    ranked = sorted(MovementSource, key=lambda s: s.priority)
    assert ranked == [
        MovementSource.autonomous,
        MovementSource.planner,
        MovementSource.assisted_teleop,
        MovementSource.controller,
    ]
    assert MovementSource.controller.outranks(MovementSource.planner)
    assert not MovementSource.autonomous.outranks(MovementSource.planner)


def test_a_default_command_cannot_outrank_a_human() -> None:
    """The default source is the one that takes the robot from nobody."""
    assert MovementCommand().source is MovementSource.autonomous
    assert MovementCommand().source.priority == 0


def test_gateway_status_is_latched() -> None:
    """It is edge-published, so a late joiner would otherwise learn who is
    driving only at the next change — which may never come."""
    assert ControlTopics.status.latched


def test_estop_is_latched() -> None:
    """A node joining during a stop learns of it without waiting for an edge."""
    assert SafetyTopics.estop.latched


def test_task_keys_agree_with_the_wildcards_they_are_observed_through() -> None:
    """A concrete per-task key must match the wildcard a client subscribes to."""
    task_id = "0123456789abcdef"
    prefix, _, suffix = NavTopics.feedback.key.partition("*")
    assert task_feedback_key(task_id) == f"{prefix}{task_id}{suffix}"
    prefix, _, suffix = NavTopics.result.key.partition("*")
    assert task_result_key(task_id) == f"{prefix}{task_id}{suffix}"
    prefix, _, suffix = NavServices.status.key.partition("*")
    assert task_status_service(task_id).key == f"{prefix}{task_id}{suffix}"


def test_task_result_is_not_latched() -> None:
    """One message, never replayed — which is why the client subscribes up
    front and why the status query exists at all."""
    assert not NavTopics.result.latched
    assert not NavTopics.feedback.latched


def test_global_costmap_is_latched() -> None:
    """The map is published once at startup; a late joiner still needs it."""
    assert NavTopics.costmap_global.latched


def test_terminal_states_are_exactly_the_four_endings() -> None:
    terminal = {state for state in TaskState if state.is_terminal}
    assert terminal == {
        TaskState.SUCCEEDED,
        TaskState.FAILED,
        TaskState.CANCELED,
        TaskState.BLOCKED,
    }


@pytest.mark.parametrize(
    "goal",
    [
        NavigateToPoseGoal(target=Pose2D(x=1.0, y=2.0, theta=0.5)),
        NavigateThroughPosesGoal(poses=[Pose2D(x=1.0, y=2.0, theta=0.0)]),
    ],
    ids=["navigate_to_pose", "navigate_through_poses"],
)
def test_goal_roundtrips_through_the_submit_service(goal) -> None:
    """The envelope adds nothing to the wire, and the discriminator picks the
    right goal type back out of it."""
    codec = NavServices.submit.request_codec
    wire = codec.encode(TaskGoalEnvelope(goal))
    assert b'"kind":' in wire
    assert codec.decode(wire).goal == goal
