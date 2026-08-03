"""Contract-wide invariants.

These tests iterate the topic registry, so they cover every topic currently
declared and every topic added later without being modified.
"""

from __future__ import annotations

import pytest
from zenode import registered_services, registered_topics

import robodog_sdk  # noqa: F401  — importing registers the TopicSets
from robodog_sdk.topics import MotionTopics, SafetyTopics

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
    """Every movement lane expires, which is what makes the deadman work."""
    lanes = [
        MotionTopics.move,
        MotionTopics.move_teleop,
        MotionTopics.move_nav,
        MotionTopics.move_agent,
    ]
    for lane in lanes:
        assert lane.max_age is not None, f"{lane.key} would drive on stale commands"


def test_estop_is_latched() -> None:
    """A node joining during a stop learns of it without waiting for an edge."""
    assert SafetyTopics.estop.latched
