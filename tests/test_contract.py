"""Contract-wide invariants.

These tests iterate the topic registry, so they cover every topic currently
declared and every topic added later without being modified.
"""

from __future__ import annotations

import pytest
from pydantic import ValidationError
from zenode import registered_services, registered_topics

import robodog_sdk  # noqa: F401  — importing registers the TopicSets
from robodog_sdk import (
    EstopPhase,
    GridMap,
    MapIdentity,
    MovementCommand,
    MovementSource,
    NavigateThroughPosesGoal,
    NavigateToPoseGoal,
    PlannedPath,
    Pose2D,
    SafetyState,
    TaskFeedback,
    TaskGoalEnvelope,
    TaskState,
)
from robodog_sdk.topics import (
    ControlTopics,
    LocalizationTopics,
    MapTopics,
    MotionTopics,
    NavServices,
    NavTopics,
    SafetyTopics,
    safety_source_key,
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
    """It changes on edges, so a late joiner would otherwise learn who is
    driving only at the next change — which may never come. The gateway also
    re-asserts it on a heartbeat; the two cover different failure modes and
    neither replaces the other."""
    assert ControlTopics.status.latched


def test_the_safety_latch_is_latched() -> None:
    """A node joining during a stop learns of it without waiting for an edge."""
    assert SafetyTopics.state.latched
    assert SafetyTopics.estop.latched, "the legacy mirror carries the same duty"


def test_safety_defaults_deny_motion() -> None:
    """An unpopulated latch must read as stopped. Every default here is a
    decision about what happens when a field is forgotten."""
    fresh = SafetyState()
    assert fresh.estop and not fresh.source_alive
    assert not fresh.motion_permitted


def test_motion_is_denied_while_the_robot_is_standing_back_up() -> None:
    """``estop`` drops early so the bridge can start the recovery; motion has
    to stay denied until it finishes, or the gate is meaningless."""
    releasing = SafetyState(estop=False, source_alive=True, phase=EstopPhase.RELEASING)
    assert not releasing.estop
    assert not releasing.motion_permitted


def test_an_unpopulated_map_identity_is_not_a_map() -> None:
    """A partly-understood payload must fail closed. Defaulting ``map_id`` to
    anything but ``None`` would let stored coordinates be driven against a map
    nobody confirmed."""
    identity = MapIdentity()
    assert identity.map_id is None
    assert identity.reachable is False


def test_no_map_is_distinguishable_from_no_answer() -> None:
    """Two "no map" answers a consumer must not conflate: a producer that
    answered and has no map will not gain one by waiting, whereas a producer
    that never published shows up as a stale stamp instead."""
    answered = MapIdentity(source="nav", state="no_global_map", reachable=True)
    assert answered.map_id is None
    assert answered.reachable


def test_the_map_identity_is_latched() -> None:
    """Anything storing a map-frame coordinate needs the map before it can use
    one, so a late joiner cannot be made to wait for the next republish."""
    assert LocalizationTopics.map_identity.latched


def test_a_safety_source_key_matches_the_wildcard_the_aggregator_watches() -> None:
    prefix, _, suffix = SafetyTopics.source.key.partition("*")
    assert safety_source_key("panel-1") == f"{prefix}panel-1{suffix}"


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
    """It is republished slowly, so a late joiner would otherwise plan blind
    until the next period."""
    assert NavTopics.costmap_global.latched


def test_there_is_no_state_for_a_task_nobody_remembers() -> None:
    """An unknown task is answered on the error channel. A placeholder state
    would be indistinguishable from a real one, which is the whole problem."""
    assert not hasattr(TaskState, "PENDING")
    assert set(TaskState) == {
        TaskState.RUNNING,
        TaskState.SUCCEEDED,
        TaskState.FAILED,
        TaskState.CANCELED,
        TaskState.BLOCKED,
    }


def test_feedback_cannot_carry_a_terminal_state() -> None:
    """Feedback streams only while the task is alive, so narrowing the field
    is what makes it impossible for a frame and the result to disagree.

    The narrowing is enforced twice, which is the point: pyright rejects the
    line below at author time — hence the ignore, which *is* half the assertion
    — and Pydantic rejects the same value arriving off the wire.
    """
    assert TaskFeedback(task_id="t").state is TaskState.RUNNING
    with pytest.raises(ValidationError):
        TaskFeedback(task_id="t", state=TaskState.SUCCEEDED)  # type: ignore[arg-type]


def test_terminal_states_are_exactly_the_four_endings() -> None:
    terminal = {state for state in TaskState if state.is_terminal}
    assert terminal == {
        TaskState.SUCCEEDED,
        TaskState.FAILED,
        TaskState.CANCELED,
        TaskState.BLOCKED,
    }


def test_a_dwell_must_name_the_pose_it_belongs_to() -> None:
    """A mismatched dwell list is silently ambiguous — there is no way to tell
    which poses the entries were meant for — so it is refused, not guessed."""
    poses = [Pose2D(x=float(i), y=0.0, theta=0.0) for i in range(3)]
    assert NavigateThroughPosesGoal(poses=poses, dwell_sec=[0.0, 2.0, 0.0]).dwell_sec
    assert NavigateThroughPosesGoal(poses=poses).dwell_sec is None
    with pytest.raises(ValidationError):
        NavigateThroughPosesGoal(poses=poses, dwell_sec=[1.0])
    with pytest.raises(ValidationError):
        NavigateThroughPosesGoal(poses=poses, dwell_sec=[0.0, -1.0, 0.0])


def test_a_goal_pose_theta_does_not_ask_for_an_arrival_heading() -> None:
    """Every pose carries a theta — the approach hint — so it cannot double as
    "I care how I end up facing". Only the dedicated field opts in."""
    goal = NavigateToPoseGoal(target=Pose2D(x=1.0, y=0.0, theta=1.57))
    assert goal.orientation_at_target is None, "a theta alone is a don't-care"
    assert not PlannedPath(task_id="t", waypoints=[]).align_final_heading


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


# ── the map grid ────────────────────────────────────────────────────────────

#: One `map/grid` payload exactly as the MOLA control plane emits it, captured
#: from a real session. MOLA cannot import this package — it runs on the ROS 2
#: Humble image, whose Python is older than this package supports — so it
#: hand-writes this JSON, and nothing but this fixture stands between a field
#: renamed here and a map that silently stops decoding on the robot. The PNG is
#: a 4x3 greyscale raster: a wall around two free cells.
MOLA_GRID_PAYLOAD = (
    r'{"map_id": "session1", "revision": "W/\"3706-1752163200\"",'
    r' "resolution": 0.05000000074505806,'
    r' "origin_x": -6.400000095367432, "origin_y": -5.25, "origin_theta": 0.0,'
    r' "width": 4, "height": 3,'
    r' "occupied_thresh": 0.65, "free_thresh": 0.196, "negate": false,'
    r' "image_png": "iVBORw0KGgoAAAANSUhEUgAAAAQAAAADCAAAAACRn/EaAAAAD0lEQVR4nGNg'
    r'AIN//yA0AA7xAf15n65LAAAAAElFTkSuQmCC",'
    r' "source": "mola", "stamp": "2026-07-10T18:00:00+00:00"}'
)


def test_the_map_grid_is_latched() -> None:
    """It is published on change and never on a timer, so a consumer that is
    not latched into the cache waits for a remap that may never come."""
    assert MapTopics.grid.latched


def test_the_map_identity_is_latched_and_agrees_with_the_grid() -> None:
    """Both come from the localization source, so a consumer that has one has
    the other — and neither makes a late joiner wait for a change."""
    assert LocalizationTopics.map_identity.latched
    assert MapTopics.grid.latched


def test_a_mola_payload_decodes() -> None:
    """The contract as the producer actually writes it."""
    grid = MapTopics.grid.codec.decode(MOLA_GRID_PAYLOAD.encode())
    assert grid.map_id == "session1"
    assert (grid.width, grid.height) == (4, 3)
    assert grid.resolution == pytest.approx(0.05)
    assert (grid.origin_x, grid.origin_y) == pytest.approx((-6.4000000954, -5.25))
    assert grid.available
    assert grid.png_bytes().startswith(b"\x89PNG\r\n\x1a\n")


def test_an_empty_grid_map_is_no_map() -> None:
    """Every default has to read as "I cannot draw you a map", so a payload
    that is only partly understood is never mistaken for a usable one."""
    empty = GridMap()
    assert not empty.available
    assert empty.map_id is None
    assert empty.png_bytes() == b""


def test_a_session_without_a_grid_is_not_a_missing_session() -> None:
    """Mapping in progress: the frame is live, the raster does not exist yet.
    A consumer holding map-frame coordinates must not read this as "no map"
    and throw them away."""
    building = GridMap(map_id="hall-b_2026-07")
    assert building.map_id == "hall-b_2026-07"
    assert not building.available


def test_occupancy_thresholds_follow_map_server() -> None:
    """Dark is occupied, light is free, and the band between is unknown —
    getting this backwards reads every wall as open floor."""
    grid = GridMap(occupied_thresh=0.65, free_thresh=0.196)
    assert grid.is_occupied(0) and not grid.is_free(0)
    assert grid.is_free(255) and not grid.is_occupied(255)
    assert not grid.is_occupied(128) and not grid.is_free(128)


def test_negate_inverts_the_greyscale() -> None:
    """The flag is on the message because the raster may carry either
    convention; a consumer doing its own arithmetic has to honour it."""
    assert GridMap(negate=True).is_free(0)
    assert GridMap(negate=True).is_occupied(255)
