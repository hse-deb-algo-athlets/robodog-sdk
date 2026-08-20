# Changelog

Consumers pin a tag, so this file is how they learn what a bump costs them.
Semantic versioning; `0.x` minor versions may move keys.

## [Unreleased]

### The contract now says what the stack actually does

This package was written to a key scheme the stack was expected to adopt and
did not. Nine keys were therefore addressing nothing at all — a subscriber saw
silence, with no error, which is the worst way for a contract to be wrong. Every
key is now checked against `robodog-digipro:src/interfaces/topics/topics.py`,
and there is no drift left.

| was (addressed nothing) | is |
|---|---|
| `state/{highstate,odometry,battery,motor}` | `system_state/*` |
| `control/status` | `motion/gateway/status` |
| `safety/estop` | `command/motion/estop` |
| `safety/collision_zone` | `motion/collision/event` |
| `input/gamepad`, `input/gamepad/status` | `nodes/joy`, `nodes/controller_status` — **absolute**, outside the namespace |

The attribute names are unchanged (`StateTopics.odometry`, `SafetyTopics.estop`,
`InputTopics.gamepad`), so code that went through the contract rather than
through raw key strings only needs the version bump.

**The namespace cannot be used for isolation yet.** The stack hard-codes the
`robodog/` prefix into its own key strings, so a deployment must run with
`namespace = "robodog"` exactly. `robodog/team-03` addresses nothing on the
robot. Deriving the prefix is pending on the stack side; nothing here changes
until it lands.

**`latched=True` is intent, not a guarantee.** It rides on zenoh-ext advanced
pub/sub, which needs the producer to participate, and the stack publishes with
plain Zenoh publishers. Where late-join actually matters the stack answers a
query instead — `safety/state`, `system_state/vda` and `system_state/system`
are backed by queryables. The flags stay so the day the producers upgrade
nothing has to move.

### Map identity, and the coordinates it keeps honest

New topic `localization/map_identity` carrying `MapIdentity`, in the new
`robodog_sdk.msgs.localization`. A map-frame coordinate only means something
while the map is the same one, and nothing in a pose says which map that was —
so anything persisting one (a saved spot, a landmark, a patrol route) has to
record the map id beside it and refuse coordinates from another. Without it a
rebuilt or re-sessioned map turns every stored coordinate into a confident
drive to nowhere, with nothing in the data to say so.

- `RobotClient.map_id()` returns the current id or `None`. `None` covers three
  cases a caller should treat alike — nothing published, the identity gone
  stale, or the producer saying it has no map (odometry fallback, SLAM down).
  It never means "unchanged".
- Every `MapIdentity` default is "I do not know where I am", so a payload only
  partly understood cannot read as a valid map.
- `FakeStack.set_map(...)` drives it in tests, including the `None` case.

### Navigation: dwell, and an arrival heading that is finally honoured

- **`NavigateThroughPosesGoal.dwell_sec`** — seconds to hold at each pose
  before driving on, one entry per pose with the last ignored. Validated: a
  mismatched length is silently ambiguous, so it is refused rather than guessed
  at. `RobotClient.navigate_through(dwell_sec=[...])`.
- **`orientation_at_target` / `final_orientation` are no longer decorative.**
  They were on the wire and read by no skill: a goal asking to arrive facing a
  heading finished on whatever the approach ended on and still reported
  `SUCCEEDED`. Skills now rotate onto the requested heading and withhold
  arrival until the yaw is within `arrival_orientation_deviation`. The goal
  pose's own `theta` deliberately does **not** opt in — every pose carries one
  as the approach hint, so it cannot double as "I care how I end up facing".
- **`PlannedPath.align_final_heading` and `arrival_yaw_tolerance_rad`** carry
  that opt-in to the tracker.
- **`global_nav` drives multi-pose routes now**, planning to each pose in turn,
  and is the skill that honours `dwell_sec`. It used to reject any route with
  more than one pose, which is why this package recommended `waypoint_follow`
  for routes; that advice is gone.

### The gateway status is no longer edge-only

`motion/gateway/status` re-asserts the current value on a heartbeat (~1 Hz)
alongside its edges, so a late subscriber is at most one beat behind instead of
waiting for a change that may never come. Its age is therefore meaningful:
silence means the gateway is gone. `motion/collision/event` stays edge-only.

**`GatewayAction.stop` no longer implies zero.** A breached stop zone is
directional — only the velocity component heading into the obstacle is
stripped, leaving receding motion and rotation, so the robot can back out
instead of being trapped. Only being surrounded, a stale scan or a tripped
watchdog collapses the command entirely.

### The safety path, which this package did not have

The stack grew a safety aggregator: every panel publishes its own latch on
`safety/source/{source_id}`, and the aggregator OR-combines them, runs the
recovery phase machine, and publishes the one authority on `safety/state`.

- `SafetyState` is a **level**, not an event — republished on a heartbeat as
  well as on change, so a dropped packet costs one tick rather than the truth.
  Every default is fail-safe: an unpopulated one reads as stopped with no live
  source.
- **Read `motion_permitted`, not `estop`.** They disagree for a whole phase.
  `estop` drops at the start of `RELEASING` on purpose, because the bridge's
  recovery — standing back up — is what triggers on the falling edge; holding
  it engaged would mean the robot never gets up, never reports ready, and never
  leaves `RELEASING`. Motion stays denied through that window by
  `motion_permitted` instead.
- `EstopPhase.SOURCE_LOST` is deliberately not a pressed button. It stops just
  as hard, needs no release, and clears itself when frames resume — telling an
  operator the emergency stop is pressed over a dropped heartbeat sends them
  hunting a switch nobody touched.
- `RobotClient.motion_permitted(within=...)` **fails safe on silence**: a latch
  that stopped arriving reads exactly like one that says stopped. It takes a
  freshness window rather than being a property, because the answer depends on
  when the last frame arrived and not only on what it said.
- **`RobotClient.emergency_stop()` changed key and payload.** It published
  `EmergencyStopCommand` on `command/motion/estop`, which the safety node owns
  and immediately overwrites from the composite — and which nav stopped reading.
  It now publishes the `safety/cancel` button event that the safety node, the
  nav coordinator and the fleet bridge each act on themselves: zero command,
  task cancelled, order runtime wiped. It does not latch, and cannot: only the
  physical switch latches.

### Navigation follows the stack's task contract

- **`TaskState.PENDING` is gone.** A submit starts the skill immediately, so a
  task is always running or finished. A status query for a task the coordinator
  does not remember is answered on the Zenoh **error channel** and now raises
  `ServiceError` — `task_status()` used to document a `PENDING` reply that no
  producer has ever sent. "Unknown" is not a lifecycle state.
- **`task_status()` can answer `RUNNING`.** Unlike the result key it is not
  necessarily terminal; check `state.is_terminal`.
- **`TaskFeedback.state` is narrowed to `Literal[TaskState.RUNNING]`**, so a
  feedback frame and the `TaskResult` cannot disagree — a stray terminal value
  is rejected at validation rather than believed.
- **`NavActivity` replaces a string convention.** The skill's sub-state used to
  be smuggled as a colon suffix on `active_skill` (`"waypoint_follow:stalled
  3.1s"`), which this package documented and told callers to parse. It is now a
  typed field, with the prose in `note` beside it. `active_skill` is a plain
  name again.
- `TaskFeedback` gained `current_segment_index` / `total_segments` — which
  waypoint of a route has been reached, reported as it is passed rather than at
  the end of the route.
- **`EstopPolicy`, as `?on_estop=` on the submit.** The default, `CANCEL`,
  discards the task on a stop; nothing here could previously ask for anything
  else, so every task submitted through this package was silently discarded.
  Pass `HOLD` only if the caller owns the mission and handles the recovery.
- `NavigateThroughPosesGoal.corridor_deviation_m` — how far off the route a
  human may hand the robot back before the task ends `BLOCKED` instead of
  resuming.
- `PathWaypoint.skill` and `PlannedPath.skill`, and **`PlannedPath` now has a
  key**: `nav/path`, republished on every replan. This package said no key
  carried it.
- `corridor_assist` joins the documented skills — plans with A*, hands the
  wheel to a reactive controller through the tight bits.

### Waiting for the stack never worked

`wait_until_ready()` waits for zenode presence tokens, at
`<ns>/node/<name>`. **No node of the control stack holds one** — nav, the
motion gateway, the safety aggregator and the robot bridge are plain Zenoh
applications. `wait_until_ready("nav")` therefore raised `TimeoutError` with
nav running perfectly well, and `examples/navigate.py` opened by calling
exactly that.

- `RobotClient.wait_for_nav()` replaces it for navigation. The coordinator
  holds no token, so this asks it the status of a task that cannot exist: a
  running one refuses on the error channel, and only a running one can produce
  that refusal. `ServiceTimeout` **subclasses** `ServiceError`, so the two
  clauses are ordered — the other way round reads silence as an answer and
  reports an absent coordinator as a live one.
- `wait_until_ready()` keeps its behaviour and says plainly what it is for:
  peers built on zenode, not the stack. For the rest of the stack the honest
  signal is the data — a safety latch arriving means the safety node is up.

### Added

- `robodog_sdk.msgs.safety` — `SafetyState`, `EstopPhase`, `ButtonEvent`.
- `robodog_sdk.msgs.system_state` — `SystemState` and its facets (`Posture`,
  `ControlMode`, `OrderActivity`, `Headline`, `Location`, `VdaFacet`). The
  facets are orthogonal on purpose: a robot can be `control=AUTO`, `order=IDLE`
  and `posture=LYING` at the same moment, which a flat enum cannot say without
  lying about one of them. `headline` and `ready_to_move` are derived for
  consumers that have one line to spend.
- `StateTopics.system`, `.vda` and `.releasebutton`; `SafetyTopics.state`,
  `.source`, `.release` and `.cancel`; `NavTopics.path`.
- `safety_source_key()` / `safety_source_topic()`, for one panel's own latch.
- `StateView.safety` and `StateView.system`.
- `FakeStack.set_safety()` and `.set_system()`, and `FakeStack.cancels` — the
  record of what `emergency_stop()` asked for. `FakeNav.activity` makes a skill
  appear to stall without an obstacle, and `FakeNav` now serves the status
  query, including the raise for a task it never ran.

### Fixed

- **`StateView.localization` was never populated.** The field was documented
  and typed, and nothing subscribed to `localization/pose`. Navigation goals are
  expressed in that frame, so a caller reading `state.odometry` instead was
  reasoning in the drifting one. Which of the two to measure against is a real
  choice, now that both arrive: `examples/client_drive.py` says why it keeps
  odometry for a two-metre drive.
- **`RobotClient.halt()` claimed more than it does.** It stops this client's
  contribution — a higher-ranking source keeps driving and a navigation task
  keeps running — where the docstring pointed at it as a lesser e-stop.

### Arbitration is the motion gateway, not an arbiter

The lane-and-handshake design of ADR-010 was never built. What the stack built
instead is the **motion gateway**, and this package now describes that. The two
models answer the same question and disagree on everything else: lanes ranked a
*key*, granted a lease over it, and expected a holder to release; the gateway
ranks the `MovementSource` carried on every individual command and re-decides on
every frame.

Nothing is acquired and nothing is released, which removes a whole class of
failure — a crashed holder, an expired TTL, a lease outliving the process that
took it. A source that stops publishing simply stops winning.

```python
async with robot.control(), robot.driving(x=0.3):   # before
async with robot.driving(x=0.3):                    # now
```

| removed | replacement |
|---|---|
| `command/motion/move/{teleop,nav,agent}` | one inlet, `MotionTopics.request` (`motion/gateway/in`) |
| `Lane`, `ControlRequest`/`Grant`/`Release` | `MovementSource`, carried on every command |
| `ControlServices.acquire` | nothing — there is no handshake |
| `ControlTopics.release` | nothing — silence is the release |
| `ArbiterStatus` | `MotionGatewayStatus` on `ControlTopics.status` |
| `RobotClient.control()` | `robot.preempted_by`, `robot.driving_now` — observation, not negotiation |
| `robodog_sdk.msgs.control` | merged into `robodog_sdk.msgs.motion` |
| `testing.FakeArbiter` | `stack.set_driver(...)` on `FakeStack` |

Consequences worth knowing before the bump:

- **`MovementCommand.source` now decides who drives.** It was documented as
  "provenance only", and it is now the whole of the arbitration. Naming a
  source you are not is how you take the robot from whoever that source is.
- **The default source changed, `controller` → `autonomous`.** The old default
  was the *highest* rank, so `MovementCommand(x=0.3)` from any node would have
  out-ranked the gamepad the moment source began to matter. The default is now
  the bottom rank, and `MovementSource.priority` / `.outranks()` make the order
  explicit — higher wins, which is the reverse of `Lane.priority`.
- **`ControlTopics.status` keeps its name and changes its payload.** It answers
  the same question it always did — who is driving — with the answer the
  gateway actually produces, including what the collision monitor did to the command
  (`action`, `active_zones`) and whether the winner went silent
  (`watchdog_tripped`).
- **`MotionTopics.move` is unchanged and still an output.** It is the gateway's
  result and the robot bridge's only input. Publishing there bypasses both
  arbitration and the collision zones.
- **`RobotClient.wait_until_ready()` lost its default.** It defaulted to
  `"arbiter"`, a node that does not exist; it now requires a name.

### Navigation is now a task contract

The stack replaced request/status navigation with an action contract, and this
package follows it. The change is not a rename: a navigation goal now has an
identity, a lifetime and an outcome, where before it had a correlation id and a
state that streamed on one shared key.

**What a caller does now.** Submit a goal to the `nav/task/submit` service, get
a `TaskHandle` back, and follow the task by its id — `TaskFeedback` while it
runs, one `TaskResult` when it ends.

```python
result = await robot.navigate_to(2.0, 0.5, timeout=60.0)  # was: NavigationState
if result.state is TaskState.BLOCKED:
    ...
```

| removed | replacement |
|---|---|
| `NavTopics.request` (`NavigationRequest`) | `NavServices.submit` (`NavigateToPoseGoal` / `NavigateThroughPosesGoal`) |
| `NavTopics.cancel` (`NavigationCancel`) | `NavServices.cancel` (`CancelRequest` → `CancelAck`) |
| `NavTopics.status` (`NavigationStatus`, latched) | `NavTopics.feedback` + `NavTopics.result`, per task |
| `NavigationState` | `TaskState` |
| `NavigationSegment`, `Corridor` | goal fields on the two goal types |
| `NavTopics.planned_path` | `NavTopics.path` (`nav/path`) |
| `RobotClient.cancel_navigation()` | `RobotClient.cancel_task(task_id)` |

Consequences worth knowing before the bump:

- **Terminal states went from three to four.** `ARRIVED_FINAL` → `SUCCEEDED`;
  `BLOCKED` and `FAILED` kept their meaning; `CANCELED` is new and is what a
  cancel or a preemption produces, where before a cancel produced nothing at
  all. `ARRIVED_SEGMENT` has no successor — the contract no longer reports
  arrival at an intermediate waypoint.
- **One task at a time, and no queue.** A submit while a task runs is refused
  unless it asks to preempt. `RobotClient.navigate_to(..., preempt=True)`.
- **The task keys are not latched and a result is published once.** A
  subscription declared after a task ended receives nothing;
  `RobotClient` therefore subscribes to the wildcard keys for its whole life,
  and `task_status_service(task_id)` exists for a process that has to ask
  afterwards.
- **A navigation goal is no longer a trace root.** `nav/request` was declared
  `trace=True`, which started one trace per request. A `Service` cannot be a
  trace root, so a task submitted outside a handler starts no trace.

### Added

- The contract on `zenode.Topic`/`Service`: `MotionTopics`, `PoseTopics`,
  `StateTopics`, `LocalizationTopics`, `NavTopics`, `NavServices`,
  `ControlTopics`, `ControlServices`.
- `robodog_sdk.msgs.occupancy` — `CostMap` and its cost bands, on
  `nav/costmap/global` (latched, already inflated by the robot radius) and
  `nav/costmap/local`. Pure Python: the grid arrives as a flat list of ints and
  reshaping it is the caller's line of numpy, so the package keeps its two
  dependencies.
- `RobotClient.submit()` / `wait_for_task()` / `cancel_task()` /
  `task_status()` / `navigating`, and `navigate_through()` for a route.
- `robodog_sdk.testing.FakeNav` — accepts goals, streams feedback and ends a
  task wherever the test says, so the non-arrival branches are testable
  without an obstacle.
- `examples/navigate.py` — the task contract end to end.
- `MovementSource.assisted_teleop`, `.priority` and `.outranks()`, plus
  `GatewayAction` and `MotionGatewayStatus` — the gateway's vocabulary.
- Payload schemas ported from `robodog-digipro:src/interfaces` — motion, robot
  state, navigation.
- `robodog_sdk.limits` — the robot's capability envelope, enforced as Pydantic
  field constraints on `MovementCommand` and `TiltBody`.
- Trace roots on the topics that begin a causal chain:
  `system_state/odometry` and `localization/pose`, both sampled at
  `TRACE_RATIO`. The movement keys are
  deliberately not roots, and a service call joins the caller's trace rather
  than starting one.
- `RobotClient` — facade over the contract.
- `robodog_sdk.testing.FakeStack` — the other side of the conversation, for
  tests and offline development.

### Changed from `src/interfaces` — wire-visible

The stack's keys are taken as they are. An earlier draft of this package
restyled them — `system_state/*` to `state/*`, `nodes/joy` to `input/gamepad`,
the safety path under one `safety/` prefix — on the expectation that the stack
would follow; it did not, and those keys addressed nothing. See the realignment
at the top of this entry. What remains below is the difference between the
schemas here and the ones in `src/interfaces`, which is real.

- **Keys are relative.** `[transport] namespace` is applied at runtime — but
  the stack hard-codes `robodog/`, so today that namespace must be exactly
  `robodog`.
- **`nodes/joy` and `nodes/controller_status` are absolute.** They sit at the
  root of the keyspace, outside the namespace, because that is where the
  teleoperation node publishes them.
- **`CollisionZoneEvent` replaces `ProtectiveFieldEvent`.** The stack's payload
  names the zone that fired; this package's had no zone and assumed there was
  only one.
- **`system_state/agv_state` → `system_state/vda`**, carrying a typed
  `VdaFacet` where the old key carried hand-rolled JSON.
- **`nav/simple/*` is not in the contract.** The stack's `nav-remote` node
  offers a flattened JSON pub/sub facade for clients that cannot depend on the
  schemas. A client that has this package has the schemas, so it talks to the
  coordinator directly and does not need a second node in the path.
- **`command/motion/move` is the gateway's output** and the robot bridge's only
  input; every producer publishes to `motion/gateway/in` instead. This is the
  one place the package keeps the stack's own spelling rather than restyling
  it: the gateway keys exist and work today, and a prettier name for a
  deployed key buys nothing.
- **Movement commands expire** (`max_age = 0.3 s`), replacing the bridge's
  `movement-max-delay-ms` and serving as the deadman.
- **State topics are declared latched**, in place of the hand-rolled
  `session.get()` startup pulls in the stack. The stack's publishers do not
  participate in it yet — see the note at the top of this entry.
- **Velocities and tilt are bounded.** Values the old schemas accepted now
  raise `ValidationError`. See the warning in `limits.py`: the numbers are
  conservative placeholders until someone measures the robot.
- **`MovementCommand.scale()` validates.** It used `model_copy(update=…)`,
  which skips validation in Pydantic v2 — a scaled command could leave the
  envelope silently, which is exactly what `joy`'s speed factor does.
- **MOLA's ROS 2 output is not in the contract.** `lidar_odometry/pose` is
  CDR-encoded `PoseStamped` bridged from the MOLA container, not an
  `OdometryState`. The fused pose has one key, `localization/pose`, with one
  producer at a time.
- **`ConnectionStatus` removed.** Nothing publishes it; a contract should not
  ship a type that describes nothing on the wire.

### Not yet ported

- `camera.py` / `image.py` — JPEG frame envelopes, and the sensor topics that
  carry them (`RawCodec`, `shm=True`).
- `livox.py` — CDR point-cloud decode; lands in `robodog_sdk.contrib` behind
  the `[livox]` extra rather than in the core.
- Diagnostic topics (`robodog/diagnostic/*`) — deliberately not ported. The
  diagnostic node re-publishes fields of highstate and battery one scalar per
  key inside a `{name, category, data}` envelope; it is a view of
  `StateTopics`, not a source.
- Liveliness keys (`robodog/liveliness/*`) — liveliness tokens held by the Go2
  bridge, not pub/sub topics.
