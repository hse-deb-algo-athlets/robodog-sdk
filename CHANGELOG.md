# Changelog

Consumers pin a tag, so this file is how they learn what a bump costs them.
Semantic versioning; `0.x` minor versions may move keys.

## [Unreleased]

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
| `ArbiterStatus` on `control/status` | `MotionGatewayStatus` on the same key |
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
- **`control/status` keeps its key and changes its payload.** It answers the
  same question it always did — who is driving — with the answer the gateway
  actually produces, including what the collision monitor did to the command
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
| `NavTopics.planned_path` | `PlannedPath` stays as a type; no key carries it |
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
- Trace roots on the topics that begin a causal chain: `state/odometry` and
  `localization/pose`, both sampled at `TRACE_RATIO`. The movement keys are
  deliberately not roots, and a service call joins the caller's trace rather
  than starting one.
- `RobotClient` — facade over the contract.
- `robodog_sdk.testing.FakeStack` — the other side of the conversation, for
  tests and offline development.

### Changed from `src/interfaces` — wire-visible

The key scheme was restructured in the stack first (ADR-010 step 1); this
package mirrors it. The `robodog/` prefix had come to mean nothing — introduced
for "data from the robot", it ended up on commands, navigation and diagnostics
too — so it becomes the deployment namespace, and the first segment of a key
now names the *kind* of thing it carries.

- **Keys are relative.** `[transport] namespace` is applied at runtime.
- **`system_state/*` → `state/*`.**
- **`nav/*` and `nodes/*` come under the namespace.** Outside it they cannot be
  isolated per deployment: two simulations on one network would share them.
- **`nodes/joy` → `input/gamepad`**, `nodes/controller_status` →
  `input/gamepad/status`. `node/` is reserved for zenode's presence, health,
  log and trace keys.
- **`command/motion/estop` → `safety/estop`**, and the motion gateway's
  `motion/collision/event` → `safety/collision_zone`, so the whole safety path
  sits under one prefix. The payload is the stack's `CollisionZoneEvent`, which
  names the zone that fired; it replaces this package's `ProtectiveFieldEvent`,
  which had no zone and assumed there was only one.
- **`sensors/realsense/*` → `sensors/d435i/*`**, and `sensors/go2_camera` →
  `sensors/go2/camera`: device first, then stream, so a second camera does not
  require renaming the first.
- **`system_state/agv_state` → `vda5050/state`**, owned by the MQTT bridge.
- **`nav/task/*` keys and `nav/costmap/*` come under the namespace**, like the
  rest of `nav/*`.
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
- **State topics are latched**, replacing three hand-rolled `session.get()`
  startup pulls in the stack.
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
- `controller.py` — gamepad state (`nodes/joy`, `nodes/controller_status`).
- `livox.py` — CDR point-cloud decode; lands in `robodog_sdk.contrib` behind
  the `[livox]` extra rather than in the core.
- Diagnostic topics (`robodog/diagnostic/*`) — pending the decision on whether
  zenode's health heartbeat subsumes the diagnostic node.
