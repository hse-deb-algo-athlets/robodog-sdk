# Changelog

Consumers pin a tag, so this file is how they learn what a bump costs them.
Semantic versioning; `0.x` minor versions may move keys.

## [Unreleased]

### Added

- The contract on `zenode.Topic`/`Service`: `MotionTopics`, `PoseTopics`,
  `StateTopics`, `LocalizationTopics`, `NavTopics`, `ControlTopics`,
  `ControlServices`.
- Payload schemas ported from `robodog-digipro:src/interfaces` — motion, robot
  state, navigation.
- `robodog_sdk.limits` — the robot's capability envelope, enforced as Pydantic
  field constraints on `MovementCommand` and `TiltBody`.
- `robodog_sdk.msgs.control` — command-lane arbitration (`Lane`,
  `ControlRequest`/`Grant`/`Release`, `ArbiterStatus`). No counterpart in
  `src/interfaces`; new with ADR-010.
- `NavigationCancel` and `nav/cancel`, so cancelling is not an invalid
  `NavigationRequest`.
- Trace roots on the topics that begin a causal chain: `state/odometry` and
  `localization/pose` sampled at `TRACE_RATIO`, `nav/request` unsampled.
  Command lanes are deliberately not roots.
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
- **`command/motion/estop` → `safety/estop`**, and `nav/protective_field` →
  `safety/protective_field`, so the whole safety path sits under one prefix.
- **`sensors/realsense/*` → `sensors/d435i/*`**, and `sensors/go2_camera` →
  `sensors/go2/camera`: device first, then stream, so a second camera does not
  require renaming the first.
- **`system_state/agv_state` → `vda5050/state`**, owned by the MQTT bridge.
- **Movement lanes added.** `command/motion/move` becomes the arbiter's output
  and the robot bridge's only input; producers publish to
  `command/motion/move/{teleop,nav,agent}`.
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
