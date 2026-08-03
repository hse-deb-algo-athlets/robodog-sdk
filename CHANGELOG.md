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

These are breaking against the stack as it stands today. Each is deliberate;
none can be deferred past the first release, because a key that moves later
breaks a cohort mid-semester rather than nobody.

- **Keys are relative.** The `robodog/` prefix comes from
  `[transport] namespace` at runtime instead of being baked into every string.
- **Namespace inconsistencies normalized.** `nav/*`, `nodes/*` and
  `lidar_odometry/pose` escaped the prefix; they are now namespaced like
  everything else. Exception: `livox/lidar` and `lidar_odometry/pose` stay
  absolute where an external producer owns the key.
- **`system_state/*` → `state/*`.**
- **Movement lanes added.** `command/motion/move` is now the arbiter's *output*
  and the robot bridge's only input. Producers publish to
  `command/motion/move/{teleop,nav,agent}`.
- **Movement commands expire** (`max_age = 0.3 s`) — this replaces the bridge's
  `movement-max-delay-ms` and is the deadman.
- **State topics are latched**, replacing three hand-rolled `session.get()`
  startup pulls in the stack.
- **Velocities and tilt are bounded.** Values the old schemas accepted now
  raise `ValidationError`. See the warning in `limits.py`: the numbers are
  conservative placeholders until someone measures the robot.
- **`MovementCommand.scale()` validates.** It used `model_copy(update=…)`,
  which skips validation in Pydantic v2 — a scaled command could leave the
  envelope silently, which is exactly what `joy`'s speed factor does.

### Not yet ported

- `camera.py` / `image.py` — JPEG frame envelopes, and the sensor topics that
  carry them (`RawCodec`, `shm=True`).
- `controller.py` — gamepad state (`nodes/joy`, `nodes/controller_status`).
- `livox.py` — CDR point-cloud decode; lands in `robodog_sdk.contrib` behind
  the `[livox]` extra rather than in the core.
- Diagnostic topics (`robodog/diagnostic/*`) — pending the decision on whether
  zenode's health heartbeat subsumes the diagnostic node.
