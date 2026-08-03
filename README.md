# robodog-sdk

**The wire contract for the Robodog control system — and the way to build on it
without forking it.**

The Robodog stack is a set of independent processes talking over
[Eclipse Zenoh](https://zenoh.io): a bridge to the Unitree Go2 (or a MuJoCo
simulation of it), navigation, localization, teleoperation, safety. They are
coupled only through typed messages on well-known keys.

This package *is* that coupling. Install it, and your process is a peer of
every node in the stack — in your own repository, your own virtualenv, your own
process. You never clone the stack.

Built on [zenode](https://github.com/hse-deb-algo-athlets/zenode). Rationale
and the packaging decision: ADR-010 in the robodog-digipro repository.

## Install

```bash
uv add "robodog-sdk @ git+https://github.com/hse-deb-algo-athlets/robodog-sdk@v0.1.0"
```

Two dependencies (`zenode`, `pydantic`), no hardware or simulation packages.
It installs on any laptop in seconds.

## A first node

```python
from zenode import Node, run, subscribe
from robodog_sdk import OdometryState, RobotClient, StateTopics


class Wanderer(Node):
    name = "wanderer"

    async def on_start(self) -> None:
        self.robot = RobotClient(self)

    @subscribe(StateTopics.odometry, mode="latest")
    async def on_pose(self, msg: OdometryState) -> None:
        if msg.x > 2.0:
            self.robot.halt()


def cli() -> None:
    run(Wanderer)
```

```toml
# zenode.toml
[transport]
mode = "client"
connect = ["tcp/localhost:7447"]
namespace = "robodog"          # your team's sandbox, e.g. robodog/team-03
```

Bring up the stack (simulation, no robot needed), then run your node:

```bash
uv run wanderer
```

## The two ways in

**The contract** is the supported interface: `robodog_sdk.topics` binds every
key to its payload type, and `robodog_sdk.msgs` holds the schemas.

```python
from zenode import publish
from robodog_sdk import MotionTopics, MovementCommand

cmd = publish(MotionTopics.move_agent)
cmd.put(MovementCommand(x=0.3))
```

**`RobotClient`** is a facade over exactly that — same keys, same priority, no
privileged access. It exists so a first node is short:

```python
async with robot.control(), robot.driving(x=0.3):
    await asyncio.sleep(2)  # stops on the way out, always
```

Use whichever reads better. Dropping to the contract is not leaving the
supported path.

[`examples/`](examples/README.md) has the same node written both ways —
[`contract_drive.py`](examples/contract_drive.py) and
[`client_drive.py`](examples/client_drive.py). Diff them to see exactly what the
client does and does not do for you.

See the whole contract without reading the source:

```bash
uv run zenode topics --contract robodog_sdk.topics
```

## Things that will bite you once

**Commands expire.** Every movement lane carries `max_age = 0.3 s`, which is
the deadman: stop publishing and the robot stops. One `robot.move(...)` drives
for 300 ms, not until you say otherwise. Use `robot.driving(...)`, or publish
at 10 Hz yourself.

**Velocities are bounded.** `MovementCommand` enforces the robot's capability
envelope (`robodog_sdk.limits`), so an out-of-range value raises
`ValidationError` in *your* process rather than surprising anyone in the lab.
If a limit is wrong, fix it in `limits.py` — not by working around the model.

**You are not the only driver.** Commands go on the `agent` lane, which the
arbiter ranks below teleoperation and the e-stop. Take the lane explicitly with
`async with robot.control():` so it is visible who is driving.

**Nothing may release an e-stop.** `robot.emergency_stop()` exists;
there is no counterpart. Clearing a stop happens at the physical button.

**The namespace must match the deployment.** Keys in the contract are relative;
`[transport] namespace` is prefixed at runtime, so `state/odometry` becomes
`robodog/state/odometry`. Everything talking to one robot uses that robot's
namespace — set it to anything else and you will see no data at all, with no
error. It is the first thing to check when nothing arrives, and
`uv run zenode nodes` shows whether you are looking in the right place.

A namespace separates *deployments*, not users. Your own simulation can run
under `robodog/team-03` in complete isolation; against the shared robot you use
its namespace like everyone else, and the arbiter — not the namespace — decides
who drives.

## Tracing

Your node is traced whether or not you ask for it, and that is the point: when
a command goes out and nothing moves, the question spans four processes.

A trace starts at the topics that begin a causal chain — `state/odometry`,
`localization/pose` (both sampled at 1 %) and `nav/request` (every one) — and
follows the data from there. **Everything your handler causes stays in that
trace automatically**: `put()`, `await self.call()`, `self.spawn()` and
`await self.blocking()`. There is nothing to configure and no API to learn.

Follow one message across the fleet:

```bash
uv run zenode logs --trace <id>    # every log record from that chain
uv run zenode trace <id>           # the path it took, hop by hop
```

Both work with nothing installed and no collector running. Spans are optional
(`zenode[otel]`); without them a traced `put()` costs about 3.6 µs against
1.9 µs untraced, and untraced topics are unaffected either way.

### The one thing to watch: timers break the chain

A timer body is caused by the clock, not by a message, so it runs outside any
trace. The common sense-then-act shape therefore loses the link:

```python
@subscribe(StateTopics.odometry, mode="latest")
async def on_pose(self, msg):
    self.latest = msg


@every(0.1)
async def tick(self):
    self.cmd.put(...)  # orphaned — no link to the pose that caused it
```

Capture the context in the handler and restore it before publishing:

```python
from zenode import trace


@subscribe(StateTopics.odometry, mode="latest")
async def on_pose(self, msg):
    self.latest = (msg, trace.current())


@every(0.1)
async def tick(self):
    msg, traceparent = self.latest
    with trace.using(traceparent):
        self.cmd.put(...)
```

[`examples/contract_drive.py`](examples/contract_drive.py) does exactly this.

The same applies to `RobotClient.driving()`: its republish pump is clock-driven,
so those commands are not linked to a measurement either. Use `robot.move()`
inside a handler when you want the causal chain preserved.

### Declaring your own topics

`trace=` is yours to set on topics you declare. Mark the topic that *starts* a
chain, and sample it if it is a stream:

```python
class MyTopics(TopicSet):
    detections = Topic("perception/detections", Detection, trace=True, trace_ratio=0.05)
```

Marking a downstream topic as well is harmless — a topic starts a trace only
when none is active, so a pipeline stays one trace. It does mean `trace_ratio`
only takes effect on whichever topic actually started it.

## Testing without a robot

`robodog_sdk.testing.FakeStack` plays the other side of the conversation:
latched state, an arbiter that grants lanes, and a record of everything your
node tried to drive. With `zenode.testing`, the whole thing runs in-process —
no router, no network.

```python
async with harness() as h:
    stack = await h.start_node(FakeStack)
    agent = await h.start_node(MyAgent)

    stack.set_battery(soc=5, level=BatteryLevel.critical)
    await asyncio.sleep(0.2)

    assert stack.stopped
```

It is a stand-in, not a simulator — nothing moves. For "does the robot actually
get there", run the MuJoCo simulation.

## Versioning

Semantic versioning, `0.x` while the contract settles: minor versions may move
keys. Pin a tag. `CONTRACT_VERSION` is reported on each node's health
heartbeat, so a skew between your project and the deployed stack shows up in
`zenode health` instead of as a parse error somewhere else.

## Development

```bash
uv sync
uv run pytest
uv run ruff check --fix && uv run ruff format
uv run pyright
```
