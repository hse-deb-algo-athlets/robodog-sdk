# robodog-sdk

```{toctree}
:hidden:

driving
navigation
safety
tracing
testing
api/index
```

**The wire contract for the Robodog control system — and the way to build on
it without forking it.**

The Robodog stack is a set of independent processes talking over
[Eclipse Zenoh](https://zenoh.io): a bridge to the Unitree Go2 (or a MuJoCo
simulation of it), navigation, localization, teleoperation, safety. They are
coupled only through typed messages on well-known keys.

This package *is* that coupling. Install it, and your process is a peer of
every node in the stack — in your own repository, your own virtualenv, your
own process. You never clone the stack.

Built on [zenode](https://hse-deb-algo-athlets.github.io/zenode/). Rationale
and the packaging decision: ADR-010 in the robodog-digipro repository.

## Reference

| Document | Covers |
|---|---|
| [Driving](driving.md) | The contract vs. `RobotClient`, the deadman, arbitration, the gateway |
| [Navigation](navigation.md) | Tasks, the four outcomes, feedback, map identity |
| [Safety](safety.md) | The e-stop, `motion_permitted`, what software can and cannot do |
| [Tracing](tracing.md) | Following one command across four processes |
| [Testing](testing.md) | `FakeStack` and `FakeNav` — no router, no robot |
| [API reference](api/index.rst) | Generated from the package docstrings |

## Install

```bash
uv add "robodog-sdk @ git+https://github.com/hse-deb-algo-athlets/robodog-sdk@v0.2.1"
```

Two dependencies (`zenode`, `pydantic`), no hardware or simulation packages.
It installs on any laptop in seconds. The optional `livox` extra adds CDR
point-cloud decoding for the externally-produced `livox/lidar` topic:

```bash
uv add "robodog-sdk[livox] @ git+https://github.com/hse-deb-algo-athlets/robodog-sdk@v0.2.1"
```

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
namespace = "robodog"          # must be exactly this — see below
```

Bring up the stack (simulation, no robot needed), then run your node:

```bash
uv run wanderer
```

## The namespace must match the deployment

Keys in the contract are relative; `[transport] namespace` is prefixed at
runtime, so `system_state/odometry` becomes `robodog/system_state/odometry`.
Everything talking to one robot uses that robot's namespace — set it to
anything else and you will see no data at all, with no error. It is the first
thing to check when nothing arrives, and `uv run zenode nodes` shows whether
you are looking in the right place.

Today that means **`namespace = "robodog"`, and nothing else**: the stack
still hard-codes the prefix into its own key strings rather than deriving it
from a namespace, so an isolated sandbox under `robodog/team-03` would talk to
itself and to nothing on the robot. Deriving it is a change pending on the
stack side.

## Seeing the contract

The whole contract is introspectable without reading the source:

```bash
uv run zenode topics --contract robodog_sdk.topics
```

The [`examples/`](https://github.com/hse-deb-algo-athlets/robodog-sdk/tree/main/examples)
directory has the same drive node written against the raw contract and through
`RobotClient` — diff them to see exactly what the client does and does not do
for you — plus `navigate.py` for the task contract.

## Versioning

Semantic versioning, `0.x` while the contract settles: minor versions may move
keys. Pin a tag. `CONTRACT_VERSION` is reported on each node's health
heartbeat, so a skew between your project and the deployed stack shows up in
`zenode health` instead of as a parse error somewhere else.

## License

Apache License 2.0 — see
[LICENSE](https://github.com/hse-deb-algo-athlets/robodog-sdk/blob/main/LICENSE)
and [NOTICE](https://github.com/hse-deb-algo-athlets/robodog-sdk/blob/main/NOTICE).
Copyright 2026 Hochschule Esslingen.
