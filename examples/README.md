# Examples

Two nodes, one behaviour: drive forward until 2 m are covered or the protective
field fires, then stop.

| | |
|---|---|
| [`contract_drive.py`](contract_drive.py) | Written against `Topic` objects — `publish`, `@subscribe`, `@every` |
| [`client_drive.py`](client_drive.py) | The same thing through `RobotClient` |

They are deliberately the same task. Diff them: the client saves you the
republish timer, the stop-on-exit, and the lane handshake — and costs you
nothing you cannot reach past. Neither file is the "right" way; pick whichever
reads better for what you are building.

## Running them

Bring up a stack first (simulation is enough — the examples never touch
hardware), then point the node at the router:

```toml
# zenode.toml, next to wherever you run from
[transport]
mode = "client"
connect = ["tcp/localhost:7447"]
namespace = "robodog"
```

```bash
uv run python examples/contract_drive.py
uv run python examples/client_drive.py
```

Both stop the robot when you Ctrl-C them.

Watch what they publish, from another terminal:

```bash
uv run zenode echo "command/motion/move/agent" --contract robodog_sdk.topics
```

## Running them without a stack

`tests/test_examples.py` starts both against
[`FakeStack`](../src/robodog_sdk/testing.py) in one process — no router, no
robot, no simulation:

```bash
uv run pytest tests/test_examples.py -q
```

That file is also the shortest answer to "how do I test my own node".

## Configuration

Both read `[node.contract-drive]` / `[node.client-drive]`, and every field has a
default, so they run with no config at all:

```toml
[node.client-drive]
speed = 0.2
distance = 5.0
```

Environment overrides work the same way: `ZENODE_CLIENT_DRIVE__SPEED=0.2`.
