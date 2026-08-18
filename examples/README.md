# Examples

Two nodes, one behaviour: drive forward until 2 m are covered or a collision
zone fires, then stop.

| | |
|---|---|
| [`contract_drive.py`](contract_drive.py) | Written against `Topic` objects — `publish`, `@subscribe`, `@every` |
| [`client_drive.py`](client_drive.py) | The same thing through `RobotClient` |

They are deliberately the same task. Diff them: the client saves you the
republish timer, the stop-on-exit, and the bookkeeping around the latched pose —
and costs you nothing you cannot reach past. Neither file is the "right" way;
pick whichever reads better for what you are building.

A third node covers the other half of the contract:

| | |
|---|---|
| [`navigate.py`](navigate.py) | Navigation as a task — submit, watch, and handle every way it can end |

Driving is a velocity you keep sending. Navigating is a goal you hand over and
an outcome you wait for, and the outcome is not always arrival — `navigate.py`
is mostly about that difference.

## Running them

Bring up a stack first — simulation is enough, the examples never touch
hardware. Transport and namespace come from [`zenode.toml`](../zenode.toml) in
the repository root; override either with `ZENODE_CONFIG` or
`ZENODE_TRANSPORT__NAMESPACE`.

> Both drive nodes need the **motion gateway**, which is the only writer on
> `command/motion/move`. Without it their commands reach the inlet and stop
> there — the robot will not move and nothing will say why, which is exactly
> what `robot.state.gateway` staying empty means.
>
> `navigate.py` needs the **nav** node, a localization source and a global map
> — everything the navigation stack normally brings up. Without them it stops
> with `no navigation coordinator answered`.
>
> It asks nav a question to find that out rather than waiting for a presence
> token, because no node of the stack holds one: they are plain Zenoh
> applications, invisible to `wait_until_ready()`. That method is for waiting
> on peers built on this package.

```bash
uv run python examples/contract_drive.py
uv run python examples/client_drive.py
uv run python examples/navigate.py
```

Both stop the robot when you Ctrl-C them.

Watch what they publish, from another terminal:

```bash
uv run zenode echo "motion/gateway/in" --contract robodog_sdk.topics
```

## Running them without a stack

`tests/test_examples.py` starts all three against the doubles in
[`robodog_sdk.testing`](../src/robodog_sdk/testing.py) in one process — no
router, no robot, no simulation:

```bash
uv run pytest tests/test_examples.py -q
```

That file is also the shortest answer to "how do I test my own node".

## Configuration

Each reads its own section — `[node.contract-drive]`, `[node.client-drive]`,
`[node.navigate]` — and every field has a default, so they run with no config
at all:

```toml
[node.client-drive]
speed = 0.2
distance = 5.0
```

Environment overrides work the same way: `ZENODE_CLIENT_DRIVE__SPEED=0.2`.
