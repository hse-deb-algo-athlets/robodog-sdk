# Testing without a robot

{mod}`robodog_sdk.testing` plays the other side of the conversation:
{class}`~robodog_sdk.testing.FakeStack` for latched state and a record of
everything your node tried to drive, and
{class}`~robodog_sdk.testing.FakeNav` for navigation — it accepts goals,
streams feedback, and ends a task wherever you tell it to, so the `BLOCKED`
branch of your code gets exercised without needing an obstacle.
`nav.activity = NavActivity.STALLED` makes the skill appear to stall mid-task
without one either.

`stack.set_driver(...)` fakes a human grabbing the gamepad or a collision zone
firing, and `stack.set_safety(...)` fakes the e-stop — a pressed button, or
the safety source going quiet, which stops just as hard for an entirely
different reason. All of it is otherwise hard to arrange on a desk. With
`zenode.testing`, the whole thing runs in-process — no router, no network.

```python
async with harness() as h:
    stack = await h.start_node(FakeStack)
    agent = await h.start_node(MyAgent)

    stack.set_battery(soc=5, level=BatteryLevel.critical)
    await asyncio.sleep(0.2)

    assert stack.stopped
```

It is a stand-in, not a simulator — nothing moves. For "does the robot
actually get there", run the MuJoCo simulation.

[`tests/test_examples.py`](https://github.com/hse-deb-algo-athlets/robodog-sdk/blob/main/tests/test_examples.py)
starts all three example nodes against these doubles in one process; that file
is also the shortest answer to "how do I test my own node".
