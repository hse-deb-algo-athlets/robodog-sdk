# Tracing

Your node is traced whether or not you ask for it, and that is the point: when
a command goes out and nothing moves, the question spans four processes.

A trace starts at the topics that begin a causal chain —
`system_state/odometry` and `localization/pose`, both sampled at 1 %
({data}`~robodog_sdk.topics.TRACE_RATIO`) — and follows the data from there.
**Everything your handler causes stays in that trace automatically**: `put()`,
`await self.call()`, `self.spawn()` and `await self.blocking()`. There is
nothing to configure and no API to learn.

A service call is not a trace root: it joins the caller's trace, or none. So a
navigation task submitted from a script has no trace of its own, while one
submitted from inside a handler belongs to the trace of whatever triggered it.

Follow one message across the fleet:

```bash
uv run zenode logs --trace <id>    # every log record from that chain
uv run zenode trace <id>           # the path it took, hop by hop
```

Both work with nothing installed and no collector running. Spans are optional
(`zenode[otel]`); without them a traced `put()` costs about 3.6 µs against
1.9 µs untraced, and untraced topics are unaffected either way.

## The one thing to watch: timers break the chain

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

[`examples/contract_drive.py`](https://github.com/hse-deb-algo-athlets/robodog-sdk/blob/main/examples/contract_drive.py)
does exactly this.

The same applies to `RobotClient.driving()`: its republish pump is
clock-driven, so those commands are not linked to a measurement either. Use
`robot.move()` inside a handler when you want the causal chain preserved.

## Declaring your own topics

`trace=` is yours to set on topics you declare. Mark the topic that *starts* a
chain, and sample it if it is a stream:

```python
class MyTopics(TopicSet):
    detections = Topic("perception/detections", Detection, trace=True, trace_ratio=0.05)
```

Marking a downstream topic as well is harmless — a topic starts a trace only
when none is active, so a pipeline stays one trace. It does mean `trace_ratio`
only takes effect on whichever topic actually started it.
