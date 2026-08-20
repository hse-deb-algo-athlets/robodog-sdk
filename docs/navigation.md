# Navigation

## A task, not a command

`await robot.navigate_to(2.0, 0.5)` submits a goal, gets an id back, and
returns the {class}`~robodog_sdk.msgs.navigation.TaskResult` that id ends on —
which can be `SUCCEEDED`, `BLOCKED`, `FAILED` or `CANCELED`. Only the first is
arrival, and `BLOCKED` is not an error: the robot met the world and stopped.
Check the result, do not assume it.

[`examples/navigate.py`](https://github.com/hse-deb-algo-athlets/robodog-sdk/blob/main/examples/navigate.py)
covers the whole task contract: submit, watch, and handle each of the four
ways a task can end.

## Lifecycle and activity are different questions

`TaskFeedback.state` is `RUNNING` for the whole task and never anything else —
the terminal verdict lives only on the result key, so the two can never
disagree. What moves is `activity`: `cruising`, `aligning`, `stalled`,
`retreating`. A stall is transient and the skill is still trying; only a
`TaskResult` carrying `BLOCKED` means it gave up.

## One task at a time

There is no queue. A goal submitted while the robot is navigating is *refused*
unless you pass `preempt=True`, which cancels the running task first. Refusal
raises `PermissionError` from `navigate_to`; if you would rather branch than
catch, use `robot.submit()` and read `handle.accepted`.

## A task you are not watching is discarded on an e-stop

That is the default, and it is the right one for a goal sent from a script:
nobody wants a route resuming itself minutes after a human walked over and hit
the button. Pass `on_estop=EstopPolicy.HOLD` only if this process owns the
mission and is still there to handle the recovery.

## Asking after a forgotten task raises

`task_status()` answers with a real state for a task the coordinator
remembers — including `RUNNING` for one still under way, so check
`state.is_terminal` — and raises `ServiceError` for one it has never heard of
or has since evicted. "Unknown" is not a lifecycle state and the contract will
not invent one.

## A map-frame coordinate is only valid while the map is

If you store a pose and drive to it later, store `robot.map_id()` beside it
and refuse the coordinate when the ids differ — nothing in a bare pose says
which map it came from, so a rebuilt or re-sessioned map turns every saved
coordinate into a confident drive to the wrong place. `map_id()` returns
`None` for "no usable map" (SLAM down, odometry fallback, or nothing
published), which never means "unchanged".
