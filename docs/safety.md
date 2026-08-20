# Safety

## Software can stop the robot; it cannot pretend to be the button

`robot.emergency_stop()` publishes the cancel event that the safety node, the
navigation coordinator and the fleet bridge each act on themselves — the robot
is zeroed, the running task is cancelled, the order runtime is wiped. It does
not *latch*, and there is no counterpart to it, because only the physical
switch latches and only the release press on the panel clears one.

## Ask `motion_permitted()`, not `state.safety.value.estop`

The latch drops one phase before the robot can actually move — deliberately,
so the bridge can start standing it back up — and
{meth}`~robodog_sdk.client.RobotClient.motion_permitted` is the field that
closes that gap. It also fails safe on silence: a safety latch that stopped
arriving reads exactly like one that says stopped, which is why it takes a
freshness window rather than being a property.

## Collision zones shape commands, they do not own them

A breached stop zone strips only the velocity heading into the obstacle (see
[Driving](driving.md#when-commands-go-out-and-nothing-moves)); the robot can
still reverse or turn out of it. `robot.blocked_by_zone` lists the zones
currently shaping your commands, and `robot.state.gateway` records what the
gateway actually did.
