"""The robot's capability envelope.

Properties of the machine rather than tunables of any one node, so they are
part of the contract. :mod:`robodog_sdk.msgs.motion` applies them as Pydantic
field constraints, making an out-of-range command invalid at construction.

.. warning::

   The linear and yaw values below are conservative placeholders and need
   confirming against the robot. They are not derived from an agreed limit,
   because the stack does not currently have one:

   - ``joy`` scales stick input by a speed factor of up to 5.0, permitting
     roughly 5 m/s and 300 deg/s.
   - ``nav`` uses ``default_speed = 1.0`` m/s and caps rotation at
     ``max_angular_velocity = 0.8`` rad/s (~46 deg/s).
   - ``[node-joy] max-tilt-angle`` and ``max-yaw-angle`` are 20°, the source
     of the tilt values here and the only measured figures.

   Adjust the values in this module rather than working around them in a node.
"""

from __future__ import annotations

#: m/s — forward/backward. Placeholder, see the module warning.
MAX_LINEAR_MS = 1.5

#: m/s — lateral (strafe). Placeholder, see the module warning.
MAX_LATERAL_MS = 1.0

#: deg/s — yaw rate. Placeholder, see the module warning.
MAX_YAW_RATE_DEG = 120.0

#: deg — body pitch/roll, from ``[node-joy] max-tilt-angle``.
MAX_TILT_DEG = 20.0

#: deg — body yaw while standing, from ``[node-joy] max-yaw-angle``.
MAX_BODY_YAW_DEG = 20.0

#: m — commanded body height range of the Go2 standing posture.
MIN_BODY_HEIGHT_M = 0.15
MAX_BODY_HEIGHT_M = 0.35


def clamp(value: float, low: float, high: float) -> float:
    """Clamp ``value`` into ``[low, high]``."""
    return max(low, min(high, value))


def clamp_velocity(x: float, y: float, z_deg: float) -> tuple[float, float, float]:
    """Clamp a velocity triple into the envelope.

    For inputs that should saturate rather than raise, such as a joystick
    mapping or a controller that overshoots slightly. Where a value is the
    result of a decision rather than a signal, prefer letting the model
    validate: clamping a planner request to the limit conceals the fault.
    """
    return (
        clamp(x, -MAX_LINEAR_MS, MAX_LINEAR_MS),
        clamp(y, -MAX_LATERAL_MS, MAX_LATERAL_MS),
        clamp(z_deg, -MAX_YAW_RATE_DEG, MAX_YAW_RATE_DEG),
    )
