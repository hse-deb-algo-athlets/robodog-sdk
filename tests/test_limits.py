"""The capability envelope rejects out-of-range values.

Without these, the constraints in :mod:`robodog_sdk.limits` could be removed or
widened without any test failing.
"""

from __future__ import annotations

from typing import Any

import pytest
from pydantic import ValidationError

from robodog_sdk import MovementCommand, TiltBody, limits


def test_defaults_are_zero() -> None:
    assert MovementCommand().is_zero()


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("x", limits.MAX_LINEAR_MS + 0.1),
        ("x", -limits.MAX_LINEAR_MS - 0.1),
        ("y", limits.MAX_LATERAL_MS + 0.1),
        ("z_deg", limits.MAX_YAW_RATE_DEG + 1.0),
    ],
)
def test_velocity_outside_envelope_is_rejected(field: str, value: float) -> None:
    kwargs: dict[str, Any] = {field: value}
    with pytest.raises(ValidationError):
        MovementCommand(**kwargs)


def test_velocity_at_the_boundary_is_accepted() -> None:
    cmd = MovementCommand(x=limits.MAX_LINEAR_MS, z_deg=-limits.MAX_YAW_RATE_DEG)
    assert cmd.x == limits.MAX_LINEAR_MS


def test_tilt_outside_envelope_is_rejected() -> None:
    with pytest.raises(ValidationError):
        TiltBody(pitch_deg=limits.MAX_TILT_DEG + 1)


def test_clamp_velocity_saturates() -> None:
    x, y, z = limits.clamp_velocity(99.0, -99.0, 9999.0)
    assert (x, y, z) == (
        limits.MAX_LINEAR_MS,
        -limits.MAX_LATERAL_MS,
        limits.MAX_YAW_RATE_DEG,
    )


def test_scale_can_leave_the_envelope() -> None:
    """``scale()`` validates, so scaling past the envelope raises.

    This is the path ``joy`` uses when it multiplies stick input by a speed
    factor of up to 5.0.
    """
    cmd = MovementCommand(x=limits.MAX_LINEAR_MS)
    with pytest.raises(ValidationError):
        cmd.scale(2.0)
