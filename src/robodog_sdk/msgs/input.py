"""Human input devices.

Published by the teleoperation node, not by the robot. Under ``input/`` rather
than ``node/`` — the latter is reserved for zenode's presence, health, log and
trace keys.
"""

from __future__ import annotations

from pydantic import BaseModel, Field


class AnalogInput(BaseModel):
    """A 2D analog axis, normalized to -1.0…1.0."""

    x: float = 0.0
    y: float = 0.0


class Axis(BaseModel):
    """All analog axes on a gamepad."""

    left_stick: AnalogInput = Field(default_factory=AnalogInput)
    right_stick: AnalogInput = Field(default_factory=AnalogInput)
    left_trigger: float = 0.0
    right_trigger: float = 0.0


class Buttons(BaseModel):
    """Digital button state for a standard gamepad layout."""

    a: bool = False
    b: bool = False
    x: bool = False
    y: bool = False
    lb: bool = False
    rb: bool = False
    back: bool = False
    start: bool = False
    dpad_up: bool = False
    dpad_down: bool = False
    dpad_left: bool = False
    dpad_right: bool = False


class GamepadState(BaseModel):
    """Complete gamepad snapshot.

    Raw input, not a command: the teleoperation node maps this onto the
    gateway inlet as ``controller``. A consumer reading it directly is observing what
    the operator is doing, not driving the robot.
    """

    axis: Axis = Field(default_factory=Axis)
    buttons: Buttons = Field(default_factory=Buttons)


class GamepadStatus(BaseModel):
    """Whether a gamepad is connected and actively sending input."""

    connected: bool = False
    active: bool = False
