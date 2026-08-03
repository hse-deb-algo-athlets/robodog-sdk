"""Frame identifiers and the conventions that make poses comparable.

Names only — this package provides no transform tree. Its purpose is to give
every node the same identifier for the same frame.
"""

from __future__ import annotations

from typing import Final

#: World-fixed frame the localization source publishes into. Every
#: :class:`~robodog_sdk.msgs.navigation.Pose2D` in the contract is expressed in
#: this frame unless documented otherwise.
MAP: Final = "map"

#: Odometry frame — continuous, drifting, no jumps. The Go2's onboard pose.
ODOM: Final = "odom"

#: Robot body frame: x forward, y left, z up (right-handed).
BASE_LINK: Final = "base_link"

#: Livox MID-360 sensor frame.
LIVOX: Final = "livox_frame"

#: Conventions that hold everywhere in this contract:
#:
#: - Linear velocity in m/s, distances in m.
#: - ``MovementCommand.z_deg`` is in deg/s; every other angle in the contract
#:   is in radians (``Pose2D.theta``, orientation deviations). The asymmetry is
#:   inherited from the Go2 command API and should not be extended to new
#:   topics.
#: - Yaw is counter-clockwise-positive about z, wrapped to [-pi, pi].
#: - Timestamps are timezone-aware UTC.
