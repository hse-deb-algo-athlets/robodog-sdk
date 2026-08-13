"""The occupancy / cost grid, shared by the global map and the local window.

Two keys carry this type: ``nav/costmap/global`` — the deployment's map, loaded
once and already inflated by the robot radius — and ``nav/costmap/local``, a
rolling body-frame window rasterized from the LiDAR.

Kept out of :mod:`~robodog_sdk.msgs.navigation` because a consumer of the map
(a planner, a visualizer) does not otherwise care about the task contract.

The grid arrives as a flat list of ints. Deliberately no numpy: reshaping it is
one line for a project that has numpy, and this package stays installable in
seconds for one that does not.

.. code:: python

    import numpy as np

    grid = np.array(costmap.data, dtype=np.uint8).reshape(costmap.height, costmap.width)
"""

from __future__ import annotations

from typing import Final

from pydantic import BaseModel

#: Free space.
FREE: Final = 0

#: Inscribed obstacle — the robot's centre cannot be here without the robot
#: touching something. Everything at or above this value is impassable.
INSCRIBED: Final = 253

#: Lethal obstacle. Also what a lookup outside the grid returns: off the map is
#: not free.
LETHAL: Final = 255


class CostMap(BaseModel):
    """A 2D cost grid.

    Cell values run ``0`` (free) through ``1..252`` (rising cost — the decay
    around an obstacle that pushes a planner away from walls) to
    :data:`INSCRIBED` and :data:`LETHAL`, which are impassable.

    Row-major: ``data[row * width + col]``. ``(row 0, col 0)`` sits at
    ``(origin_x, origin_y)``; columns increase with x, rows with y.

    The global map is published already inflated by the robot radius, so a
    planner treating the robot as a point is correct — it must not inflate a
    second time.
    """

    origin_x: float  # m, map frame
    origin_y: float  # m, map frame
    resolution: float  # m per cell
    width: int  # cells
    height: int  # cells
    data: list[int]

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """``(row, col)`` for a point in metres. May be outside the grid."""
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return row, col

    def grid_to_world(self, row: int, col: int) -> tuple[float, float]:
        """``(x, y)`` at the centre of a cell, in metres."""
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def get_cost(self, row: int, col: int) -> int:
        """Cost at a cell. Out of bounds is :data:`LETHAL`, not an error."""
        if 0 <= row < self.height and 0 <= col < self.width:
            return self.data[row * self.width + col]
        return LETHAL

    def is_free(self, row: int, col: int, threshold: int = INSCRIBED) -> bool:
        """Whether a cell is below ``threshold``.

        The default admits every graded cost and rejects only the inscribed
        and lethal bands. Lower it to keep a margin from obstacles beyond the
        one already inflated into the map.
        """
        return self.get_cost(row, col) < threshold

    def is_free_at(self, x: float, y: float, threshold: int = INSCRIBED) -> bool:
        """:meth:`is_free` for a point in metres."""
        return self.is_free(*self.world_to_grid(x, y), threshold=threshold)
