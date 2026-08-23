"""The occupancy grids: what the SLAM map is, and what the planner sees.

:class:`GridMap` on ``map/grid`` is the map itself, exactly as SLAM built it —
a PNG raster plus its georeference, uninflated, the input every other consumer
should start from.

:class:`CostMap` is the planner's view of it, and two keys carry it:
``nav/costmap/global`` — that same map, already inflated by the robot radius —
and ``nav/costmap/local``, a rolling body-frame window rasterized from the
LiDAR.

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

import base64
from datetime import UTC, datetime
from typing import Final

from pydantic import BaseModel, Field


def _utcnow() -> datetime:
    return datetime.now(UTC)


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


class GridMap(BaseModel):
    """The occupancy grid a SLAM session built, mirrored onto the bus.

    This is the map as MOLA produced it — raw, and in the ``map_server``
    format its own tooling writes. It is not :class:`CostMap`: nothing here is
    inflated by a robot radius, so it is the right input for *any* consumer,
    and the wrong thing to plan on without inflating it first.

    The raster travels as the PNG itself rather than as a cell list, which is
    two orders of magnitude smaller — a hall-sized grid is a couple of
    kilobytes as a PNG and hundreds as ``list[int]``. Raster and georeference
    ride in one message on purpose: split across two keys there is a window
    where a consumer holds a new image against an old origin, and a map drawn
    on the wrong georeference is worse than no map.

    **Row order is not** :class:`CostMap`'s. The PNG follows the image
    convention — row 0 is the *top*, the largest y — while ``CostMap`` row 0
    sits at ``origin``. Flip vertically when converting, or the building comes
    out mirrored::

        import numpy as np
        from PIL import Image

        px = np.flipud(np.array(Image.open(io.BytesIO(grid.png_bytes()))))

    Two independent things can be missing, and the pair of them is the whole
    state. ``map_id`` names the session the deployment is operating in, and is
    ``None`` when it is operating in none. ``image_png`` is empty when no grid
    has been built for that session yet — which is the normal state during
    mapping, and after recording a session but before ``build-grid``. So
    ``map_id`` set with an empty raster reads as "this frame is live, but I
    cannot draw it for you", which is different from having no frame at all.
    Every default is that second, emptier answer, so a payload that is only
    partly understood cannot read as a usable map.
    """

    #: Identifier of the map — the SLAM session name. Changes when the map
    #: does, which is the point: it is the same value as
    #: :attr:`~robodog_sdk.msgs.localization.MapIdentity.map_id`, so a stored
    #: map-frame coordinate can be checked against it. ``None`` means the
    #: producer is operating in no map at all.
    map_id: str | None = None
    #: Opaque token that changes when, and only when, the raster is rebuilt.
    #: Two messages with the same ``map_id`` and ``revision`` carry the same
    #: grid, so a consumer can skip re-decoding one it already holds.
    revision: str | None = None
    #: Metres per cell.
    resolution: float = 0.0
    #: World coordinates of the grid's lower-left corner, in metres — the
    #: bottom-left of the image once it is flipped into map orientation.
    origin_x: float = 0.0
    origin_y: float = 0.0
    #: Rotation of the grid about that corner, in radians. Zero in every map
    #: MOLA writes; carried because the ``map_server`` format has it.
    origin_theta: float = 0.0
    #: Raster size in cells, so a consumer can size a canvas or reject a
    #: mismatch without decoding the image.
    width: int = 0
    height: int = 0
    #: Occupancy probability at or above which a cell counts as an obstacle.
    occupied_thresh: float = 0.65
    #: Occupancy probability at or below which a cell counts as free. Between
    #: the two thresholds is *unknown*, not "somewhat occupied".
    free_thresh: float = 0.196
    #: Whether the raster's greyscale is inverted with respect to occupancy.
    #: Handled by :meth:`probability`; consumers doing their own arithmetic
    #: must honour it or they read every wall as open floor.
    negate: bool = False
    #: The grid raster: a PNG, base64-encoded (RFC 4648). Empty when no grid
    #: has been built — see the class docstring. Use :meth:`png_bytes`.
    image_png: str = ""
    #: What produced the map, e.g. ``"mola"``.
    source: str = "mola"
    #: When this was published. A latched value can be old; this is how old.
    stamp: datetime = Field(default_factory=_utcnow)

    @property
    def available(self) -> bool:
        """Whether this message carries a grid that can actually be drawn."""
        return bool(self.map_id) and bool(self.image_png)

    def png_bytes(self) -> bytes:
        """The raster, decoded. Empty when no grid has been built."""
        return base64.b64decode(self.image_png) if self.image_png else b""

    def probability(self, pixel: int) -> float:
        """Occupancy probability ``0.0..1.0`` for one greyscale pixel value.

        The ``map_server`` rule, :attr:`negate` included: darker means more
        occupied unless the raster says otherwise. Compare the result against
        :attr:`occupied_thresh` / :attr:`free_thresh` — or use
        :meth:`is_occupied` and :meth:`is_free`, which do exactly that.
        """
        return (pixel if self.negate else 255 - pixel) / 255.0

    def is_occupied(self, pixel: int) -> bool:
        """Whether a pixel is an obstacle."""
        return self.probability(pixel) >= self.occupied_thresh

    def is_free(self, pixel: int) -> bool:
        """Whether a pixel is known-free. Neither this nor
        :meth:`is_occupied` is unknown space — that is the band between."""
        return self.probability(pixel) <= self.free_thresh
