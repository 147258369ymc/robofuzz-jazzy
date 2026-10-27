"""Occupancy-grid map context + BFS reachability gate for nav2 goal sampling.

Pure logic (numpy only); no ROS runtime. Grid convention matches
nav_msgs/OccupancyGrid: row-major, cell (col,row), value -1 unknown / 0 free /
100 occupied. Reachability is 4-neighbour BFS over free cells so a goal the
fuzzer picks is guaranteed geometrically connected to the robot start before
it is ever sent -- which is what makes the ABORTED "false failure" verdict
defensible.
"""

from collections import deque
import numpy as np
import yaml
from PIL import Image  # pgm decode; only used by load_occupancy_map


class MapContext:
    def __init__(self, grid, resolution, origin):
        self.grid = np.asarray(grid, dtype=np.int16)
        self.height, self.width = self.grid.shape
        self.resolution = float(resolution)
        self.origin = tuple(origin)  # (x, y, yaw); yaw assumed 0 here

    @classmethod
    def from_grid(cls, grid, resolution, origin):
        return cls(grid, resolution, origin)

    def world_to_cell(self, x, y):
        col = int(np.floor((x - self.origin[0]) / self.resolution))
        row = int(np.floor((y - self.origin[1]) / self.resolution))
        return col, row

    def cell_to_world(self, col, row):
        x = self.origin[0] + (col + 0.5) * self.resolution
        y = self.origin[1] + (row + 0.5) * self.resolution
        return x, y

    def _in_bounds(self, col, row):
        return 0 <= col < self.width and 0 <= row < self.height

    def is_free(self, col, row, occ_thresh=50):
        if not self._in_bounds(col, row):
            return False
        v = self.grid[row, col]
        return 0 <= v < occ_thresh

    def reachable(self, start_xy, goal_xy, occ_thresh=50):
        sc, sr = self.world_to_cell(*start_xy)
        gc, gr = self.world_to_cell(*goal_xy)
        if not self.is_free(sc, sr, occ_thresh) \
                or not self.is_free(gc, gr, occ_thresh):
            return False
        seen = {(sc, sr)}
        q = deque([(sc, sr)])
        while q:
            c, r = q.popleft()
            if (c, r) == (gc, gr):
                return True
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nc, nr = c + dc, r + dr
                if (nc, nr) not in seen and self.is_free(nc, nr, occ_thresh):
                    seen.add((nc, nr))
                    q.append((nc, nr))
        return False

    def clearance(self, x, y, max_r=1.0, occ_thresh=50):
        """Distance (m) from world point (x,y) to nearest occupied/unknown cell,
        capped at max_r. Used to reject goals inside nav2's costmap inflation
        layer (BFS-reachable != costmap-plannable), the root cause of the
        FALSE_FAILURE false positives observed in the 1000-round campaign.
        Out-of-bounds -> max_r (treated as open, permissive)."""
        import math as _m
        gc, gr = self.world_to_cell(x, y)
        if not self._in_bounds(gc, gr):
            return max_r
        steps = int(max_r / self.resolution)
        best = max_r
        for dr in range(-steps, steps + 1):
            for dc in range(-steps, steps + 1):
                c, r = gc + dc, gr + dr
                if self._in_bounds(c, r) and not self.is_free(c, r, occ_thresh):
                    d = _m.hypot(dc, dr) * self.resolution
                    if d < best:
                        best = d
        return best

    def nearest_free_cell(self, col, row, occ_thresh=50, max_radius=None):
        if self.is_free(col, row, occ_thresh):
            return col, row
        max_radius = max_radius or max(self.width, self.height)
        for radius in range(1, max_radius + 1):
            for dc in range(-radius, radius + 1):
                for dr in range(-radius, radius + 1):
                    nc, nr = col + dc, row + dr
                    if self.is_free(nc, nr, occ_thresh):
                        return nc, nr
        return None


def load_occupancy_map(yaml_path):
    """Load a map_server-style .yaml + .pgm into a MapContext.

    Verified only at smoke time (needs a real map file); not unit tested.
    """
    import os
    with open(yaml_path, "r", encoding="utf-8") as fp:
        meta = yaml.safe_load(fp)
    img_path = meta["image"]
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(yaml_path), img_path)
    resolution = float(meta["resolution"])
    origin = tuple(meta.get("origin", [0.0, 0.0, 0.0]))
    negate = int(meta.get("negate", 0))
    occ_th = float(meta.get("occupied_thresh", 0.65))
    free_th = float(meta.get("free_thresh", 0.196))

    img = np.array(Image.open(img_path).convert("L"), dtype=np.float32)
    p = (255.0 - img) / 255.0 if negate == 0 else img / 255.0
    grid = np.full(img.shape, -1, dtype=np.int16)
    grid[p > occ_th] = 100
    grid[p < free_th] = 0
    # pgm row 0 is top; OccupancyGrid row 0 is bottom -> flip vertically
    grid = np.flipud(grid)
    return MapContext(grid, resolution, origin)
