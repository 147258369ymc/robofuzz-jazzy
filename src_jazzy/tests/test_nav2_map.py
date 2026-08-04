import os, sys, unittest
import numpy as np

SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

import nav2_map  # noqa: E402


def _grid():
    # 5x5 free (0) with a vertical wall (100) at col=2 rows 0..3 (gap at row4)
    g = np.zeros((5, 5), dtype=np.int8)
    g[0:4, 2] = 100
    return g


class TestMapContext(unittest.TestCase):
    def setUp(self):
        self.m = nav2_map.MapContext.from_grid(
            _grid(), resolution=1.0, origin=(0.0, 0.0, 0.0))

    def test_world_cell_roundtrip(self):
        col, row = self.m.world_to_cell(2.5, 3.5)
        x, y = self.m.cell_to_world(col, row)
        self.assertAlmostEqual(x, 2.5, places=6)
        self.assertAlmostEqual(y, 3.5, places=6)

    def test_is_free(self):
        c, r = self.m.world_to_cell(2.5, 0.5)  # wall col
        self.assertFalse(self.m.is_free(c, r))
        c, r = self.m.world_to_cell(0.5, 0.5)  # free
        self.assertTrue(self.m.is_free(c, r))

    def test_reachable_through_gap(self):
        # left side to right side only via the row-4 gap
        self.assertTrue(self.m.reachable((0.5, 0.5), (4.5, 0.5)))

    def test_unreachable_when_walled(self):
        walled = _grid()
        walled[4, 2] = 100  # close the gap -> fully separated
        m = nav2_map.MapContext.from_grid(walled, 1.0, (0.0, 0.0, 0.0))
        self.assertFalse(m.reachable((0.5, 0.5), (4.5, 0.5)))

    def test_nearest_free_cell(self):
        c, r = self.m.world_to_cell(2.5, 0.5)  # wall
        nc, nr = self.m.nearest_free_cell(c, r)
        self.assertTrue(self.m.is_free(nc, nr))

    def test_world_to_cell_floors_negative(self):
        # origin at (-2,-2); a point below origin must map to a negative
        # cell index (out of bounds -> not free), not be truncated into cell 0.
        m = nav2_map.MapContext.from_grid(
            np.zeros((5, 5), dtype=np.int8), resolution=1.0,
            origin=(-2.0, -2.0, 0.0))
        col, row = m.world_to_cell(-2.5, -2.5)  # 0.5m below origin corner
        self.assertEqual((col, row), (-1, -1))
        self.assertFalse(m.is_free(col, row))
