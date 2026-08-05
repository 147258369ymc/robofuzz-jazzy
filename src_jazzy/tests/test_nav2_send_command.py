import math, os, sys, unittest
from types import SimpleNamespace as NS
import numpy as np

SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

import nav2_map          # noqa: E402
import harness           # noqa: E402


def _quat(yaw):
    return NS(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

def _pose(x, y, yaw=0.0):
    return NS(pose=NS(position=NS(x=x, y=y, z=0.0), orientation=_quat(yaw)))

def _map():
    g = np.zeros((5, 5), dtype=np.int8)
    g[2, 2] = 100  # single occupied cell at world ~ (2.5, 2.5)
    return nav2_map.MapContext.from_grid(g, 1.0, (0.0, 0.0, 0.0))


class TestResolveGoal(unittest.TestCase):
    def test_free_goal_unchanged(self):
        x, y, yaw, snapped = harness._nav2_resolve_goal(
            _pose(0.5, 0.5, 0.0), _map(), start_xy=(0.5, 0.5))
        self.assertAlmostEqual(x, 0.5, places=6)
        self.assertFalse(snapped)

    def test_occupied_goal_snapped_to_free(self):
        x, y, yaw, snapped = harness._nav2_resolve_goal(
            _pose(2.5, 2.5, 0.0), _map(), start_xy=(0.5, 0.5))
        self.assertTrue(snapped)
        c, r = _map().world_to_cell(x, y)
        self.assertTrue(_map().is_free(c, r))

    def test_no_map_returns_raw_pose(self):
        x, y, yaw, snapped = harness._nav2_resolve_goal(
            _pose(9.0, 9.0, 1.0), None, start_xy=(0.0, 0.0))
        self.assertAlmostEqual(x, 9.0, places=6)
        self.assertFalse(snapped)
