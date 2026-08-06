import os, sys, unittest
import numpy as np

SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

import nav2_map  # noqa: E402
import seed_generator  # noqa: E402


def _open_grid():
    # 8x8 fully free arena, resolution 1.0, origin at (0,0)
    return np.zeros((8, 8), dtype=np.int8)


class TestGenerateNav2GoalSeeds(unittest.TestCase):
    def setUp(self):
        self.m = nav2_map.MapContext.from_grid(
            _open_grid(), resolution=1.0, origin=(0.0, 0.0, 0.0))

    def test_returns_reachable_goals(self):
        seeds = seed_generator.generate_nav2_goal_seeds(
            self.m, start_xy=(0.5, 0.5), count=6)
        self.assertTrue(len(seeds) > 0)
        for (x, y, yaw) in seeds:
            self.assertTrue(self.m.reachable((0.5, 0.5), (x, y)))

    def test_respects_count_cap(self):
        seeds = seed_generator.generate_nav2_goal_seeds(
            self.m, start_xy=(0.5, 0.5), count=4)
        self.assertLessEqual(len(seeds), 4)

    def test_yaw_varies(self):
        seeds = seed_generator.generate_nav2_goal_seeds(
            self.m, start_xy=(0.5, 0.5), count=6)
        yaws = {round(yaw, 4) for (_, _, yaw) in seeds}
        self.assertGreater(len(yaws), 1)

    def test_tuple_shape(self):
        seeds = seed_generator.generate_nav2_goal_seeds(
            self.m, start_xy=(0.5, 0.5), count=3)
        for entry in seeds:
            self.assertEqual(len(entry), 3)
            for v in entry:
                self.assertIsInstance(v, float)

    def test_deterministic(self):
        a = seed_generator.generate_nav2_goal_seeds(
            self.m, start_xy=(0.5, 0.5), count=6)
        b = seed_generator.generate_nav2_goal_seeds(
            self.m, start_xy=(0.5, 0.5), count=6)
        self.assertEqual(a, b)

    def test_empty_grid_returns_empty(self):
        walled = np.full((5, 5), 100, dtype=np.int8)
        m = nav2_map.MapContext.from_grid(walled, 1.0, (0.0, 0.0, 0.0))
        self.assertEqual(
            seed_generator.generate_nav2_goal_seeds(m, count=6), [])


if __name__ == "__main__":
    unittest.main()
