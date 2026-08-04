import os, sys, unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

import checker  # noqa: E402


class TestNav2Routing(unittest.TestCase):
    def test_routes_to_nav2_oracle(self):
        cfg = NS(rospkg="", oracle_mode="nav2_jazzy", topic_aliases={})
        state = {"/robofuzz/nav2_goal": [], "/sim_ground_truth_pose": []}
        with patch("oracles.nav2.check", return_value=["SENTINEL"]) as m:
            errs = checker.run_checks(cfg, [], state, [])
        m.assert_called_once()
        self.assertEqual(errs, ["SENTINEL"])

    def test_slam_still_routes_to_slam(self):
        cfg = NS(rospkg="", oracle_mode="slam_toolbox_jazzy", topic_aliases={})
        state = {"/pose": [], "/sim_ground_truth_pose": []}
        with patch("oracles.slam.check", return_value=["SLAM"]) as m:
            errs = checker.run_checks(cfg, [], state, [])
        m.assert_called_once()
        self.assertEqual(errs, ["SLAM"])
