import math
import os
import sys
import unittest
from types import SimpleNamespace as NS


SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)


def _vec(x=0.0, y=0.0, z=0.0):
    return NS(x=x, y=y, z=z)


def _quat(x=0.0, y=0.0, z=0.0, w=1.0):
    return NS(x=x, y=y, z=z, w=w)


def make_odom(
    px=0.0, py=0.0, pz=0.0,
    qx=0.0, qy=0.0, qz=0.0, qw=1.0,
    lx=0.0, ly=0.0, lz=0.0,
    ax=0.0, ay=0.0, az=0.0,
):
    pose = NS(pose=NS(position=_vec(px, py, pz),
                      orientation=_quat(qx, qy, qz, qw)))
    twist = NS(twist=NS(linear=_vec(lx, ly, lz), angular=_vec(ax, ay, az)))
    return NS(pose=pose, twist=twist)


def make_scan(ranges, range_min=0.1, range_max=12.0):
    return NS(ranges=list(ranges), range_min=range_min, range_max=range_max)


def make_twist(lx=0.0, az=0.0):
    """geometry_msgs/msg/Twist shape (no .twist attribute)."""
    return NS(linear=_vec(lx, 0.0, 0.0), angular=_vec(0.0, 0.0, az))


def make_twist_stamped(lx=0.0, az=0.0):
    """geometry_msgs/msg/TwistStamped shape (.twist wrapper)."""
    return NS(twist=make_twist(lx, az))


def tb4_config():
    return NS(oracle_mode="turtlebot4_jazzy")


def odom_series(n, **kw):
    """n identical odom samples with incrementing timestamps."""
    return [(1_000_000_000 + i, make_odom(**kw)) for i in range(n)]


class TurtleBot4SmokeOracleTests(unittest.TestCase):
    def setUp(self):
        import oracles.turtlebot as tb
        self.tb = tb
        self.config = tb4_config()

    def run_check(self, msg_list, state_dict, feedback_list=None):
        return self.tb.check(
            self.config, msg_list, state_dict, feedback_list or []
        )

    # --- existence ---
    def test_clean_state_has_no_errors(self):
        state = {
            "/odom": odom_series(6, lx=0.1),
            "/scan": [(1, make_scan([0.5, 1.0, float("inf"), 2.0]))],
        }
        msg_list = [make_twist(lx=0.1)]
        errs = self.run_check(msg_list, state)
        self.assertEqual(errs, [])

    def test_missing_odom_reports_error(self):
        state = {"/scan": [(1, make_scan([1.0, 2.0]))]}
        errs = self.run_check([], state)
        self.assertTrue(any("/odom" in e for e in errs), errs)

    def test_missing_scan_reports_error(self):
        state = {"/odom": odom_series(3)}
        errs = self.run_check([], state)
        self.assertTrue(any("/scan" in e for e in errs), errs)

    # --- scan sanity ---
    def test_scan_nan_reports_error(self):
        state = {
            "/odom": odom_series(3),
            "/scan": [(1, make_scan([1.0, float("nan"), 2.0]))],
        }
        errs = self.run_check([], state)
        self.assertTrue(any("NaN" in e for e in errs), errs)

    def test_scan_negative_reports_error(self):
        state = {
            "/odom": odom_series(3),
            "/scan": [(1, make_scan([1.0, -0.5, 2.0]))],
        }
        errs = self.run_check([], state)
        self.assertTrue(any("negative" in e for e in errs), errs)

    def test_scan_inf_is_allowed(self):
        state = {
            "/odom": odom_series(3),
            "/scan": [(1, make_scan([float("inf"), float("inf"), 1.5]))],
        }
        errs = self.run_check([], state)
        self.assertEqual(errs, [])

    def test_scan_value_outside_declared_range_reports_error(self):
        # range_max is 12.0; 50.0 finite value is far outside + tolerance
        state = {
            "/odom": odom_series(3),
            "/scan": [(1, make_scan([1.0, 50.0], range_min=0.1,
                                    range_max=12.0))],
        }
        errs = self.run_check([], state)
        self.assertTrue(any("range" in e.lower() for e in errs), errs)

    # --- odom sanity ---
    def test_odom_nan_position_reports_error(self):
        state = {
            "/odom": [(1, make_odom(px=float("nan")))],
            "/scan": [(1, make_scan([1.0]))],
        }
        errs = self.run_check([], state)
        self.assertTrue(any("odom" in e.lower() for e in errs), errs)

    def test_odom_inf_twist_reports_error(self):
        state = {
            "/odom": [(1, make_odom(lx=float("inf")))],
            "/scan": [(1, make_scan([1.0]))],
        }
        errs = self.run_check([], state)
        self.assertTrue(any("odom" in e.lower() for e in errs), errs)

    def test_quaternion_norm_far_from_unit_reports_error(self):
        # quaternion (0,0,0,0) has norm 0, far from 1.0
        state = {
            "/odom": [(1, make_odom(qw=0.0))],
            "/scan": [(1, make_scan([1.0]))],
        }
        errs = self.run_check([], state)
        self.assertTrue(any("quaternion" in e.lower() for e in errs), errs)

    # --- cmd/odom consistency (windowed, msg_list only) ---
    def test_sustained_cmd_odom_conflict_reports_error(self):
        # commanded forward, but odom consistently reverse
        state = {
            "/odom": odom_series(6, lx=-0.2),
            "/scan": [(1, make_scan([1.0]))],
        }
        msg_list = [make_twist(lx=0.15)]
        errs = self.run_check(msg_list, state)
        self.assertTrue(
            any("conflict" in e.lower() or "odom" in e.lower() for e in errs),
            errs,
        )

    def test_sustained_angular_cmd_odom_conflict_reports_error(self):
        # commanded left turn, but odom consistently turns right
        state = {
            "/odom": odom_series(6, az=-0.3),
            "/scan": [(1, make_scan([1.0]))],
        }
        msg_list = [make_twist(az=0.4)]
        errs = self.run_check(msg_list, state)
        self.assertTrue(
            any("angular" in e.lower() or "turn" in e.lower()
                for e in errs),
            errs,
        )

    def test_single_transient_opposite_sample_is_tolerated(self):
        # mostly forward odom matching a forward command, one reverse blip
        odom = odom_series(6, lx=0.12)
        odom[2] = (odom[2][0], make_odom(lx=-0.2))  # single transient
        state = {"/odom": odom, "/scan": [(1, make_scan([1.0]))]}
        msg_list = [make_twist(lx=0.12)]
        errs = self.run_check(msg_list, state)
        self.assertEqual(errs, [])

    def test_mixed_forward_reverse_sequence_does_not_use_last_cmd_as_round_truth(self):
        state = {
            "/odom": odom_series(6, lx=0.09),
            "/scan": [(1, make_scan([1.0]))],
        }
        msg_list = [
            make_twist(lx=0.15),
            make_twist(lx=0.15),
            make_twist(lx=-0.15),
        ]
        errs = self.run_check(msg_list, state)
        self.assertFalse(any("reverse command conflicts" in e for e in errs), errs)

    def test_command_extractor_handles_twist_and_twist_stamped(self):
        self.assertEqual(self.tb._cmd_twist(make_twist(lx=0.3)).linear.x, 0.3)
        self.assertEqual(
            self.tb._cmd_twist(make_twist_stamped(lx=0.4)).linear.x, 0.4
        )

    # --- feedback population ---
    def test_feedback_metrics_are_populated(self):
        from feedback import Feedback, FeedbackType
        fbk_names = [
            "scan_min_range",
            "scan_invalid_ratio",
            "cmd_odom_linear_agreement",
            "cmd_odom_angular_agreement",
        ]
        feedback_list = [Feedback(n, FeedbackType.INC) for n in fbk_names]
        state = {
            "/odom": odom_series(6, lx=0.1),
            "/scan": [(1, make_scan([0.5, 1.0, 2.0]))],
        }
        msg_list = [make_twist(lx=0.1)]
        self.run_check(msg_list, state, feedback_list)
        by_name = {f.name: f for f in feedback_list}
        self.assertIsNotNone(by_name["scan_min_range"].value)
        self.assertIsNotNone(by_name["scan_invalid_ratio"].value)


if __name__ == "__main__":
    unittest.main()
