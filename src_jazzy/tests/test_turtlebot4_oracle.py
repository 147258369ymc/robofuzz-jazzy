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
    stamp_sec=None, stamp_nanosec=0,
):
    header = NS(stamp=NS(sec=stamp_sec, nanosec=stamp_nanosec))
    pose = NS(pose=NS(position=_vec(px, py, pz),
                      orientation=_quat(qx, qy, qz, qw)))
    twist = NS(twist=NS(linear=_vec(lx, ly, lz), angular=_vec(ax, ay, az)))
    return NS(header=header, pose=pose, twist=twist)


def make_scan(ranges, range_min=0.1, range_max=12.0):
    return NS(ranges=list(ranges), range_min=range_min, range_max=range_max)


def make_wheel_vels(left=0.0, right=0.0):
    return NS(velocity_left=left, velocity_right=right)


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


def odom_ns_series(values, field="lx", start_ns=1_000_000_000_000_000_000,
                   step_ns=100_000_000):
    samples = []
    for i, value in enumerate(values):
        kwargs = {field: value}
        samples.append((start_ns + i * step_ns, make_odom(**kwargs)))
    return samples


def odom_header_series(values, field="lx",
                       bag_start_ns=1_000_000_000_000_000_000,
                       bag_step_ns=1_000_000,
                       header_start_ns=2_000_000_000,
                       header_step_ns=100_000_000):
    samples = []
    for i, value in enumerate(values):
        header_ns = header_start_ns + i * header_step_ns
        kwargs = {
            field: value,
            "stamp_sec": header_ns // 1_000_000_000,
            "stamp_nanosec": header_ns % 1_000_000_000,
        }
        samples.append((bag_start_ns + i * bag_step_ns, make_odom(**kwargs)))
    return samples


class FeedbackProbe:
    def __init__(self, name):
        self.name = name
        self.value = None

    def update_value(self, value):
        self.value = value


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

    def test_missing_scan_is_not_a_motion_bug(self):
        state = {"/odom": odom_series(3)}
        errs = self.run_check([], state)
        self.assertFalse(any("/scan" in e for e in errs), errs)

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

    def test_all_inf_scan_feedback_is_not_zero_distance(self):
        state = {
            "/odom": odom_series(3),
            "/scan": [(1, make_scan([float("inf"), float("inf")],
                                    range_max=12.0))],
        }
        feedback = [FeedbackProbe("scan_min_range")]

        errs = self.run_check([], state, feedback)

        self.assertEqual(errs, [])
        self.assertGreater(feedback[0].value, 1.0)

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
            "tb4_cmd_linear_velocity_ratio",
            "tb4_cmd_angular_velocity_ratio",
            "tb4_odom_linear_velocity_ratio",
            "tb4_odom_angular_velocity_ratio",
            "tb4_linear_accel_ratio",
            "tb4_angular_accel_ratio",
            "tb4_odom_publish_gap",
        ]
        feedback_list = [Feedback(n, FeedbackType.INC) for n in fbk_names]
        state = {
            "/odom": odom_ns_series([0.0, 0.05, 0.1, 0.1, 0.1, 0.1]),
            "/scan": [(1, make_scan([0.5, 1.0, 2.0]))],
        }
        msg_list = [make_twist(lx=0.1)]
        self.run_check(msg_list, state, feedback_list)
        by_name = {f.name: f for f in feedback_list}
        self.assertIsNotNone(by_name["scan_min_range"].value)
        self.assertIsNotNone(by_name["scan_invalid_ratio"].value)
        self.assertIsNotNone(by_name["tb4_cmd_linear_velocity_ratio"].value)
        self.assertIsNotNone(by_name["tb4_linear_accel_ratio"].value)
        self.assertIsNotNone(by_name["tb4_odom_publish_gap"].value)

    # --- deeper OracleIR-derived checks ---
    def test_command_velocity_envelope_uses_whitelist_fields(self):
        state = {
            "/odom": odom_ns_series([0.0, 0.0, 0.0]),
            "/scan": [(1, make_scan([1.0]))],
        }
        feedback = [
            FeedbackProbe("tb4_cmd_linear_velocity_ratio"),
            FeedbackProbe("tb4_cmd_angular_velocity_ratio"),
        ]

        errs = self.run_check([make_twist(lx=0.60, az=2.10)], state,
                              feedback)

        self.assertTrue(any("cmd_vel linear velocity envelope" in e
                            for e in errs), errs)
        self.assertTrue(any("cmd_vel angular velocity envelope" in e
                            for e in errs), errs)
        self.assertGreater(feedback[0].value, 1.0)
        self.assertGreater(feedback[1].value, 1.0)

    def test_odom_velocity_envelope_reports_error_and_feedback(self):
        state = {
            "/odom": odom_ns_series([0.55, 0.56, 0.57]),
            "/scan": [(1, make_scan([1.0]))],
        }
        feedback = [FeedbackProbe("tb4_odom_linear_velocity_ratio")]

        errs = self.run_check([make_twist(lx=0.15)], state, feedback)

        self.assertTrue(any("linear velocity envelope" in e for e in errs),
                        errs)
        self.assertGreater(feedback[0].value, 1.0)

    def test_odom_acceleration_uses_adjacent_samples(self):
        state = {
            "/odom": odom_ns_series([0.0, 0.2, 0.4]),
            "/scan": [(1, make_scan([1.0]))],
        }
        feedback = [FeedbackProbe("tb4_linear_accel_ratio")]

        errs = self.run_check([make_twist(lx=0.15)], state, feedback)

        self.assertTrue(any("linear acceleration envelope" in e
                            for e in errs), errs)
        self.assertGreater(feedback[0].value, 1.0)

    def test_single_mild_acceleration_spike_is_feedback_only(self):
        state = {
            "/odom": odom_ns_series([0.0, 0.11, 0.11, 0.11]),
            "/scan": [(1, make_scan([1.0]))],
        }
        feedback = [FeedbackProbe("tb4_linear_accel_ratio")]

        errs = self.run_check([make_twist(lx=0.15)], state, feedback)

        self.assertFalse(any("linear acceleration envelope" in e
                             for e in errs), errs)
        self.assertGreater(feedback[0].value, 1.0)

    def test_acceleration_uses_odom_header_stamp_over_bag_burst(self):
        state = {
            "/odom": odom_header_series([0.0, 0.11, 0.11, 0.11],
                                        bag_step_ns=200_000,
                                        header_step_ns=100_000_000),
            "/scan": [(1, make_scan([1.0]))],
        }
        feedback = [FeedbackProbe("tb4_linear_accel_ratio")]

        errs = self.run_check([make_twist(lx=0.15)], state, feedback)

        self.assertFalse(any("linear acceleration envelope" in e
                             for e in errs), errs)
        self.assertGreater(feedback[0].value, 1.0)
        self.assertLess(feedback[0].value, self.tb.TB4_ACCEL_STRONG_RATIO_ERROR)

    def test_header_time_sustained_acceleration_reports_error(self):
        state = {
            "/odom": odom_header_series([0.0, 0.12, 0.24, 0.24],
                                        bag_step_ns=100_000_000,
                                        header_step_ns=100_000_000),
            "/scan": [(1, make_scan([1.0]))],
        }

        errs = self.run_check([make_twist(lx=0.15)], state)

        self.assertTrue(any("linear acceleration envelope" in e
                            for e in errs), errs)

    def test_strong_single_acceleration_spike_still_reports_error(self):
        state = {
            "/odom": odom_ns_series([0.0, 0.60, 0.60, 0.60]),
            "/scan": [(1, make_scan([1.0]))],
        }

        errs = self.run_check([make_twist(lx=0.15)], state)

        self.assertTrue(any("linear acceleration envelope" in e
                            for e in errs), errs)

    def test_timeout_requires_recorded_command_coverage(self):
        base = 1_000_000_000_000_000_000
        state = {
            "/cmd_vel": [(base, make_twist_stamped(lx=0.1))],
            "/odom": [
                (base + 1_100_000_000, make_odom(lx=0.12)),
                (base + 1_200_000_000, make_odom(lx=0.12)),
                (base + 1_300_000_000, make_odom(lx=0.12)),
            ],
            "/scan": [(base, make_scan([1.0]))],
        }
        msg_list = [make_twist_stamped(lx=0.1) for _ in range(8)]
        feedback = [FeedbackProbe("tb4_cmd_timeout_motion")]

        errs = self.run_check(msg_list, state, feedback)

        self.assertFalse(any("cmd_vel timeout" in e for e in errs), errs)
        self.assertGreater(feedback[0].value, 0.0)

    def test_timeout_requires_complete_recorded_command_sequence(self):
        base = 1_000_000_000_000_000_000
        state = {
            "/cmd_vel": [
                (base + i * 100_000_000, make_twist_stamped(lx=0.1))
                for i in range(7)
            ],
            "/odom": [
                (base + 1_400_000_000, make_odom(lx=0.12)),
                (base + 1_500_000_000, make_odom(lx=0.12)),
                (base + 1_600_000_000, make_odom(lx=0.12)),
            ],
            "/scan": [(base, make_scan([1.0]))],
        }
        msg_list = [make_twist_stamped(lx=0.1) for _ in range(8)]
        feedback = [FeedbackProbe("tb4_cmd_timeout_motion")]

        errs = self.run_check(msg_list, state, feedback)

        self.assertFalse(any("cmd_vel timeout" in e for e in errs), errs)
        self.assertGreater(feedback[0].value, 0.0)

    def test_stale_command_with_continuing_motion_reports_error(self):
        base = 1_000_000_000_000_000_000
        state = {
            "/cmd_vel": [
                (base + i * 100_000_000, make_twist_stamped(lx=0.1))
                for i in range(8)
            ],
            "/odom": [
                (base + 1_500_000_000, make_odom(lx=0.12)),
                (base + 1_600_000_000, make_odom(lx=0.12)),
                (base + 1_700_000_000, make_odom(lx=0.12)),
            ],
            "/scan": [(base, make_scan([1.0]))],
        }
        feedback = [FeedbackProbe("tb4_cmd_timeout_motion")]

        errs = self.run_check([make_twist_stamped(lx=0.1) for _ in range(8)],
                              state, feedback)

        self.assertTrue(any("cmd_vel timeout" in e for e in errs), errs)
        self.assertGreater(feedback[0].value, 0.0)

    def test_short_stale_motion_after_timeout_is_feedback_only(self):
        base = 1_000_000_000_000_000_000
        state = {
            "/cmd_vel": [(base, make_twist_stamped(lx=0.1))],
            "/odom": [
                (base + 520_000_000, make_odom(lx=0.12)),
                (base + 540_000_000, make_odom(lx=0.12)),
                (base + 560_000_000, make_odom(lx=0.12)),
            ],
            "/scan": [(base, make_scan([1.0]))],
        }
        feedback = [FeedbackProbe("tb4_cmd_timeout_motion")]

        errs = self.run_check([make_twist(lx=0.1)], state, feedback)

        self.assertFalse(any("cmd_vel timeout" in e for e in errs), errs)
        self.assertGreater(feedback[0].value, 0.0)

    def test_wheel_odom_sign_conflict_reports_error(self):
        base = 1_000_000_000_000_000_000
        state = {
            "/odom": odom_ns_series([-0.12, -0.11, -0.10], start_ns=base),
            "/wheel_vels": [
                (base + i * 100_000_000, make_wheel_vels(1.0, 1.1))
                for i in range(3)
            ],
            "/scan": [(base, make_scan([1.0]))],
        }
        feedback = [FeedbackProbe("tb4_wheel_odom_consistency_error")]

        errs = self.run_check([], state, feedback)

        self.assertTrue(any("wheel/odom direction conflict" in e
                            for e in errs), errs)
        self.assertGreater(feedback[0].value, 0.0)


if __name__ == "__main__":
    unittest.main()
