import math
import os
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch


SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from oracles import slam  # noqa: E402


# --- message shape helpers -------------------------------------------------

def _vec(x=0.0, y=0.0, z=0.0):
    return NS(x=x, y=y, z=z)


def _quat_from_yaw(yaw):
    # planar rotation about z: (0, 0, sin(yaw/2), cos(yaw/2))
    return NS(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def make_pose_cov(x=0.0, y=0.0, yaw=0.0):
    """PoseWithCovarianceStamped / Odometry shape (.pose.pose.*)."""
    return NS(pose=NS(pose=NS(position=_vec(x, y),
                              orientation=_quat_from_yaw(yaw))))


def make_pose_stamped(x=0.0, y=0.0, yaw=0.0):
    """PoseStamped shape (.pose.* — no inner .pose)."""
    return NS(pose=NS(position=_vec(x, y), orientation=_quat_from_yaw(yaw)))


def with_header_stamp(msg, sec, nanosec):
    msg.header = NS(stamp=NS(sec=sec, nanosec=nanosec))
    return msg


START_NS = 1_000_000_000_000_000_000  # 16-digit-trim -> 1e9 sec
STEP_NS = 100_000_000                  # 0.1 s -> 10 Hz


def track_to_msgs(track, maker=make_pose_cov, start_ns=START_NS,
                  step_ns=STEP_NS):
    """track: list of (x, y, yaw) -> list of (ts_ns, msg)."""
    return [
        (start_ns + i * step_ns, maker(x, y, yaw))
        for i, (x, y, yaw) in enumerate(track)
    ]


def track_to_header_msgs(track, header_start_sec, maker=make_pose_cov,
                         bag_start_ns=START_NS, step_ns=STEP_NS):
    """Build records whose ROS header time is independent of bag time."""
    header_start_ns = int(header_start_sec * 1_000_000_000)
    records = []
    for i, (x, y, yaw) in enumerate(track):
        total_ns = header_start_ns + i * step_ns
        sec, nanosec = divmod(total_ns, 1_000_000_000)
        msg = with_header_stamp(maker(x, y, yaw), sec, nanosec)
        records.append((bag_start_ns + i * step_ns, msg))
    return records


def straight_truth(n, step=0.05, yaw=0.0):
    """Robot moving along +x at constant heading."""
    return [(step * i, 0.0, yaw) for i in range(n)]


def apply_rigid(track, dx, dy, dtheta):
    """Rigid-transform a whole track (rotate about origin then translate)."""
    c, s = math.cos(dtheta), math.sin(dtheta)
    out = []
    for x, y, yaw in track:
        out.append((c * x - s * y + dx, s * x + c * y + dy, yaw + dtheta))
    return out


class FeedbackProbe:
    """Minimal stand-in for feedback.Feedback (name + update_value)."""

    def __init__(self, name):
        self.name = name
        self.value = None

    def update_value(self, value):
        self.value = value


def slam_config():
    return NS(oracle_mode="slam_toolbox_jazzy")


def feedback_probes():
    return [
        FeedbackProbe("slam_oracle_valid"),
        FeedbackProbe("slam_collision_detected"),
        FeedbackProbe("slam_max_rpe_drift"),
        FeedbackProbe("slam_ate_rmse"),
        FeedbackProbe("slam_max_yaw_drift"),
        FeedbackProbe("slam_pose_sample_count"),
    ]


def probe_value(probes, name):
    for p in probes:
        if p.name == name:
            return p.value
    return None


class TestSlamOracleTimeSemantics(unittest.TestCase):

    def test_pose_track_prefers_nonzero_header_stamp_over_bag_time(self):
        msg = with_header_stamp(make_pose_cov(), 12, 345_000_000)

        track = slam._pose_track([(START_NS, msg)])

        self.assertEqual(len(track), 1)
        self.assertAlmostEqual(track[0][0], 12.345)

    def test_pose_track_missing_header_falls_back_to_exact_bag_time(self):
        bag_timestamp = START_NS + 125_000_000

        track = slam._pose_track([(bag_timestamp, make_pose_cov())])

        self.assertEqual(len(track), 1)
        self.assertEqual(track[0][0], slam._ts_to_sec(bag_timestamp))

    def test_pose_track_zero_header_stamp_falls_back_to_bag_time(self):
        bag_timestamp = START_NS + 250_000_000
        msg = with_header_stamp(make_pose_cov(), 0, 0)

        track = slam._pose_track([(bag_timestamp, msg)])

        self.assertEqual(len(track), 1)
        self.assertAlmostEqual(
            track[0][0], slam._ts_to_sec(bag_timestamp)
        )

    def test_pose_track_invalid_header_stamp_falls_back_to_bag_time(self):
        bag_timestamp = START_NS + 500_000_000
        msg = with_header_stamp(make_pose_cov(), 12, 1_000_000_000)

        track = slam._pose_track([(bag_timestamp, msg)])

        self.assertEqual(len(track), 1)
        self.assertAlmostEqual(
            track[0][0], slam._ts_to_sec(bag_timestamp)
        )

    def test_match_nearest_rejects_estimate_outside_time_tolerance(self):
        est_track = [(10.0, 0.0, 0.0, 0.0)]
        truth_track = [(10.251, 0.0, 0.0, 0.0)]

        pairs, matched_times = slam._match_nearest(est_track, truth_track)

        self.assertEqual(pairs, [])
        self.assertEqual(matched_times, [])

    def test_match_nearest_accepts_exact_time_tolerance_boundary(self):
        est_track = [(10.0, 1.0, 2.0, 0.3)]
        truth_track = [
            (10.0 + slam.SLAM_MATCH_MAX_DT_SEC, 4.0, 5.0, 0.6)
        ]

        pairs, matched_times = slam._match_nearest(est_track, truth_track)

        self.assertEqual(
            pairs, [(1.0, 2.0, 0.3, 4.0, 5.0, 0.6)]
        )
        self.assertEqual(matched_times, [10.0])

    def test_match_nearest_returns_empty_when_either_track_is_empty(self):
        estimate = [(10.0, 1.0, 2.0, 0.3)]
        truth = [(10.0, 4.0, 5.0, 0.6)]

        for est_track, truth_track in (([], truth), (estimate, [])):
            with self.subTest(
                estimate_count=len(est_track), truth_count=len(truth_track)
            ):
                try:
                    result = slam._match_nearest(est_track, truth_track)
                except Exception as exc:  # convert the old crash into RED
                    self.fail(
                        "empty trajectory must return empty matches, not "
                        f"raise {exc!r}"
                    )
                self.assertEqual(result, ([], []))

    def test_match_nearest_uses_next_unused_truth_within_tolerance(self):
        est_track = [
            (10.049, 1.0, 0.0, 0.0),
            (10.049, 2.0, 0.0, 0.0),
        ]
        truth_track = [
            (10.0, 100.0, 0.0, 0.0),
            (10.1, 200.0, 0.0, 0.0),
        ]

        pairs, matched_times = slam._match_nearest(est_track, truth_track)

        self.assertEqual([pair[3] for pair in pairs], [100.0, 200.0])
        self.assertEqual(matched_times, [10.049, 10.049])

    def test_non_overlapping_header_timelines_are_not_judged(self):
        truth = straight_truth(30)
        state = {
            "/pose": track_to_header_msgs(truth, header_start_sec=10),
            "/sim_ground_truth_pose": track_to_header_msgs(
                truth, header_start_sec=20
            ),
        }
        probes = feedback_probes()

        errs = slam.check(slam_config(), [], state, probes)

        self.assertEqual(errs, [])
        self.assertEqual(
            probe_value(probes, "slam_pose_sample_count"), 0.0
        )
        self.assertEqual(probe_value(probes, "slam_oracle_valid"), 0.0)

    def test_check_counts_only_ten_time_aligned_pairs_as_valid(self):
        truth = straight_truth(10)
        truth_records = track_to_header_msgs(truth, header_start_sec=10)
        estimate_records = track_to_header_msgs(truth, header_start_sec=10)

        # These bag times are only 0.1 s and 0.2 s beyond the last truth bag
        # sample, but their ROS header times are far away. Header-time matching
        # must therefore reject exactly these two estimates.
        for i, header_sec in enumerate((20, 21)):
            msg = with_header_stamp(make_pose_cov(0.5, 0.0, 0.0),
                                    header_sec, 0)
            estimate_records.append(
                (START_NS + (10 + i) * STEP_NS, msg)
            )

        probes = feedback_probes()
        errs = slam.check(
            slam_config(),
            [],
            {
                "/pose": estimate_records,
                "/sim_ground_truth_pose": truth_records,
            },
            probes,
        )

        self.assertEqual(errs, [])
        self.assertEqual(
            probe_value(probes, "slam_pose_sample_count"), 10.0
        )
        self.assertEqual(probe_value(probes, "slam_oracle_valid"), 1.0)

    def test_check_does_not_reuse_one_truth_for_ten_estimates(self):
        truth_msg = with_header_stamp(make_pose_cov(), 10, 0)
        truth_records = [(START_NS, truth_msg)]
        estimate_records = []
        header_start_ns = 9_800_000_000
        for i in range(10):
            total_ns = header_start_ns + i * 40_000_000
            sec, nanosec = divmod(total_ns, 1_000_000_000)
            msg = with_header_stamp(make_pose_cov(), sec, nanosec)
            estimate_records.append((START_NS + i * STEP_NS, msg))

        probes = feedback_probes()
        errs = slam.check(
            slam_config(),
            [],
            {
                "/pose": estimate_records,
                "/sim_ground_truth_pose": truth_records,
            },
            probes,
        )

        self.assertEqual(errs, [])
        self.assertEqual(
            probe_value(probes, "slam_pose_sample_count"), 1.0
        )
        self.assertEqual(probe_value(probes, "slam_oracle_valid"), 0.0)


class TestSlamOraclePositive(unittest.TestCase):
    """SLAM estimate tracks truth (up to a rigid frame offset) -> no bug."""

    def test_perfect_same_frame_no_error(self):
        truth = straight_truth(30)
        state = {
            "/pose": track_to_msgs(truth),
            "/sim_ground_truth_pose": track_to_msgs(truth),
        }
        errs = slam.check(slam_config(), [], state, feedback_probes())
        self.assertEqual(errs, [])

    def test_rigid_frame_offset_no_error(self):
        # SLAM map frame differs from world frame by a fixed rigid transform.
        truth = straight_truth(30)
        est = apply_rigid(truth, dx=3.0, dy=-2.0, dtheta=0.4)
        state = {
            "/pose": track_to_msgs(est),
            "/sim_ground_truth_pose": track_to_msgs(truth),
        }
        probes = feedback_probes()
        errs = slam.check(slam_config(), [], state, probes)
        self.assertEqual(errs, [])
        # RPE / yaw drift are frame-invariant -> ~0 under a rigid offset.
        self.assertLess(probe_value(probes, "slam_max_rpe_drift"), 1e-6)
        self.assertLess(probe_value(probes, "slam_max_yaw_drift"), 1e-6)


class TestSlamOracleDrift(unittest.TestCase):

    def test_rpe_translation_drift_detected(self):
        truth = straight_truth(30)
        jump = slam.SLAM_RPE_DRIFT_ERROR * 2.0 + 0.5
        est = [
            (x + (jump if i >= 15 else 0.0), y, yaw)
            for i, (x, y, yaw) in enumerate(truth)
        ]
        state = {
            "/pose": track_to_msgs(est),
            "/sim_ground_truth_pose": track_to_msgs(truth),
        }
        probes = feedback_probes()
        errs = slam.check(slam_config(), [], state, probes)
        self.assertTrue(any("RPE" in e or "relative" in e.lower() for e in errs),
                        msg=f"expected RPE drift error, got {errs}")
        self.assertGreater(probe_value(probes, "slam_max_rpe_drift"),
                           slam.SLAM_RPE_DRIFT_ERROR)

    def test_yaw_drift_detected(self):
        truth = straight_truth(30)
        yaw_jump = slam.SLAM_YAW_DRIFT_ERROR * 2.0
        est = [
            (x, y, yaw + (yaw_jump if i >= 15 else 0.0))
            for i, (x, y, yaw) in enumerate(truth)
        ]
        state = {
            "/pose": track_to_msgs(est),
            "/sim_ground_truth_pose": track_to_msgs(truth),
        }
        errs = slam.check(slam_config(), [], state, feedback_probes())
        self.assertTrue(any("yaw" in e.lower() or "heading" in e.lower()
                            for e in errs),
                        msg=f"expected yaw drift error, got {errs}")

    def test_ate_global_drift_detected(self):
        # Truth on a small circle (2D extent); SLAM adds a large growing
        # translation the rigid alignment cannot absorb.
        truth = [(math.cos(0.2 * i), math.sin(0.2 * i), 0.2 * i)
                 for i in range(30)]
        ramp = slam.SLAM_ATE_RMSE_ERROR * 20.0 / 30.0
        est = [(x + ramp * i, y, yaw) for i, (x, y, yaw) in enumerate(truth)]
        state = {
            "/pose": track_to_msgs(est),
            "/sim_ground_truth_pose": track_to_msgs(truth),
        }
        probes = feedback_probes()
        errs = slam.check(slam_config(), [], state, probes)
        self.assertTrue(any("ATE" in e or "trajectory" in e.lower()
                            for e in errs),
                        msg=f"expected ATE drift error, got {errs}")
        self.assertGreater(probe_value(probes, "slam_ate_rmse"),
                           slam.SLAM_ATE_RMSE_ERROR)


class TestSlamOracleRobustness(unittest.TestCase):

    def test_empty_state_clears_all_previous_round_feedback(self):
        truth = straight_truth(30)
        estimate = [
            (
                x * 1.01,
                0.01 * math.sin(0.4 * i),
                0.002 * i,
            )
            for i, (x, _y, _yaw) in enumerate(truth)
        ]
        probes = feedback_probes()
        first_errs = slam.check(
            slam_config(),
            [],
            {
                "/pose": track_to_msgs(estimate),
                "/sim_ground_truth_pose": track_to_msgs(truth),
            },
            probes,
        )

        self.assertEqual(first_errs, [])
        self.assertEqual(probe_value(probes, "slam_oracle_valid"), 1.0)
        for name in (
            "slam_pose_sample_count",
            "slam_max_rpe_drift",
            "slam_ate_rmse",
            "slam_max_yaw_drift",
        ):
            self.assertGreater(
                probe_value(probes, name), 0.0,
                msg=f"first valid round must populate non-zero {name}",
            )

        second_errs = slam.check(slam_config(), [], {}, probes)

        self.assertEqual(second_errs, [])
        for name in (
            "slam_oracle_valid",
            "slam_collision_detected",
            "slam_pose_sample_count",
            "slam_max_rpe_drift",
            "slam_ate_rmse",
            "slam_max_yaw_drift",
        ):
            self.assertEqual(
                probe_value(probes, name), 0.0,
                msg=f"empty round must reset stale {name}",
            )

    def test_validity_becomes_one_after_all_drift_metrics_succeed(self):
        truth = straight_truth(30)
        probes = feedback_probes()

        def assert_invalid_during_last_metric(_pairs):
            self.assertEqual(
                probe_value(probes, "slam_oracle_valid"), 0.0
            )
            return 0.0

        with patch.object(
            slam,
            "_yaw_drift_spread",
            side_effect=assert_invalid_during_last_metric,
        ) as yaw_drift:
            errs = slam.check(
                slam_config(), [], track_state(truth), probes
            )

        self.assertEqual(errs, [])
        yaw_drift.assert_called_once()
        self.assertEqual(probe_value(probes, "slam_oracle_valid"), 1.0)

    def test_drift_metric_exception_leaves_validity_zero(self):
        truth = straight_truth(30)
        probes = feedback_probes()

        with patch.object(
            slam, "_ate_rmse", side_effect=RuntimeError("metric failed")
        ):
            with self.assertRaisesRegex(RuntimeError, "metric failed"):
                slam.check(
                    slam_config(), [], track_state(truth), probes
                )

        self.assertEqual(probe_value(probes, "slam_oracle_valid"), 0.0)

    def test_missing_ground_truth_not_bug(self):
        state = {"/pose": track_to_msgs(straight_truth(30))}
        probes = feedback_probes()
        errs = slam.check(slam_config(), [], state, probes)
        self.assertEqual(errs, [])
        self.assertEqual(
            probe_value(probes, "slam_collision_detected"), 0.0)
        self.assertEqual(probe_value(probes, "slam_oracle_valid"), 0.0)

    def test_missing_estimate_not_bug(self):
        state = {"/sim_ground_truth_pose": track_to_msgs(straight_truth(30))}
        probes = feedback_probes()
        errs = slam.check(slam_config(), [], state, probes)
        self.assertEqual(errs, [])
        self.assertEqual(probe_value(probes, "slam_oracle_valid"), 0.0)

    def test_insufficient_samples_no_error(self):
        truth = straight_truth(4)
        state = {
            "/pose": track_to_msgs(truth),
            "/sim_ground_truth_pose": track_to_msgs(truth),
        }
        probes = feedback_probes()
        errs = slam.check(slam_config(), [], state, probes)
        self.assertEqual(errs, [])
        self.assertEqual(
            probe_value(probes, "slam_collision_detected"), 0.0)
        self.assertEqual(probe_value(probes, "slam_oracle_valid"), 0.0)

    def test_pose_stamped_shape_supported(self):
        # Estimate published as PoseStamped (.pose.*) must also be parsed.
        truth = straight_truth(30)
        state = {
            "/pose": track_to_msgs(truth, maker=make_pose_stamped),
            "/sim_ground_truth_pose": track_to_msgs(truth),
        }
        errs = slam.check(slam_config(), [], state, feedback_probes())
        self.assertEqual(errs, [])

    def test_feedback_populated_on_normal_run(self):
        truth = straight_truth(30)
        probes = feedback_probes()
        slam.check(slam_config(), [], track_state(truth), probes)
        self.assertIsNotNone(probe_value(probes, "slam_max_rpe_drift"))
        self.assertIsNotNone(probe_value(probes, "slam_ate_rmse"))
        self.assertIsNotNone(probe_value(probes, "slam_max_yaw_drift"))
        self.assertGreaterEqual(
            probe_value(probes, "slam_pose_sample_count"), 10)
        self.assertEqual(probe_value(probes, "slam_oracle_valid"), 1.0)


def track_state(truth):
    return {
        "/pose": track_to_msgs(truth),
        "/sim_ground_truth_pose": track_to_msgs(truth),
    }


def _contacts_msg(n_contacts):
    # Duck-typed ros_gz_interfaces/msg/Contacts: only `.contacts` (a list)
    # is read by the oracle. ros_gz_interfaces is not importable in the base
    # test env, so we synthesize the shape.
    from types import SimpleNamespace as NS
    return NS(contacts=[NS() for _ in range(n_contacts)])


def _drifting_state():
    # Same construction as test_rpe_translation_drift_detected: an estimate
    # that jumps mid-run so RPE would fire without collision suppression.
    truth = straight_truth(30)
    jump = slam.SLAM_RPE_DRIFT_ERROR * 2.0 + 0.5
    est = [
        (x + (jump if i >= 15 else 0.0), y, yaw)
        for i, (x, y, yaw) in enumerate(truth)
    ]
    return {
        "/pose": track_to_msgs(est),
        "/sim_ground_truth_pose": track_to_msgs(truth),
    }


class TestSlamOracleCollision(unittest.TestCase):

    def test_collision_feedback_marks_collision_round(self):
        state = _drifting_state()
        state["/bumper_contact"] = [(START_NS, _contacts_msg(1))]
        probes = feedback_probes()

        slam.check(slam_config(), [], state, probes)

        self.assertEqual(
            probe_value(probes, "slam_collision_detected"), 1.0)
        self.assertEqual(probe_value(probes, "slam_oracle_valid"), 0.0)

    def test_collision_feedback_marks_clean_round(self):
        probes = feedback_probes()

        slam.check(slam_config(), [], track_state(straight_truth(30)), probes)

        self.assertEqual(
            probe_value(probes, "slam_collision_detected"), 0.0)

    def test_collision_feedback_does_not_stick_across_rounds(self):
        collision_state = _drifting_state()
        collision_state["/bumper_contact"] = [
            (START_NS, _contacts_msg(1))
        ]
        probes = feedback_probes()
        slam.check(slam_config(), [], collision_state, probes)
        self.assertEqual(
            probe_value(probes, "slam_collision_detected"), 1.0)

        slam.check(slam_config(), [], track_state(straight_truth(30)), probes)

        self.assertEqual(
            probe_value(probes, "slam_collision_detected"), 0.0)

    def test_collision_suppresses_drift_error(self):
        state = _drifting_state()
        state["/bumper_contact"] = [(START_NS, _contacts_msg(1))]
        errs = slam.check(slam_config(), [], state, feedback_probes())
        self.assertEqual(errs, [],
                         msg=f"collision round should report no drift, got {errs}")

    def test_no_contact_points_still_judges(self):
        state = _drifting_state()
        state["/bumper_contact"] = [(START_NS, _contacts_msg(0))]
        errs = slam.check(slam_config(), [], state, feedback_probes())
        self.assertTrue(any("RPE" in e for e in errs),
                        msg=f"empty-contacts round should still judge, got {errs}")

    def test_missing_bumper_topic_backward_compatible(self):
        state = _drifting_state()
        errs = slam.check(slam_config(), [], state, feedback_probes())
        self.assertTrue(any("RPE" in e for e in errs),
                        msg=f"missing bumper topic must stay backward-compatible, got {errs}")

    def test_collision_still_records_sample_count(self):
        state = _drifting_state()
        state["/bumper_contact"] = [(START_NS, _contacts_msg(2))]
        probes = feedback_probes()
        slam.check(slam_config(), [], state, probes)
        self.assertGreaterEqual(probe_value(probes, "slam_pose_sample_count"), 10)


if __name__ == "__main__":
    unittest.main()
