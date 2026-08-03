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


def make_odom(x=0.0, y=0.0, yaw=0.0):
    msg = make_pose_cov(x, y, yaw)
    msg.child_frame_id = "base_link"
    return msg


def with_header_stamp(msg, sec, nanosec):
    msg.header = NS(stamp=NS(sec=sec, nanosec=nanosec))
    return msg


def with_header(msg, sec=1, nanosec=0, frame_id=""):
    msg.header = NS(
        stamp=NS(sec=sec, nanosec=nanosec),
        frame_id=frame_id,
    )
    return msg


def make_scan(ranges=None, frame_id="rplidar_link"):
    return with_header(
        NS(
            angle_min=-1.0,
            angle_max=1.0,
            angle_increment=1.0,
            range_min=0.1,
            range_max=12.0,
            ranges=list(ranges if ranges is not None else [1.0, 2.0, 3.0]),
        ),
        frame_id=frame_id,
    )


def make_map(data=None, width=2, height=2, resolution=0.05,
             frame_id="map"):
    return with_header(
        NS(
            info=NS(
                resolution=resolution,
                width=width,
                height=height,
                origin=NS(
                    position=_vec(),
                    orientation=_quat_from_yaw(0.0),
                ),
            ),
            data=list(data if data is not None else [-1, 0, 100, 0]),
        ),
        frame_id=frame_id,
    )


def make_tf(parent="map", child="odom", translation=None,
            rotation=None):
    transform = NS(
        header=NS(stamp=NS(sec=1, nanosec=0), frame_id=parent),
        child_frame_id=child,
        transform=NS(
            translation=translation if translation is not None else _vec(),
            rotation=(
                rotation if rotation is not None else _quat_from_yaw(0.0)
            ),
        ),
    )
    return NS(transforms=[transform])


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
        FeedbackProbe("slam_match_ratio"),
        FeedbackProbe("slam_displacement_magnitude_error"),
        FeedbackProbe("slam_rpe_translation_rmse"),
        FeedbackProbe("slam_rpe_rotation_max"),
        FeedbackProbe("slam_rpe_rotation_rmse"),
        FeedbackProbe("slam_ate_max"),
        FeedbackProbe("slam_ate_median"),
        FeedbackProbe("slam_odom_rpe_drift"),
        FeedbackProbe("slam_pose_odom_excess_rpe"),
        FeedbackProbe("slam_translation_excitation"),
        FeedbackProbe("slam_rotation_excitation"),
        FeedbackProbe("slam_scan_finite_ratio"),
        FeedbackProbe("slam_scan_gate_opportunities"),
        FeedbackProbe("slam_map_known_area"),
        FeedbackProbe("slam_map_growth"),
        FeedbackProbe("slam_revisit_count"),
        FeedbackProbe("slam_loop_closure_residual"),
        FeedbackProbe("slam_tf_coverage"),
        FeedbackProbe("slam_tf_max_age"),
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

    def test_hard_error_thresholds_match_clean_campaign_calibration(self):
        # Widened from the original ~10% headroom after the 1000-round campaign
        # (slam_calibrated_long_1000_v3) showed ATE/RPE firing 0.02-0.6 mm over
        # the old limits — indistinguishable from clean-baseline jitter. RPE/ATE
        # widened to ~1.35x/1.5x the clean maxima; YAW kept tight (real alarms
        # sit far above it). See oracles/slam.py calibration comment.
        self.assertAlmostEqual(slam.SLAM_RPE_DRIFT_ERROR, 0.023)
        self.assertAlmostEqual(slam.SLAM_ATE_RMSE_ERROR, 0.009)
        self.assertAlmostEqual(slam.SLAM_YAW_DRIFT_ERROR, 0.023)
        # thresholds must stay above the observed clean-baseline maxima
        self.assertGreater(slam.SLAM_RPE_DRIFT_ERROR, 0.017)
        self.assertGreater(slam.SLAM_ATE_RMSE_ERROR, 0.006)
        self.assertGreater(slam.SLAM_YAW_DRIFT_ERROR, 0.021)

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

    def test_se2_rpe_detects_direction_error_hidden_by_displacement_length(self):
        # Both trajectories travel equally far in every segment, so the old
        # abs(||delta_est|| - ||delta_truth||) proxy reports zero.  After the
        # corner, however, the estimate turns the opposite way; no single
        # rigid map/world frame offset can explain both halves.
        truth = []
        estimate = []
        for i in range(15):
            truth.append((0.1 * i, 0.0, 0.0))
            estimate.append((0.1 * i, 0.0, 0.0))
        for i in range(1, 16):
            truth.append((1.4, 0.1 * i, 0.0))
            estimate.append((1.4, -0.1 * i, 0.0))

        pairs = [
            (ex, ey, eyaw, tx, ty, tyaw)
            for (ex, ey, eyaw), (tx, ty, tyaw)
            in zip(estimate, truth)
        ]

        metrics = slam._rpe_metrics(pairs, window=10)

        self.assertGreater(metrics["translation_max"], 0.5)
        self.assertAlmostEqual(
            slam._max_displacement_magnitude_error(pairs, window=10),
            0.0,
            places=9,
        )

    def test_yaw_drift_uses_circular_center_across_pi_boundary(self):
        near_pi = math.pi - math.radians(1.0)
        pairs = [
            (0.0, 0.0, near_pi, 0.0, 0.0, 0.0),
            (0.0, 0.0, -near_pi, 0.0, 0.0, 0.0),
        ]

        drift = slam._yaw_drift_spread(pairs)

        self.assertLess(drift, math.radians(3.0))

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

    def test_good_odom_baseline_keeps_pose_drift_attributed_to_slam(self):
        truth = straight_truth(30)
        jump = slam.SLAM_RPE_DRIFT_ERROR * 2.0 + 0.5
        estimate = [
            (x + (jump if i >= 15 else 0.0), y, yaw)
            for i, (x, y, yaw) in enumerate(truth)
        ]
        state = {
            "/pose": track_to_msgs(estimate),
            "/odom": track_to_msgs(truth, maker=make_odom),
            "/sim_ground_truth_pose": track_to_msgs(truth),
        }

        errs = slam.check(slam_config(), [], state, feedback_probes())

        self.assertTrue(
            any(error.startswith("SLAM_RPE_ERROR:pose") for error in errs),
            msg=f"good odom baseline must retain SLAM attribution, got {errs}",
        )

    def test_normal_odom_noise_does_not_use_calibrated_slam_error_limit(self):
        truth = straight_truth(30)
        estimate = [
            (x + (0.10 if i >= 15 else 0.0), y, yaw)
            for i, (x, y, yaw) in enumerate(truth)
        ]
        odom = [
            (x + (0.05 if i >= 15 else 0.0), y, yaw)
            for i, (x, y, yaw) in enumerate(truth)
        ]
        state = {
            "/pose": track_to_msgs(estimate),
            "/odom": track_to_msgs(odom, maker=make_odom),
            "/sim_ground_truth_pose": track_to_msgs(truth),
        }

        errs = slam.check(slam_config(), [], state, feedback_probes())

        self.assertTrue(
            any(error.startswith("SLAM_RPE_ERROR:pose") for error in errs),
            msg=f"ordinary odom noise must not suppress SLAM errors: {errs}",
        )

    def test_bad_odom_baseline_suppresses_slam_specific_drift_attribution(self):
        truth = straight_truth(30)
        jump = slam.SLAM_RPE_DRIFT_ERROR * 2.0 + 0.5
        contaminated = [
            (x + (jump if i >= 15 else 0.0), y, yaw)
            for i, (x, y, yaw) in enumerate(truth)
        ]
        state = {
            "/pose": track_to_msgs(contaminated),
            "/odom": track_to_msgs(contaminated, maker=make_odom),
            "/sim_ground_truth_pose": track_to_msgs(truth),
        }

        errs = slam.check(slam_config(), [], state, feedback_probes())

        self.assertEqual(
            errs,
            [],
            msg=f"shared odom/pose contamination is not a SLAM-only bug: {errs}",
        )


class TestSlamOracleRobustness(unittest.TestCase):

    def test_nonfinite_pose_position_is_reported_not_silently_dropped(self):
        msg = make_pose_cov(float("nan"), 0.0, 0.0)

        errs = slam.check(
            slam_config(), [], {"/pose": [(START_NS, msg)]},
            feedback_probes(),
        )

        self.assertTrue(
            any(error.startswith("SLAM_NUMERIC_NONFINITE:pose.position")
                for error in errs),
            msg=f"non-finite pose must be explicit, got {errs}",
        )

    def test_pose_frame_mismatch_is_reported_when_header_frame_is_present(self):
        msg = with_header(make_pose_cov(), frame_id="odom")

        errs = slam.check(
            slam_config(), [], {"/pose": [(START_NS, msg)]},
            feedback_probes(),
        )

        self.assertTrue(
            any(error.startswith("SLAM_FRAME_MISMATCH:pose")
                for error in errs),
            msg=f"/pose must be in map when declared, got {errs}",
        )

    def test_pose_header_timestamp_regression_is_reported(self):
        first = with_header(make_pose_cov(), sec=2, frame_id="map")
        second = with_header(make_pose_cov(), sec=1, frame_id="map")

        errs = slam.check(
            slam_config(), [],
            {"/pose": [(START_NS, first), (START_NS + STEP_NS, second)]},
            feedback_probes(),
        )

        self.assertTrue(
            any(error.startswith("SLAM_TIMESTAMP_REGRESSION:pose")
                for error in errs),
            msg=f"header-time regression must be explicit, got {errs}",
        )

    def test_empty_state_clears_all_previous_round_feedback(self):
        truth = straight_truth(30)
        estimate = [
            (
                x * 1.001,
                0.001 * math.sin(0.4 * i),
                0.0002 * i,
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

    def test_collision_does_not_hide_invalid_pose_quaternion(self):
        state = track_state(straight_truth(30))
        for _timestamp, msg in state["/pose"]:
            msg.pose.pose.orientation = NS(x=0.0, y=0.0, z=0.0, w=0.0)
        state["/bumper_contact"] = [(START_NS, _contacts_msg(1))]

        errs = slam.check(slam_config(), [], state, feedback_probes())

        self.assertTrue(
            any(error.startswith("SLAM_INVALID_QUATERNION:pose")
                for error in errs),
            msg=f"collision must not hide invalid output, got {errs}",
        )

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


class TestSlamStructuralOracles(unittest.TestCase):

    def test_scan_positive_infinity_is_a_valid_no_return(self):
        state = {"/scan": [(START_NS, make_scan([float("inf"), 1.0, 2.0]))]}

        errs = slam.check(slam_config(), [], state, feedback_probes())

        self.assertFalse(
            any(error.startswith("SLAM_SCAN_") for error in errs),
            msg=f"+inf LaserScan return is valid, got {errs}",
        )

    def test_scan_nan_is_reported(self):
        state = {"/scan": [(START_NS, make_scan([float("nan"), 1.0, 2.0]))]}

        errs = slam.check(slam_config(), [], state, feedback_probes())

        self.assertTrue(
            any(error.startswith("SLAM_SCAN_INVALID_RANGE:scan")
                for error in errs),
            msg=f"NaN range must be rejected, got {errs}",
        )

    def test_scan_finite_range_outside_declared_bounds_is_reported(self):
        state = {"/scan": [(START_NS, make_scan([0.05, 1.0, 13.0]))]}

        errs = slam.check(slam_config(), [], state, feedback_probes())

        self.assertTrue(
            any(error.startswith("SLAM_SCAN_INVALID_RANGE:scan")
                for error in errs),
            msg=f"finite range outside [min,max] must be rejected, got {errs}",
        )

    def test_map_data_length_must_match_declared_grid(self):
        state = {"/map": [(START_NS, make_map(data=[-1, 0, 100]))]}

        errs = slam.check(slam_config(), [], state, feedback_probes())

        self.assertTrue(
            any(error.startswith("SLAM_MAP_SHAPE:map") for error in errs),
            msg=f"OccupancyGrid shape mismatch must be rejected, got {errs}",
        )

    def test_map_cell_must_follow_occupancy_grid_range(self):
        state = {"/map": [(START_NS, make_map(data=[-1, 0, 101, 0]))]}

        errs = slam.check(slam_config(), [], state, feedback_probes())

        self.assertTrue(
            any(error.startswith("SLAM_MAP_INVALID_CELL:map")
                for error in errs),
            msg=f"OccupancyGrid cells must stay in [-1,100], got {errs}",
        )

    def test_map_resolution_must_match_slam_profile(self):
        state = {"/map": [(START_NS, make_map(resolution=0.1))]}

        errs = slam.check(slam_config(), [], state, feedback_probes())

        self.assertTrue(
            any(error.startswith("SLAM_MAP_RESOLUTION:map")
                for error in errs),
            msg=f"unexpected map resolution must be reported, got {errs}",
        )

    def test_map_declared_frame_must_be_map(self):
        state = {"/map": [(START_NS, make_map(frame_id="odom"))]}

        errs = slam.check(slam_config(), [], state, feedback_probes())

        self.assertTrue(
            any(error.startswith("SLAM_FRAME_MISMATCH:map")
                for error in errs),
            msg=f"map output frame must be map, got {errs}",
        )

    def test_missing_tf_is_insufficient_not_an_error(self):
        errs = slam.check(slam_config(), [], {}, feedback_probes())

        self.assertFalse(
            any(error.startswith("SLAM_TF_") for error in errs),
            msg=f"optional TF absence must not be an error, got {errs}",
        )

    def test_tf_self_edge_is_reported_when_tf_is_observed(self):
        state = {"/tf": [(START_NS, make_tf(parent="map", child="map"))]}

        errs = slam.check(slam_config(), [], state, feedback_probes())

        self.assertTrue(
            any(error.startswith("SLAM_TF_SELF_EDGE:tf") for error in errs),
            msg=f"observed TF self-edge must be rejected, got {errs}",
        )

    def test_tf_invalid_quaternion_is_reported(self):
        invalid = NS(x=0.0, y=0.0, z=0.0, w=0.0)
        state = {"/tf": [(START_NS, make_tf(rotation=invalid))]}

        errs = slam.check(slam_config(), [], state, feedback_probes())

        self.assertTrue(
            any(error.startswith("SLAM_TF_INVALID_QUATERNION:tf")
                for error in errs),
            msg=f"observed invalid TF rotation must be rejected, got {errs}",
        )

    def test_valid_tf_transform_is_not_reported(self):
        state = {"/tf": [(START_NS, make_tf())]}

        errs = slam.check(slam_config(), [], state, feedback_probes())

        self.assertFalse(
            any(error.startswith("SLAM_TF_") for error in errs),
            msg=f"valid optional TF must not be rejected, got {errs}",
        )


class TestSlamSemanticFeedback(unittest.TestCase):

    def test_trajectory_feedback_exposes_full_rpe_and_odom_decomposition(self):
        truth = straight_truth(30)
        estimate = [
            (x * 1.02, 0.01 * math.sin(i * 0.3), 0.002 * i)
            for i, (x, _y, _yaw) in enumerate(truth)
        ]
        probes = feedback_probes()
        state = {
            "/pose": track_to_msgs(estimate),
            "/odom": track_to_msgs(truth, maker=make_odom),
            "/sim_ground_truth_pose": track_to_msgs(truth),
        }

        slam.check(slam_config(), [], state, probes)

        self.assertGreater(probe_value(probes, "slam_match_ratio"), 0.9)
        self.assertGreater(
            probe_value(probes, "slam_rpe_translation_rmse"), 0.0
        )
        self.assertGreater(
            probe_value(probes, "slam_rpe_rotation_max"), 0.0
        )
        self.assertLess(
            probe_value(probes, "slam_odom_rpe_drift"), 1e-9
        )
        self.assertGreater(
            probe_value(probes, "slam_pose_odom_excess_rpe"), 0.0
        )
        self.assertGreater(
            probe_value(probes, "slam_translation_excitation"), 1.0
        )

    def test_scan_feedback_counts_finite_returns_and_motion_gate_opportunities(self):
        truth = straight_truth(30, step=0.05)
        probes = feedback_probes()
        state = {
            "/sim_ground_truth_pose": track_to_msgs(truth),
            "/scan": [
                (START_NS, make_scan([float("inf"), 1.0, 2.0])),
                (START_NS + STEP_NS, make_scan([3.0, 4.0, float("inf")])),
            ],
        }

        slam.check(slam_config(), [], state, probes)

        self.assertAlmostEqual(
            probe_value(probes, "slam_scan_finite_ratio"), 4.0 / 6.0
        )
        self.assertGreaterEqual(
            probe_value(probes, "slam_scan_gate_opportunities"), 10.0
        )

    def test_map_feedback_reports_known_area_and_growth(self):
        probes = feedback_probes()
        state = {
            "/map": [
                (START_NS, make_map(data=[-1, -1, -1, -1])),
                (START_NS + STEP_NS, make_map(data=[0, 0, 100, -1])),
            ]
        }

        slam.check(slam_config(), [], state, probes)

        cell_area = 0.05 * 0.05
        self.assertAlmostEqual(
            probe_value(probes, "slam_map_known_area"), 3.0 * cell_area
        )
        self.assertAlmostEqual(
            probe_value(probes, "slam_map_growth"), 3.0 * cell_area
        )

    def test_truth_revisit_produces_black_box_loop_feedback(self):
        truth = []
        for i in range(11):
            truth.append((0.1 * i, 0.0, 0.0))
        for i in range(1, 11):
            truth.append((1.0, 0.1 * i, math.pi / 2.0))
        for i in range(1, 11):
            truth.append((1.0 - 0.1 * i, 1.0, math.pi))
        for i in range(1, 11):
            truth.append((0.0, 1.0 - 0.1 * i, -math.pi / 2.0))
        estimate = list(truth)
        x, y, yaw = estimate[-1]
        estimate[-1] = (x + 0.2, y, yaw)
        probes = feedback_probes()
        state = {
            "/pose": track_to_msgs(estimate),
            "/sim_ground_truth_pose": track_to_msgs(truth),
        }

        slam.check(slam_config(), [], state, probes)

        self.assertGreaterEqual(
            probe_value(probes, "slam_revisit_count"), 1.0
        )
        self.assertGreater(
            probe_value(probes, "slam_loop_closure_residual"), 0.1
        )

    def test_tf_feedback_reports_complete_map_to_laser_chain(self):
        probes = feedback_probes()
        scan = make_scan()
        dynamic = [
            (START_NS, make_tf("map", "odom")),
            (START_NS, make_tf("odom", "base_link")),
        ]
        static = [
            (START_NS, make_tf("base_link", "rplidar_link")),
        ]
        state = {
            "/scan": [(START_NS, scan)],
            "/tf": dynamic,
            "/tf_static": static,
        }

        slam.check(slam_config(), [], state, probes)

        self.assertEqual(probe_value(probes, "slam_tf_coverage"), 1.0)


if __name__ == "__main__":
    unittest.main()
