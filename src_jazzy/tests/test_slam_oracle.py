import math
import os
import sys
import unittest
from types import SimpleNamespace as NS


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


START_NS = 1_000_000_000_000_000_000  # 16-digit-trim -> 1e9 sec
STEP_NS = 100_000_000                  # 0.1 s -> 10 Hz


def track_to_msgs(track, maker=make_pose_cov, start_ns=START_NS,
                  step_ns=STEP_NS):
    """track: list of (x, y, yaw) -> list of (ts_ns, msg)."""
    return [
        (start_ns + i * step_ns, maker(x, y, yaw))
        for i, (x, y, yaw) in enumerate(track)
    ]


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

    def test_missing_ground_truth_not_bug(self):
        state = {"/pose": track_to_msgs(straight_truth(30))}
        errs = slam.check(slam_config(), [], state, feedback_probes())
        self.assertEqual(errs, [])

    def test_missing_estimate_not_bug(self):
        state = {"/sim_ground_truth_pose": track_to_msgs(straight_truth(30))}
        errs = slam.check(slam_config(), [], state, feedback_probes())
        self.assertEqual(errs, [])

    def test_insufficient_samples_no_error(self):
        truth = straight_truth(4)
        state = {
            "/pose": track_to_msgs(truth),
            "/sim_ground_truth_pose": track_to_msgs(truth),
        }
        errs = slam.check(slam_config(), [], state, feedback_probes())
        self.assertEqual(errs, [])

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


def track_state(truth):
    return {
        "/pose": track_to_msgs(truth),
        "/sim_ground_truth_pose": track_to_msgs(truth),
    }


if __name__ == "__main__":
    unittest.main()
