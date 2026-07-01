"""SLAM semantic oracle for slam_toolbox on the TurtleBot4 Jazzy base.

This oracle checks a *correctness/semantic* property that crash-oriented
fuzzers (e.g. RoFER) cannot express: does the SLAM pose estimate stay
faithful to the simulator ground-truth trajectory?

The SLAM estimate lives in the ``map`` frame (anchored at the robot start
pose); the Gazebo ground truth lives in the world frame. The two differ by an
unknown but *fixed* rigid transform, so absolute poses cannot be compared
directly. Following standard SLAM evaluation (TUM/KITTI ATE/RPE) we use three
frame-aware criteria:

  * RPE  - relative pose error: over a sliding window, compare how far the
           estimate says the robot moved vs how far ground truth moved. Frame
           invariant (a rigid frame offset preserves displacement magnitudes).
  * ATE  - absolute trajectory error: rigidly align (Umeyama, no scale) the
           estimate positions onto ground truth, then RMSE of the residual.
  * yaw  - heading drift: the estimate-vs-truth yaw difference should be a
           constant frame offset; its spread over the run is the drift.

None of these depend on a crash. Thresholds are conservative placeholders
intended to be replaced by a baseline-calibrated value (see the design spec).
"""

import math

import numpy as np


# === Drift thresholds (baseline-calibratable placeholders) ===
# Design decision: final thresholds = clean-baseline max x safety factor.
# Until that calibration run is done, use conservative literature-informed
# values so the oracle is exercisable and unit-testable.
SLAM_RPE_DRIFT_ERROR = 0.5    # m, per-window relative displacement error
SLAM_ATE_RMSE_ERROR = 0.30    # m, aligned absolute trajectory RMSE
SLAM_YAW_DRIFT_ERROR = 0.35   # rad (~20 deg), heading-offset spread

# Sampling / windowing.
SLAM_MIN_SAMPLES = 10         # matched pairs needed before judging
SLAM_RPE_WINDOW_SEC = 1.0     # relative-motion comparison window

ESTIMATE_TOPIC = "/pose"
GROUND_TRUTH_TOPIC = "/sim_ground_truth_pose"


def _ts_to_sec(ts):
    """Convert a (possibly 16+ digit trimmed) bag timestamp to seconds."""
    ts_str = str(ts)
    if len(ts_str) <= 10:
        return float(ts)
    return int(ts_str[:10]) + int(ts_str[10:]) / (10 ** (len(ts_str) - 10))


def _yaw_from_quat(orientation):
    x = float(getattr(orientation, "x", 0.0))
    y = float(getattr(orientation, "y", 0.0))
    z = float(getattr(orientation, "z", 0.0))
    w = float(getattr(orientation, "w", 1.0))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _extract_pose(msg):
    """Return (x, y, yaw) from either a stamped-with-covariance / Odometry
    shape (``msg.pose.pose.*``) or a PoseStamped shape (``msg.pose.*``)."""
    pose = getattr(msg, "pose", None)
    if pose is None:
        return None
    inner = getattr(pose, "pose", None)
    if inner is not None and hasattr(inner, "position"):
        pose = inner  # PoseWithCovariance / Odometry
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    if position is None or orientation is None:
        return None
    try:
        x = float(position.x)
        y = float(position.y)
    except (AttributeError, TypeError, ValueError):
        return None
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    yaw = _yaw_from_quat(orientation)
    return x, y, yaw


def _pose_track(records):
    """records: list of (ts, msg) -> sorted list of (t_sec, x, y, yaw)."""
    track = []
    for ts, msg in records or []:
        pose = _extract_pose(msg)
        if pose is None:
            continue
        try:
            t = _ts_to_sec(ts)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(t):
            continue
        track.append((t, pose[0], pose[1], pose[2]))
    track.sort(key=lambda item: item[0])
    return track


def _match_nearest(est_track, truth_track):
    """Pair each estimate sample with the nearest-in-time truth sample.

    Returns list of (ex, ey, eyaw, tx, ty, tyaw) and the matched estimate
    timestamps (for window sizing)."""
    truth_times = [row[0] for row in truth_track]
    pairs = []
    matched_times = []
    for t, ex, ey, eyaw in est_track:
        idx = _nearest_index(truth_times, t)
        _, tx, ty, tyaw = truth_track[idx]
        pairs.append((ex, ey, eyaw, tx, ty, tyaw))
        matched_times.append(t)
    return pairs, matched_times


def _nearest_index(sorted_times, t):
    lo, hi = 0, len(sorted_times) - 1
    if t <= sorted_times[lo]:
        return lo
    if t >= sorted_times[hi]:
        return hi
    while lo < hi:
        mid = (lo + hi) // 2
        if sorted_times[mid] < t:
            lo = mid + 1
        else:
            hi = mid
    # lo is first index >= t; compare with previous
    if lo > 0 and abs(sorted_times[lo - 1] - t) <= abs(sorted_times[lo] - t):
        return lo - 1
    return lo


def _median_dt(times):
    if len(times) < 2:
        return 0.0
    deltas = [b - a for a, b in zip(times, times[1:]) if b - a > 0]
    if not deltas:
        return 0.0
    return float(np.median(deltas))


def _wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _max_rpe(pairs, window):
    """Max over windows of |est displacement - truth displacement|."""
    max_rpe = 0.0
    n = len(pairs)
    for i in range(n - window):
        ex0, ey0 = pairs[i][0], pairs[i][1]
        ex1, ey1 = pairs[i + window][0], pairs[i + window][1]
        tx0, ty0 = pairs[i][3], pairs[i][4]
        tx1, ty1 = pairs[i + window][3], pairs[i + window][4]
        est_disp = math.hypot(ex1 - ex0, ey1 - ey0)
        truth_disp = math.hypot(tx1 - tx0, ty1 - ty0)
        rpe = abs(est_disp - truth_disp)
        if rpe > max_rpe:
            max_rpe = rpe
    return max_rpe


def _ate_rmse(pairs):
    """Umeyama rigid alignment (no scale) of estimate onto truth; RMSE."""
    est = np.array([[p[0], p[1]] for p in pairs], dtype=float)
    truth = np.array([[p[3], p[4]] for p in pairs], dtype=float)
    mu_e = est.mean(axis=0)
    mu_t = truth.mean(axis=0)
    est_c = est - mu_e
    truth_c = truth - mu_t
    cov = est_c.T @ truth_c
    u, _s, vt = np.linalg.svd(cov)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    rot = vt.T @ np.diag([1.0, d]) @ u.T
    trans = mu_t - rot @ mu_e
    aligned = (rot @ est.T).T + trans
    residual = aligned - truth
    return float(np.sqrt((residual ** 2).sum(axis=1).mean()))


def _yaw_drift_spread(pairs):
    """Spread of the estimate-vs-truth yaw offset (constant offset cancels)."""
    diffs = [_wrap_to_pi(p[2] - p[5]) for p in pairs]
    center = float(np.median(diffs))
    rel = [_wrap_to_pi(d - center) for d in diffs]
    return max(rel) - min(rel)


def _update_feedback(feedback_list, name, value):
    for fbk in feedback_list or []:
        if fbk.name == name:
            fbk.update_value(value)
            return


def check(config, msg_list, state_dict, feedback_list):
    """SLAM semantic oracle entry point (see checker.run_checks dispatch)."""
    errs = []

    est_track = _pose_track(state_dict.get(ESTIMATE_TOPIC, []))
    truth_track = _pose_track(state_dict.get(GROUND_TRUTH_TOPIC, []))

    # Missing either signal is a readiness/recording-quality issue, not a bug.
    if not est_track or not truth_track:
        _update_feedback(feedback_list, "slam_pose_sample_count", 0.0)
        return errs

    pairs, matched_times = _match_nearest(est_track, truth_track)
    _update_feedback(feedback_list, "slam_pose_sample_count",
                     float(len(pairs)))

    if len(pairs) < SLAM_MIN_SAMPLES:
        _update_feedback(feedback_list, "slam_max_rpe_drift", 0.0)
        _update_feedback(feedback_list, "slam_ate_rmse", 0.0)
        _update_feedback(feedback_list, "slam_max_yaw_drift", 0.0)
        return errs

    dt = _median_dt(matched_times)
    window = max(1, int(round(SLAM_RPE_WINDOW_SEC / dt))) if dt > 0 else 1
    window = min(window, len(pairs) - 1)

    max_rpe = _max_rpe(pairs, window)
    ate_rmse = _ate_rmse(pairs)
    yaw_drift = _yaw_drift_spread(pairs)

    _update_feedback(feedback_list, "slam_max_rpe_drift", max_rpe)
    _update_feedback(feedback_list, "slam_ate_rmse", ate_rmse)
    _update_feedback(feedback_list, "slam_max_yaw_drift", yaw_drift)

    if max_rpe > SLAM_RPE_DRIFT_ERROR:
        errs.append(
            f"SLAM relative pose error (RPE) drift {max_rpe:.3f} m over "
            f"{window}-sample window exceeds {SLAM_RPE_DRIFT_ERROR} m: "
            f"estimated motion disagrees with ground-truth motion"
        )
    if ate_rmse > SLAM_ATE_RMSE_ERROR:
        errs.append(
            f"SLAM absolute trajectory error (ATE) RMSE {ate_rmse:.3f} m "
            f"exceeds {SLAM_ATE_RMSE_ERROR} m after rigid alignment: "
            f"estimated trajectory diverges from ground truth"
        )
    if yaw_drift > SLAM_YAW_DRIFT_ERROR:
        errs.append(
            f"SLAM heading (yaw) drift {yaw_drift:.3f} rad exceeds "
            f"{SLAM_YAW_DRIFT_ERROR} rad: estimated orientation offset is "
            f"not consistent with ground truth"
        )

    return errs
