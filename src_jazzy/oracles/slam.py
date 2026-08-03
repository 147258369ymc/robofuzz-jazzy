"""SLAM semantic oracle for slam_toolbox on the TurtleBot4 Jazzy base.

This oracle checks a *correctness/semantic* property that crash-oriented
fuzzers (e.g. RoFER) cannot express: does the SLAM pose estimate stay
faithful to the simulator ground-truth trajectory?

The SLAM estimate lives in the ``map`` frame (anchored at the robot start
pose); the Gazebo ground truth lives in the world frame. The two differ by an
unknown but *fixed* rigid transform, so absolute poses cannot be compared
directly. Following standard SLAM evaluation (TUM/KITTI ATE/RPE) we use three
frame-aware criteria:

  * RPE  - true SE(2) relative pose error: over a sliding window, compare the
           estimated and ground-truth relative transforms, including both
           planar translation direction and rotation. A fixed rigid frame
           offset cancels from the relative transform.
  * ATE  - absolute trajectory error: rigidly align (Umeyama, no scale) the
           estimate positions onto ground truth, then RMSE of the residual.
  * yaw  - heading drift: the estimate-vs-truth yaw difference should be a
           constant frame offset; its spread over the run is the drift.

None of these depend on a crash. Thresholds are calibrated from the version-
locked clean Jazzy campaign and must be recalibrated when the target, world,
command timing, or observation pipeline changes.
"""

import math

import numpy as np


# === Drift thresholds calibrated on the clean Jazzy campaign ===
# logs_jazzy/slam_clean_100_motion_pubfix observed rounded maxima of
# 0.017 m RPE, 0.006 m ATE RMSE, and 0.021 rad yaw drift.
#
# The original ~10% headroom proved too tight: on the 1000-round campaign
# (logs_jazzy/slam_calibrated_long_1000_v3) three ATE alarms recomputed to
# 0.006622 / 0.006659 / 0.006740 m -- i.e. 0.02-0.14 mm over the 0.0066 limit,
# indistinguishable from clean-baseline numerical variation (false positives).
# RPE likewise fired at exactly 0.019. Widen ATE/RPE to ~1.3-1.5x the clean
# maximum so ordinary baseline jitter stays quiet while genuine drift still
# crosses. YAW stays tight: its real alarms (0.034, +62% over baseline) sit
# far above the boundary and are unaffected by the widening.
SLAM_RPE_DRIFT_ERROR = 0.023   # m, per-window SE(2) translation error (clean max 0.017 x ~1.35)
SLAM_ATE_RMSE_ERROR = 0.009    # m, aligned absolute trajectory RMSE (clean max 0.006 x 1.5)
SLAM_YAW_DRIFT_ERROR = 0.023   # rad, heading-offset spread (clean max 0.021 x ~1.1)
# Odom is an attribution guard, not the SLAM estimator under test.  Keep its
# much wider trust envelope independent from the calibrated SLAM alarm limit.
SLAM_ODOM_RPE_TRUST_MAX = 0.5  # m

# Sampling / windowing.
SLAM_MIN_SAMPLES = 10         # matched pairs needed before judging
SLAM_RPE_WINDOW_SEC = 1.0     # relative-motion comparison window
SLAM_MATCH_MAX_DT_SEC = 0.25  # maximum estimate/truth timestamp separation
SLAM_QUATERNION_NORM_TOL = 1e-3

ESTIMATE_TOPIC = "/pose"
GROUND_TRUTH_TOPIC = "/sim_ground_truth_pose"
COLLISION_TOPIC = "/bumper_contact"
SCAN_TOPIC = "/scan"
MAP_TOPIC = "/map"
ODOM_TOPIC = "/odom"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
SLAM_MAP_RESOLUTION = 0.05
SLAM_SCAN_TRANSLATION_GATE = 0.1
SLAM_SCAN_ROTATION_GATE = 0.1
SLAM_ATE_MIN_TRANSLATION_EXCITATION = 0.25
SLAM_ATE_MIN_ROTATION_EXCITATION = 0.20
SLAM_REVISIT_RADIUS = 0.25
SLAM_REVISIT_MIN_TIME_SEC = 2.0
SLAM_REVISIT_MIN_PATH = 1.0

SLAM_FEEDBACK_NAMES = (
    "slam_oracle_valid",
    "slam_collision_detected",
    "slam_pose_sample_count",
    "slam_match_ratio",
    "slam_max_rpe_drift",
    "slam_displacement_magnitude_error",
    "slam_rpe_translation_rmse",
    "slam_rpe_rotation_max",
    "slam_rpe_rotation_rmse",
    "slam_ate_rmse",
    "slam_ate_max",
    "slam_ate_median",
    "slam_max_yaw_drift",
    "slam_odom_rpe_drift",
    "slam_pose_odom_excess_rpe",
    "slam_translation_excitation",
    "slam_rotation_excitation",
    "slam_scan_finite_ratio",
    "slam_scan_gate_opportunities",
    "slam_map_known_area",
    "slam_map_growth",
    "slam_revisit_count",
    "slam_loop_closure_residual",
    "slam_tf_coverage",
    "slam_tf_max_age",
)


def _quaternion_components(orientation):
    try:
        values = tuple(
            float(getattr(orientation, field))
            for field in ("x", "y", "z", "w")
        )
    except (AttributeError, TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    return values


def _quaternion_is_valid(orientation):
    values = _quaternion_components(orientation)
    if values is None:
        return False
    norm = math.sqrt(sum(value * value for value in values))
    return (
        norm > 0.0
        and abs(norm - 1.0) <= SLAM_QUATERNION_NORM_TOL
    )


def _ts_to_sec(ts):
    """Convert a (possibly 16+ digit trimmed) bag timestamp to seconds."""
    ts_str = str(ts)
    if len(ts_str) <= 10:
        return float(ts)
    return int(ts_str[:10]) + int(ts_str[10:]) / (10 ** (len(ts_str) - 10))


def _message_time_sec(msg, fallback_ts):
    """Use a valid non-zero ROS header stamp, otherwise the bag timestamp."""
    try:
        stamp = msg.header.stamp
        sec = float(stamp.sec)
        nanosec = float(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return _ts_to_sec(fallback_ts)

    valid_stamp = (
        math.isfinite(sec)
        and math.isfinite(nanosec)
        and 0.0 <= nanosec < 1_000_000_000.0
        and not (sec == 0.0 and nanosec == 0.0)
    )
    if not valid_stamp:
        return _ts_to_sec(fallback_ts)
    return sec + nanosec / 1_000_000_000.0


def _yaw_from_quat(orientation):
    values = _quaternion_components(orientation)
    if values is None:
        return float("nan")
    x, y, z, w = values
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
    if not (
        math.isfinite(x)
        and math.isfinite(y)
        and _quaternion_is_valid(orientation)
    ):
        return None
    yaw = _yaw_from_quat(orientation)
    return x, y, yaw


def _pose_object(msg):
    pose = getattr(msg, "pose", None)
    inner = getattr(pose, "pose", None)
    if inner is not None and hasattr(inner, "position"):
        return inner
    return pose


def _finite_xyz(vector):
    try:
        values = tuple(
            float(getattr(vector, field)) for field in ("x", "y", "z")
        )
    except (AttributeError, TypeError, ValueError):
        return False
    return all(math.isfinite(value) for value in values)


def _header_frame_id(msg):
    try:
        return str(msg.header.frame_id)
    except (AttributeError, TypeError, ValueError):
        return ""


def _pose_track(records):
    """records: list of (ts, msg) -> sorted list of (t_sec, x, y, yaw)."""
    track = []
    for ts, msg in records or []:
        pose = _extract_pose(msg)
        if pose is None:
            continue
        try:
            t = _message_time_sec(msg, ts)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(t):
            continue
        track.append((t, pose[0], pose[1], pose[2]))
    track.sort(key=lambda item: item[0])
    return track


def _match_nearest(est_track, truth_track):
    """Pair tracks monotonically, nearest in time and one-to-one.

    Each truth sample can be used at most once; after a match, later estimates
    consider only later unused truth samples. Returns (ex, ey, eyaw, tx, ty,
    tyaw) pairs and matched estimate timestamps for window sizing.
    """
    if not est_track or not truth_track:
        return [], []

    truth_times = [row[0] for row in truth_track]
    pairs = []
    matched_times = []
    truth_start = 0
    for t, ex, ey, eyaw in est_track:
        if truth_start >= len(truth_track):
            break
        relative_idx = _nearest_index(truth_times[truth_start:], t)
        idx = truth_start + relative_idx
        truth_t, tx, ty, tyaw = truth_track[idx]
        if abs(truth_t - t) > SLAM_MATCH_MAX_DT_SEC:
            continue
        pairs.append((ex, ey, eyaw, tx, ty, tyaw))
        matched_times.append(t)
        truth_start = idx + 1
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


def _relative_pose(a, b):
    """Return the planar transform from pose ``a`` to pose ``b``."""
    dx_world = b[0] - a[0]
    dy_world = b[1] - a[1]
    c = math.cos(a[2])
    s = math.sin(a[2])
    return (
        c * dx_world + s * dy_world,
        -s * dx_world + c * dy_world,
        _wrap_to_pi(b[2] - a[2]),
    )


def _rpe_metrics(pairs, window):
    """Compute true SE(2) relative-pose translation and rotation errors."""
    translation_errors = []
    rotation_errors = []
    n = len(pairs)
    for i in range(n - window):
        later = i + window
        est_rel = _relative_pose(pairs[i][0:3], pairs[later][0:3])
        truth_rel = _relative_pose(pairs[i][3:6], pairs[later][3:6])

        # E = inverse(T_truth_rel) * T_est_rel.  Rotate the translation
        # difference into the truth-relative frame; its norm is invariant.
        dx = est_rel[0] - truth_rel[0]
        dy = est_rel[1] - truth_rel[1]
        c = math.cos(truth_rel[2])
        s = math.sin(truth_rel[2])
        error_x = c * dx + s * dy
        error_y = -s * dx + c * dy
        translation_errors.append(math.hypot(error_x, error_y))
        rotation_errors.append(
            abs(_wrap_to_pi(est_rel[2] - truth_rel[2]))
        )

    if not translation_errors:
        return {
            "translation_max": 0.0,
            "translation_rmse": 0.0,
            "rotation_max": 0.0,
            "rotation_rmse": 0.0,
        }
    return {
        "translation_max": max(translation_errors),
        "translation_rmse": math.sqrt(
            sum(value * value for value in translation_errors)
            / len(translation_errors)
        ),
        "rotation_max": max(rotation_errors),
        "rotation_rmse": math.sqrt(
            sum(value * value for value in rotation_errors)
            / len(rotation_errors)
        ),
    }


def _max_rpe(pairs, window):
    """Backward-compatible accessor for true SE(2) translation RPE."""
    return _rpe_metrics(pairs, window)["translation_max"]


def _max_displacement_magnitude_error(pairs, window):
    """Legacy displacement-length proxy retained only as a diagnostic."""
    max_error = 0.0
    for i in range(len(pairs) - window):
        later = i + window
        est_disp = math.hypot(
            pairs[later][0] - pairs[i][0],
            pairs[later][1] - pairs[i][1],
        )
        truth_disp = math.hypot(
            pairs[later][3] - pairs[i][3],
            pairs[later][4] - pairs[i][4],
        )
        max_error = max(max_error, abs(est_disp - truth_disp))
    return max_error


def _ate_residual_distances(pairs):
    """Rigidly align estimate to truth and return per-sample distances."""
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
    return np.sqrt((residual ** 2).sum(axis=1))


def _ate_rmse(pairs):
    """Umeyama rigid alignment (no scale) of estimate onto truth; RMSE."""
    distances = _ate_residual_distances(pairs)
    return float(np.sqrt((distances ** 2).mean()))


def _ate_max_median(pairs):
    distances = _ate_residual_distances(pairs)
    return float(distances.max()), float(np.median(distances))


def _yaw_drift_spread(pairs):
    """Spread of the estimate-vs-truth yaw offset (constant offset cancels)."""
    diffs = [_wrap_to_pi(p[2] - p[5]) for p in pairs]
    sin_sum = sum(math.sin(value) for value in diffs)
    cos_sum = sum(math.cos(value) for value in diffs)
    center = math.atan2(sin_sum, cos_sum)
    rel = [_wrap_to_pi(d - center) for d in diffs]
    return max(rel) - min(rel)


def _pose_record_errors(records, source):
    """Return stable numerical errors without relying on track extraction."""
    errors = []
    seen = set()
    previous_time = None
    for timestamp, msg in records or []:
        pose = _pose_object(msg)
        position = getattr(pose, "position", None)
        orientation = getattr(pose, "orientation", None)
        if not _finite_xyz(position) and "position" not in seen:
            errors.append(
                f"SLAM_NUMERIC_NONFINITE:{source}.position source={source}"
            )
            seen.add("position")
        if orientation is not None and not _quaternion_is_valid(orientation):
            if "quaternion" not in seen:
                errors.append(
                f"SLAM_INVALID_QUATERNION:{source} source={source}"
                )
                seen.add("quaternion")

        frame_id = _header_frame_id(msg)
        if (
            source == "pose"
            and frame_id
            and frame_id != "map"
            and "frame" not in seen
        ):
            errors.append(
                f"SLAM_FRAME_MISMATCH:pose source=pose "
                f"expected=map actual={frame_id}"
            )
            seen.add("frame")

        try:
            current_time = _message_time_sec(msg, timestamp)
        except (TypeError, ValueError):
            current_time = None
        if (
            current_time is not None
            and math.isfinite(current_time)
            and previous_time is not None
            and current_time < previous_time
            and "time" not in seen
        ):
            errors.append(
                f"SLAM_TIMESTAMP_REGRESSION:{source} source={source}"
            )
            seen.add("time")
        if current_time is not None and math.isfinite(current_time):
            previous_time = current_time
    return errors


def _scan_record_errors(records):
    errors = []
    seen = set()
    for _timestamp, msg in records or []:
        try:
            angle_min = float(msg.angle_min)
            angle_max = float(msg.angle_max)
            angle_increment = float(msg.angle_increment)
            range_min = float(msg.range_min)
            range_max = float(msg.range_max)
            ranges = list(msg.ranges)
        except (AttributeError, TypeError, ValueError):
            if "structure" not in seen:
                errors.append("SLAM_SCAN_STRUCTURE:scan source=scan")
                seen.add("structure")
            continue

        metadata = (
            angle_min, angle_max, angle_increment, range_min, range_max
        )
        metadata_valid = (
            all(math.isfinite(value) for value in metadata)
            and angle_increment > 0.0
            and angle_max >= angle_min
            and range_min >= 0.0
            and range_max > range_min
        )
        if not metadata_valid:
            if "structure" not in seen:
                errors.append("SLAM_SCAN_STRUCTURE:scan source=scan")
                seen.add("structure")
            continue

        expected_count = int(round(
            (angle_max - angle_min) / angle_increment
        )) + 1
        if abs(len(ranges) - expected_count) > 1 and "shape" not in seen:
            errors.append(
                "SLAM_SCAN_SHAPE:scan source=scan "
                f"expected={expected_count} actual={len(ranges)}"
            )
            seen.add("shape")

        invalid_range = False
        for value in ranges:
            try:
                distance = float(value)
            except (TypeError, ValueError):
                invalid_range = True
                break
            if math.isinf(distance):
                if distance > 0.0:
                    continue
                invalid_range = True
                break
            if (
                not math.isfinite(distance)
                or distance < range_min
                or distance > range_max
            ):
                invalid_range = True
                break
        if invalid_range and "range" not in seen:
            errors.append("SLAM_SCAN_INVALID_RANGE:scan source=scan")
            seen.add("range")
    return errors


def _map_record_errors(records):
    errors = []
    seen = set()
    for _timestamp, msg in records or []:
        info = getattr(msg, "info", None)
        try:
            resolution = float(info.resolution)
            width = int(info.width)
            height = int(info.height)
            data = list(msg.data)
        except (AttributeError, TypeError, ValueError):
            if "structure" not in seen:
                errors.append("SLAM_MAP_STRUCTURE:map source=map")
                seen.add("structure")
            continue

        if (
            not math.isfinite(resolution)
            or resolution <= 0.0
            or not math.isclose(
                resolution, SLAM_MAP_RESOLUTION, rel_tol=0.0, abs_tol=1e-6
            )
        ) and "resolution" not in seen:
            errors.append(
                "SLAM_MAP_RESOLUTION:map source=map "
                f"expected={SLAM_MAP_RESOLUTION} actual={resolution}"
            )
            seen.add("resolution")

        if (
            width <= 0
            or height <= 0
            or len(data) != width * height
        ) and "shape" not in seen:
            errors.append(
                "SLAM_MAP_SHAPE:map source=map "
                f"width={width} height={height} cells={len(data)}"
            )
            seen.add("shape")

        if any(value < -1 or value > 100 for value in data):
            if "cell" not in seen:
                errors.append("SLAM_MAP_INVALID_CELL:map source=map")
                seen.add("cell")

        frame_id = _header_frame_id(msg)
        if frame_id and frame_id != "map" and "frame" not in seen:
            errors.append(
                "SLAM_FRAME_MISMATCH:map source=map "
                f"expected=map actual={frame_id}"
            )
            seen.add("frame")

        origin = getattr(info, "origin", None)
        if not _finite_xyz(getattr(origin, "position", None)):
            if "origin_position" not in seen:
                errors.append(
                    "SLAM_NUMERIC_NONFINITE:map.origin source=map"
                )
                seen.add("origin_position")
        if not _quaternion_is_valid(getattr(origin, "orientation", None)):
            if "origin_quaternion" not in seen:
                errors.append(
                    "SLAM_INVALID_QUATERNION:map.origin source=map"
                )
                seen.add("origin_quaternion")
    return errors


def _tf_record_errors(records):
    errors = []
    seen = set()
    for _timestamp, msg in records or []:
        transforms = getattr(msg, "transforms", None)
        if transforms is None:
            if "structure" not in seen:
                errors.append("SLAM_TF_STRUCTURE:tf source=tf")
                seen.add("structure")
            continue
        for transform_stamped in transforms:
            parent = _header_frame_id(transform_stamped)
            child = str(getattr(transform_stamped, "child_frame_id", ""))
            transform = getattr(transform_stamped, "transform", None)
            edge = f"{parent}->{child}"
            if not parent or not child:
                if "frame" not in seen:
                    errors.append(
                        f"SLAM_TF_FRAME:tf source=tf edge={edge}"
                    )
                    seen.add("frame")
            if parent and parent == child and "self_edge" not in seen:
                errors.append(
                    f"SLAM_TF_SELF_EDGE:tf source=tf edge={edge}"
                )
                seen.add("self_edge")
            if not _finite_xyz(getattr(transform, "translation", None)):
                if "translation" not in seen:
                    errors.append(
                        f"SLAM_TF_NONFINITE:tf source=tf edge={edge}"
                    )
                    seen.add("translation")
            if not _quaternion_is_valid(getattr(transform, "rotation", None)):
                if "quaternion" not in seen:
                    errors.append(
                        "SLAM_TF_INVALID_QUATERNION:tf "
                        f"source=tf edge={edge}"
                    )
                    seen.add("quaternion")
    return errors


def _scan_metrics(records):
    total = 0
    finite = 0
    for _timestamp, msg in records or []:
        for value in getattr(msg, "ranges", []) or []:
            total += 1
            try:
                if math.isfinite(float(value)):
                    finite += 1
            except (TypeError, ValueError):
                continue
    return {"finite_ratio": finite / total if total else 0.0}


def _map_metrics(records):
    known_areas = []
    for _timestamp, msg in records or []:
        info = getattr(msg, "info", None)
        try:
            resolution = float(info.resolution)
            data = list(msg.data)
        except (AttributeError, TypeError, ValueError):
            continue
        if not math.isfinite(resolution) or resolution <= 0.0:
            continue
        known = sum(1 for value in data if value >= 0)
        known_areas.append(known * resolution * resolution)
    if not known_areas:
        return {"known_area": 0.0, "growth": 0.0}
    return {
        "known_area": known_areas[-1],
        "growth": max(0.0, known_areas[-1] - known_areas[0]),
    }


def _motion_gate_opportunities(track):
    if len(track) < 2:
        return 0
    opportunities = 0
    anchor = track[0]
    for sample in track[1:]:
        distance = math.hypot(sample[1] - anchor[1], sample[2] - anchor[2])
        rotation = abs(_wrap_to_pi(sample[3] - anchor[3]))
        if (
            distance + 1e-12 >= SLAM_SCAN_TRANSLATION_GATE
            or rotation + 1e-12 >= SLAM_SCAN_ROTATION_GATE
        ):
            opportunities += 1
            anchor = sample
    return opportunities


def _trajectory_excitation(pairs):
    translation = 0.0
    rotation = 0.0
    for previous, current in zip(pairs, pairs[1:]):
        translation += math.hypot(
            current[3] - previous[3], current[4] - previous[4]
        )
        rotation += abs(_wrap_to_pi(current[5] - previous[5]))
    return translation, rotation


def _window_for_times(times, sample_count):
    dt = _median_dt(times)
    window = max(1, int(round(SLAM_RPE_WINDOW_SEC / dt))) if dt > 0 else 1
    return min(window, sample_count - 1)


def _revisit_metrics(pairs, matched_times):
    if len(pairs) < 2 or len(matched_times) != len(pairs):
        return {"count": 0.0, "max_residual": 0.0}

    cumulative_path = [0.0]
    for previous, current in zip(pairs, pairs[1:]):
        cumulative_path.append(
            cumulative_path[-1]
            + math.hypot(current[3] - previous[3], current[4] - previous[4])
        )

    count = 0
    max_residual = 0.0
    in_revisit_episode = False
    for later in range(1, len(pairs)):
        candidate_residuals = []
        for earlier in range(later):
            if (
                matched_times[later] - matched_times[earlier]
                < SLAM_REVISIT_MIN_TIME_SEC
            ):
                continue
            if (
                cumulative_path[later] - cumulative_path[earlier]
                < SLAM_REVISIT_MIN_PATH
            ):
                continue
            truth_distance = math.hypot(
                pairs[later][3] - pairs[earlier][3],
                pairs[later][4] - pairs[earlier][4],
            )
            if truth_distance > SLAM_REVISIT_RADIUS:
                continue

            est_rel = _relative_pose(
                pairs[earlier][0:3], pairs[later][0:3]
            )
            truth_rel = _relative_pose(
                pairs[earlier][3:6], pairs[later][3:6]
            )
            residual = math.hypot(
                est_rel[0] - truth_rel[0],
                est_rel[1] - truth_rel[1],
            )
            candidate_residuals.append(residual)

        if candidate_residuals:
            if not in_revisit_episode:
                count += 1
            max_residual = max(max_residual, max(candidate_residuals))
            in_revisit_episode = True
        else:
            in_revisit_episode = False
    return {"count": float(count), "max_residual": max_residual}


def _tf_metrics(state_dict):
    edges = set()
    dynamic_times = []
    for topic in (TF_TOPIC, TF_STATIC_TOPIC):
        for bag_timestamp, msg in state_dict.get(topic, []) or []:
            for transform_stamped in getattr(msg, "transforms", []) or []:
                parent = _header_frame_id(transform_stamped)
                child = str(
                    getattr(transform_stamped, "child_frame_id", "")
                )
                transform = getattr(transform_stamped, "transform", None)
                if (
                    parent
                    and child
                    and parent != child
                    and _finite_xyz(getattr(transform, "translation", None))
                    and _quaternion_is_valid(
                        getattr(transform, "rotation", None)
                    )
                ):
                    edges.add((parent, child))
                    if topic == TF_TOPIC and parent == "map" and child == "odom":
                        dynamic_times.append(
                            _message_time_sec(transform_stamped, bag_timestamp)
                        )

    if not edges:
        return {"coverage": 0.0, "max_age": 0.0}

    graph = {}
    for parent, child in edges:
        graph.setdefault(parent, set()).add(child)
        graph.setdefault(child, set()).add(parent)

    reachable = {"map"}
    frontier = ["map"]
    while frontier:
        frame = frontier.pop()
        for neighbor in graph.get(frame, set()):
            if neighbor not in reachable:
                reachable.add(neighbor)
                frontier.append(neighbor)

    targets = {"odom"}
    for _timestamp, msg in state_dict.get(SCAN_TOPIC, []) or []:
        frame_id = _header_frame_id(msg)
        if frame_id:
            targets.add(frame_id)
    for _timestamp, msg in state_dict.get(ODOM_TOPIC, []) or []:
        child = str(getattr(msg, "child_frame_id", ""))
        if child:
            targets.add(child)
    coverage = sum(target in reachable for target in targets) / len(targets)

    reference_times = []
    for topic in (ESTIMATE_TOPIC, SCAN_TOPIC):
        for bag_timestamp, msg in state_dict.get(topic, []) or []:
            reference_times.append(_message_time_sec(msg, bag_timestamp))
    max_age = 0.0
    if dynamic_times and reference_times:
        max_age = max(
            min(abs(reference - tf_time) for tf_time in dynamic_times)
            for reference in reference_times
        )
    return {"coverage": coverage, "max_age": max_age}


def _update_feedback(feedback_list, name, value):
    for fbk in feedback_list or []:
        if fbk.name == name:
            fbk.update_value(value)
            return


def _round_has_collision(state_dict):
    """True if any /bumper_contact message this round carries a contact.

    The gz Contact system publishes ros_gz_interfaces/msg/Contacts. Ground
    truth remains reliable during contact, but the robot's motion is no longer
    cleanly attributable to the fuzzed command alone. Such rounds are excluded
    from clean commanded-motion drift judgment. Missing topic -> no collision
    info -> judge normally (backward compatible without a bumper).
    """
    for _, msg in state_dict.get(COLLISION_TOPIC, []):
        contacts = getattr(msg, "contacts", None)
        if contacts:  # non-empty list => a real contact
            return True
    return False


def check(config, msg_list, state_dict, feedback_list):
    """SLAM semantic oracle entry point (see checker.run_checks dispatch)."""
    errs = []
    for feedback_name in SLAM_FEEDBACK_NAMES:
        _update_feedback(feedback_list, feedback_name, 0.0)

    est_track = _pose_track(state_dict.get(ESTIMATE_TOPIC, []))
    truth_track = _pose_track(state_dict.get(GROUND_TRUTH_TOPIC, []))
    odom_track = _pose_track(state_dict.get(ODOM_TOPIC, []))
    errs.extend(
        _pose_record_errors(state_dict.get(ESTIMATE_TOPIC, []), "pose")
    )
    errs.extend(
        _pose_record_errors(
            state_dict.get(GROUND_TRUTH_TOPIC, []), "ground_truth"
        )
    )
    errs.extend(_pose_record_errors(state_dict.get(ODOM_TOPIC, []), "odom"))
    errs.extend(_scan_record_errors(state_dict.get(SCAN_TOPIC, [])))
    errs.extend(_map_record_errors(state_dict.get(MAP_TOPIC, [])))
    errs.extend(_tf_record_errors(state_dict.get(TF_TOPIC, [])))
    errs.extend(_tf_record_errors(state_dict.get(TF_STATIC_TOPIC, [])))

    scan_metrics = _scan_metrics(state_dict.get(SCAN_TOPIC, []))
    map_metrics = _map_metrics(state_dict.get(MAP_TOPIC, []))
    tf_metrics = _tf_metrics(state_dict)
    motion_track = odom_track if odom_track else truth_track
    _update_feedback(
        feedback_list, "slam_scan_finite_ratio",
        scan_metrics["finite_ratio"],
    )
    _update_feedback(
        feedback_list, "slam_scan_gate_opportunities",
        float(_motion_gate_opportunities(motion_track)),
    )
    _update_feedback(
        feedback_list, "slam_map_known_area", map_metrics["known_area"]
    )
    _update_feedback(
        feedback_list, "slam_map_growth", map_metrics["growth"]
    )
    _update_feedback(
        feedback_list, "slam_tf_coverage", tf_metrics["coverage"]
    )
    _update_feedback(
        feedback_list, "slam_tf_max_age", tf_metrics["max_age"]
    )

    pairs, matched_times = (
        _match_nearest(est_track, truth_track)
        if est_track and truth_track else ([], [])
    )
    _update_feedback(
        feedback_list, "slam_pose_sample_count", float(len(pairs))
    )
    match_denominator = min(len(est_track), len(truth_track))
    _update_feedback(
        feedback_list,
        "slam_match_ratio",
        len(pairs) / match_denominator if match_denominator else 0.0,
    )

    revisit = _revisit_metrics(pairs, matched_times)
    _update_feedback(
        feedback_list, "slam_revisit_count", revisit["count"]
    )
    _update_feedback(
        feedback_list,
        "slam_loop_closure_residual",
        revisit["max_residual"],
    )
    round_has_collision = _round_has_collision(state_dict)
    _update_feedback(
        feedback_list,
        "slam_collision_detected",
        1.0 if round_has_collision else 0.0,
    )

    # Ground truth is still reliable after contact, but this round is not a
    # clean commanded-motion trial, so skip drift judgment while retaining
    # time-aligned sample coverage for visibility.
    if round_has_collision:
        return errs

    # Missing either signal is a readiness/recording-quality issue, not a bug.
    if not est_track or not truth_track:
        _update_feedback(feedback_list, "slam_pose_sample_count", 0.0)
        return errs

    if len(pairs) < SLAM_MIN_SAMPLES:
        return errs

    window = _window_for_times(matched_times, len(pairs))
    rpe = _rpe_metrics(pairs, window)
    max_rpe = rpe["translation_max"]
    displacement_error = _max_displacement_magnitude_error(pairs, window)
    ate_rmse = _ate_rmse(pairs)
    ate_max, ate_median = _ate_max_median(pairs)
    yaw_drift = max(_yaw_drift_spread(pairs), rpe["rotation_max"])
    translation_excitation, rotation_excitation = _trajectory_excitation(
        pairs
    )

    odom_rpe = 0.0
    odom_baseline_trustworthy = True
    if odom_track:
        odom_pairs, odom_times = _match_nearest(odom_track, truth_track)
        if len(odom_pairs) >= SLAM_MIN_SAMPLES:
            odom_window = _window_for_times(odom_times, len(odom_pairs))
            odom_rpe = _rpe_metrics(
                odom_pairs, odom_window
            )["translation_max"]
            odom_baseline_trustworthy = odom_rpe <= SLAM_ODOM_RPE_TRUST_MAX

    _update_feedback(feedback_list, "slam_max_rpe_drift", max_rpe)
    _update_feedback(
        feedback_list, "slam_displacement_magnitude_error",
        displacement_error,
    )
    _update_feedback(
        feedback_list, "slam_rpe_translation_rmse",
        rpe["translation_rmse"],
    )
    _update_feedback(
        feedback_list, "slam_rpe_rotation_max", rpe["rotation_max"]
    )
    _update_feedback(
        feedback_list, "slam_rpe_rotation_rmse", rpe["rotation_rmse"]
    )
    _update_feedback(feedback_list, "slam_ate_rmse", ate_rmse)
    _update_feedback(feedback_list, "slam_ate_max", ate_max)
    _update_feedback(feedback_list, "slam_ate_median", ate_median)
    _update_feedback(feedback_list, "slam_max_yaw_drift", yaw_drift)
    _update_feedback(feedback_list, "slam_odom_rpe_drift", odom_rpe)
    _update_feedback(
        feedback_list, "slam_pose_odom_excess_rpe",
        max(0.0, max_rpe - odom_rpe),
    )
    _update_feedback(
        feedback_list, "slam_translation_excitation",
        translation_excitation,
    )
    _update_feedback(
        feedback_list, "slam_rotation_excitation", rotation_excitation
    )
    _update_feedback(feedback_list, "slam_oracle_valid", 1.0)

    if odom_baseline_trustworthy and max_rpe > SLAM_RPE_DRIFT_ERROR:
        errs.append(
            f"SLAM_RPE_ERROR:pose source=pose rpe_m={max_rpe:.5f} "
            f"window={window} limit_m={SLAM_RPE_DRIFT_ERROR}"
        )
    ate_has_excitation = (
        translation_excitation >= SLAM_ATE_MIN_TRANSLATION_EXCITATION
        or rotation_excitation >= SLAM_ATE_MIN_ROTATION_EXCITATION
    )
    if (
        odom_baseline_trustworthy
        and ate_has_excitation
        and ate_rmse > SLAM_ATE_RMSE_ERROR
    ):
        errs.append(
            f"SLAM_ATE_ERROR:pose source=pose ate_rmse_m={ate_rmse:.5f} "
            f"limit_m={SLAM_ATE_RMSE_ERROR}"
        )
    if odom_baseline_trustworthy and yaw_drift > SLAM_YAW_DRIFT_ERROR:
        errs.append(
            f"SLAM_YAW_ERROR:pose source=pose yaw_rad={yaw_drift:.5f} "
            f"limit_rad={SLAM_YAW_DRIFT_ERROR}"
        )

    return errs
