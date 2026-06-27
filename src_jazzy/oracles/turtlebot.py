import math
import statistics
import numpy as np


# === TurtleBot3 Hardware Spec Constants ===
TB3_MAX_LIN_VEL = 0.22       # m/s (Burger)
TB3_MAX_ANG_VEL = 2.84       # rad/s (Burger)
TB3_VEL_TOLERANCE = 0.005    # m/s tolerance for simulation numerical error
TB3_MAX_LIN_ACCEL = 5.0      # m/s^2 (derived from XL430-W250 torque)
TB3_MAX_ANG_ACCEL = 30.0     # rad/s^2
TB3_QUAT_NORM_TOL = 0.01     # unit quaternion tolerance
TB3_Z_POS_TOL = 0.05         # m (allow minor bounce from contact forces)
TB3_LATERAL_VEL_TOL = 0.03   # m/s (filter simulation numerical noise)
TB3_ROLL_PITCH_TOL = 0.05    # rad/s (ground robot, no roll/pitch)
TB3_IMU_ROLL_PITCH_TOL = 2.0 # rad/s (allow physics coupling during turns)
TB3_VEL_POS_TOL = 0.05       # m (velocity-position consistency)
TB3_IMU_ODOM_ACCEL_TOL = 2.0 # m/s^2 (cross-validation tolerance)
TB3_LIDAR_TEMPORAL_TOL = 1.0 # m (mean beam change between scans)
TB3_CMD_VEL_LIN_TOL = 0.1    # m/s (steady-state tracking error)
TB3_CMD_VEL_ANG_TOL = 0.5    # rad/s


def _ts_to_sec(ts):
    """Convert 16-digit trimmed timestamp to seconds (float)."""
    ts_str = str(ts)
    if len(ts_str) <= 10:
        return float(ts)
    return int(ts_str[:10]) + int(ts_str[10:]) / (10 ** (len(ts_str) - 10))


def _odom_header_stamp_to_sec(odom):
    header = getattr(odom, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None

    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None

    try:
        ts_sec = float(sec) + float(nanosec) / 1e9
    except (TypeError, ValueError):
        return None

    return ts_sec if math.isfinite(ts_sec) else None


def _cmd_twist(msg):
    return msg.twist if hasattr(msg, "twist") else msg


# === TurtleBot4 Phase-1 sanity thresholds ===
# These are generic numerical-sanity / sign-agreement tolerances, NOT
# TurtleBot3/Burger hardware limits. Real velocity-envelope and
# hazard/reflex thresholds are deferred to later phases (derived from
# TurtleBot4/Create3 docs, runtime parameters, or measured baselines).
TB4_QUAT_NORM_TOL = 0.05      # unit-quaternion deviation tolerance
TB4_CMD_SIGN_THRESH = 0.05    # min |cmd| to assert a direction (m/s, rad/s)
TB4_ODOM_SIGN_THRESH = 0.05   # min mean |odom| to assert a direction
TB4_SCAN_RANGE_REL_TOL = 0.05  # relative tolerance over declared range_max
TB4_SCAN_RANGE_ABS_TOL = 0.05  # absolute tolerance over declared range_max

# TurtleBot4 Jazzy diffdrive_controller limits derived from
# src_jazzy/oracle_ir/specs/turtlebot4_jazzy and the corresponding cleaned
# runtime parameters. These are executable oracle thresholds; OracleIR remains
# provenance / design input rather than a runtime dependency.
TB4_MAX_LIN_VEL = 0.46        # m/s
TB4_MIN_LIN_VEL = -0.46       # m/s
TB4_MAX_ANG_VEL = 1.9         # rad/s
TB4_MIN_ANG_VEL = -1.9        # rad/s
TB4_MAX_LIN_ACCEL = 0.9       # m/s^2
TB4_MAX_ANG_ACCEL = 7.725     # rad/s^2
TB4_MIN_ANG_ACCEL = -7.725    # rad/s^2
TB4_CMD_TIMEOUT = 0.5         # s
TB4_PUBLISH_RATE_HZ = 62.0    # Hz

# Conservative bug thresholds. Values slightly over the exact parameter bound
# avoid treating bag timestamp jitter and simulator/controller discretization
# as target bugs.
TB4_VEL_RATIO_ERROR = 1.05
TB4_ACCEL_RATIO_ERROR = 1.10
TB4_ACCEL_STRONG_RATIO_ERROR = 5.00
TB4_ACCEL_SUSTAINED_SAMPLES = 2
TB4_STALE_MOTION_THRESH_LIN = 0.03
TB4_STALE_MOTION_THRESH_ANG = 0.08
TB4_CMD_TIMEOUT_ERROR_GRACE = 0.25
TB4_CMD_TIMEOUT_ERROR_SAMPLES = 2
TB4_WHEEL_SIGN_THRESH = 0.05


def _mean(values):
    return sum(values) / len(values) if values else 0.0


def _update_feedback(feedback_list, name, value):
    for fbk in feedback_list:
        if fbk.name == name:
            fbk.update_value(value)
            return


def _consistent_sign(values, threshold):
    signs = set()
    for value in values:
        if value > threshold:
            signs.add(1)
        elif value < -threshold:
            signs.add(-1)
    if not signs:
        return 0
    if len(signs) > 1:
        return None
    return next(iter(signs))


def _check_scan_sanity(scan_list, errs, feedback_list):
    """LaserScan prepass: NaN / negative are errors; inf is a valid
    'no return' reading. Finite values beyond the declared range_max
    (with tolerance) are flagged. Populates scan feedback metrics."""
    min_finite = float("inf")
    total = 0
    invalid = 0
    saw_nan = False
    saw_negative = False
    saw_over_range = False

    for _, scan in scan_list:
        ranges = getattr(scan, "ranges", [])
        range_max = getattr(scan, "range_max", None)
        for value in ranges:
            total += 1
            if math.isnan(value):
                invalid += 1
                saw_nan = True
                continue
            if math.isinf(value):
                # inf == out of range / no obstacle; valid for LaserScan
                continue
            if value < 0.0:
                invalid += 1
                saw_negative = True
                continue
            if value < min_finite:
                min_finite = value
            if (
                range_max is not None
                and range_max > 0.0
                and value > range_max * (1.0 + TB4_SCAN_RANGE_REL_TOL)
                + TB4_SCAN_RANGE_ABS_TOL
            ):
                saw_over_range = True

    if saw_nan:
        errs.append("scan.ranges contains NaN")
    if saw_negative:
        errs.append("scan.ranges contains negative value")
    if saw_over_range:
        errs.append("scan.ranges exceeds declared range_max")

    if math.isinf(min_finite):
        finite_range_max = [
            getattr(scan, "range_max", 0.0)
            for _, scan in scan_list
            if math.isfinite(getattr(scan, "range_max", 0.0))
            and getattr(scan, "range_max", 0.0) > 0.0
        ]
        min_range_val = max(finite_range_max) if finite_range_max else 0.0
    else:
        min_range_val = min_finite
    invalid_ratio = (invalid / total) if total else 0.0
    for fbk in feedback_list:
        if fbk.name == "scan_min_range":
            fbk.update_value(min_range_val)
        elif fbk.name == "scan_invalid_ratio":
            fbk.update_value(invalid_ratio)


def _check_odom_sanity(odom_list, errs):
    """Ground-robot odometry prepass: flag NaN/Inf in pose/twist and
    quaternion norm far from unit. No TB3 velocity/accel limits here."""
    for _, odom in odom_list:
        pose = odom.pose.pose
        twist = odom.twist.twist
        scalars = [
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
        ]
        if any(math.isnan(v) for v in scalars):
            errs.append("odom contains NaN")
        if any(math.isinf(v) for v in scalars):
            errs.append("odom contains Inf")

        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        if all(math.isfinite(v) for v in (qx, qy, qz, qw)):
            norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            if abs(norm - 1.0) > TB4_QUAT_NORM_TOL:
                errs.append(
                    f"odom quaternion norm ({norm:.4f}) deviates from 1.0"
                )


def _check_cmd_odom_consistency(msg_list, odom_list, errs, feedback_list):
    """Sign-agreement check using the published command (msg_list) against
    the mean odom velocity over the recorded window. A single transient
    opposite sample is tolerated because the mean is used. Populates
    cmd/odom feedback metrics."""
    if not msg_list or not odom_list:
        return

    cmd_x_values = [
        getattr(_cmd_twist(msg).linear, "x", 0.0) for msg in msg_list
    ]
    cmd_z_values = [
        getattr(_cmd_twist(msg).angular, "z", 0.0) for msg in msg_list
    ]
    last_cmd = _cmd_twist(msg_list[-1])
    cmd_x = getattr(last_cmd.linear, "x", 0.0)
    cmd_z = getattr(last_cmd.angular, "z", 0.0)
    cmd_x_sign = _consistent_sign(cmd_x_values, TB4_CMD_SIGN_THRESH)
    cmd_z_sign = _consistent_sign(cmd_z_values, TB4_CMD_SIGN_THRESH)

    lin_vals = [o.twist.twist.linear.x for _, o in odom_list
                if math.isfinite(o.twist.twist.linear.x)]
    ang_vals = [o.twist.twist.angular.z for _, o in odom_list
                if math.isfinite(o.twist.twist.angular.z)]
    mean_lin = _mean(lin_vals)
    mean_ang = _mean(ang_vals)

    if cmd_x_sign == 1 and mean_lin < -TB4_ODOM_SIGN_THRESH:
        errs.append(
            f"cmd_vel forward command conflicts with mean odom velocity "
            f"{mean_lin:.3f}"
        )
    if cmd_x_sign == -1 and mean_lin > TB4_ODOM_SIGN_THRESH:
        errs.append(
            f"cmd_vel reverse command conflicts with mean odom velocity "
            f"{mean_lin:.3f}"
        )
    if cmd_z_sign == 1 and mean_ang < -TB4_ODOM_SIGN_THRESH:
        errs.append(
            f"cmd_vel left turn conflicts with mean angular odom velocity "
            f"{mean_ang:.3f}"
        )
    if cmd_z_sign == -1 and mean_ang > TB4_ODOM_SIGN_THRESH:
        errs.append(
            f"cmd_vel right turn conflicts with mean angular odom velocity "
            f"{mean_ang:.3f}"
        )

    # Feedback: |commanded - achieved| discrepancy. INC favors larger
    # divergence (more anomalous tracking) for exploration guidance.
    for fbk in feedback_list:
        if fbk.name == "cmd_odom_linear_agreement":
            fbk.update_value(abs(cmd_x - mean_lin))
        elif fbk.name == "cmd_odom_angular_agreement":
            fbk.update_value(abs(cmd_z - mean_ang))


def _cmd_velocity_samples(msg_list, state_dict):
    records = state_dict.get("/cmd_vel", [])
    msgs = [msg for _, msg in records] if records else msg_list
    samples = []
    for msg in msgs:
        try:
            cmd = _cmd_twist(msg)
            lin_x = float(getattr(cmd.linear, "x", 0.0))
            ang_z = float(getattr(cmd.angular, "z", 0.0))
        except (AttributeError, TypeError, ValueError):
            continue
        if math.isfinite(lin_x) and math.isfinite(ang_z):
            samples.append((lin_x, ang_z))
    return samples


def _check_tb4_command_velocity_envelope(msg_list, state_dict, errs,
                                         feedback_list):
    samples = _cmd_velocity_samples(msg_list, state_dict)
    if not samples:
        _update_feedback(feedback_list, "tb4_cmd_linear_velocity_ratio", 0.0)
        _update_feedback(feedback_list, "tb4_cmd_angular_velocity_ratio", 0.0)
        return

    max_lin_ratio = 0.0
    max_ang_ratio = 0.0
    max_lin = 0.0
    max_ang = 0.0
    for lin_x, ang_z in samples:
        lin_ratio = _limit_ratio(lin_x, TB4_MAX_LIN_VEL, TB4_MIN_LIN_VEL)
        ang_ratio = _limit_ratio(ang_z, TB4_MAX_ANG_VEL, TB4_MIN_ANG_VEL)
        if lin_ratio > max_lin_ratio:
            max_lin_ratio = lin_ratio
            max_lin = lin_x
        if ang_ratio > max_ang_ratio:
            max_ang_ratio = ang_ratio
            max_ang = ang_z

    _update_feedback(feedback_list, "tb4_cmd_linear_velocity_ratio",
                     max_lin_ratio)
    _update_feedback(feedback_list, "tb4_cmd_angular_velocity_ratio",
                     max_ang_ratio)

    if max_lin_ratio > TB4_VEL_RATIO_ERROR:
        errs.append(
            "TB4 cmd_vel linear velocity envelope violated: "
            f"linear.x={max_lin:.3f} m/s ratio={max_lin_ratio:.2f} "
            f"limit=[{TB4_MIN_LIN_VEL:.2f}, {TB4_MAX_LIN_VEL:.2f}]"
        )
    if max_ang_ratio > TB4_VEL_RATIO_ERROR:
        errs.append(
            "TB4 cmd_vel angular velocity envelope violated: "
            f"angular.z={max_ang:.3f} rad/s ratio={max_ang_ratio:.2f} "
            f"limit=[{TB4_MIN_ANG_VEL:.3f}, {TB4_MAX_ANG_VEL:.3f}]"
        )


def _has_usable_timebase(samples):
    if len(samples) < 2:
        return False

    positive_intervals = 0
    prev_t = samples[0][0]
    for cur_t, _, _ in samples[1:]:
        dt = cur_t - prev_t
        if dt < 0.0:
            return False
        if dt > 1e-6:
            positive_intervals += 1
        prev_t = cur_t

    return positive_intervals > 0


def _odom_velocity_samples(odom_list, prefer_header_stamp=False):
    bag_samples = []
    header_samples = []
    for ts, odom in odom_list:
        try:
            lin_x = float(odom.twist.twist.linear.x)
            ang_z = float(odom.twist.twist.angular.z)
            ts_sec = _ts_to_sec(ts)
        except (AttributeError, TypeError, ValueError):
            continue
        if math.isfinite(ts_sec) and math.isfinite(lin_x) and math.isfinite(ang_z):
            bag_samples.append((ts_sec, lin_x, ang_z))
            header_ts_sec = _odom_header_stamp_to_sec(odom)
            if header_ts_sec is not None:
                header_samples.append((header_ts_sec, lin_x, ang_z))

    if (
        prefer_header_stamp
        and len(header_samples) == len(bag_samples)
        and _has_usable_timebase(header_samples)
    ):
        header_samples.sort(key=lambda item: item[0])
        return header_samples

    bag_samples.sort(key=lambda item: item[0])
    return bag_samples


def _limit_ratio(value, positive_limit, negative_limit):
    limit = positive_limit if value >= 0.0 else abs(negative_limit)
    if limit <= 0.0:
        return 0.0
    return abs(value) / limit


def _check_tb4_velocity_envelope(odom_list, errs, feedback_list):
    samples = _odom_velocity_samples(odom_list)
    if not samples:
        _update_feedback(feedback_list, "tb4_odom_linear_velocity_ratio", 0.0)
        _update_feedback(feedback_list, "tb4_odom_angular_velocity_ratio", 0.0)
        return

    max_lin_ratio = 0.0
    max_ang_ratio = 0.0
    max_lin = 0.0
    max_ang = 0.0
    for _, lin_x, ang_z in samples:
        lin_ratio = _limit_ratio(lin_x, TB4_MAX_LIN_VEL, TB4_MIN_LIN_VEL)
        ang_ratio = _limit_ratio(ang_z, TB4_MAX_ANG_VEL, TB4_MIN_ANG_VEL)
        if lin_ratio > max_lin_ratio:
            max_lin_ratio = lin_ratio
            max_lin = lin_x
        if ang_ratio > max_ang_ratio:
            max_ang_ratio = ang_ratio
            max_ang = ang_z

    _update_feedback(feedback_list, "tb4_odom_linear_velocity_ratio",
                     max_lin_ratio)
    _update_feedback(feedback_list, "tb4_odom_angular_velocity_ratio",
                     max_ang_ratio)

    if max_lin_ratio > TB4_VEL_RATIO_ERROR:
        errs.append(
            "TB4 linear velocity envelope violated: "
            f"odom linear.x={max_lin:.3f} m/s ratio={max_lin_ratio:.2f} "
            f"limit=[{TB4_MIN_LIN_VEL:.2f}, {TB4_MAX_LIN_VEL:.2f}]"
        )
    if max_ang_ratio > TB4_VEL_RATIO_ERROR:
        errs.append(
            "TB4 angular velocity envelope violated: "
            f"odom angular.z={max_ang:.3f} rad/s ratio={max_ang_ratio:.2f} "
            f"limit=[{TB4_MIN_ANG_VEL:.3f}, {TB4_MAX_ANG_VEL:.3f}]"
        )


def _check_tb4_acceleration_envelope(odom_list, errs, feedback_list):
    samples = _odom_velocity_samples(odom_list, prefer_header_stamp=True)
    max_lin_ratio = 0.0
    max_ang_ratio = 0.0
    max_lin_acc = 0.0
    max_ang_acc = 0.0
    lin_run = 0
    ang_run = 0
    max_lin_run = 0
    max_ang_run = 0

    for (prev_t, prev_lin, prev_ang), (cur_t, cur_lin, cur_ang) in zip(
        samples, samples[1:]
    ):
        dt = cur_t - prev_t
        if dt <= 1e-6 or dt > 1.0:
            continue

        lin_acc = (cur_lin - prev_lin) / dt
        ang_acc = (cur_ang - prev_ang) / dt
        lin_ratio = abs(lin_acc) / TB4_MAX_LIN_ACCEL
        ang_ratio = _limit_ratio(
            ang_acc, TB4_MAX_ANG_ACCEL, TB4_MIN_ANG_ACCEL
        )

        if lin_ratio > max_lin_ratio:
            max_lin_ratio = lin_ratio
            max_lin_acc = lin_acc
        if ang_ratio > max_ang_ratio:
            max_ang_ratio = ang_ratio
            max_ang_acc = ang_acc

        if lin_ratio > TB4_ACCEL_RATIO_ERROR:
            lin_run += 1
            max_lin_run = max(max_lin_run, lin_run)
        else:
            lin_run = 0

        if ang_ratio > TB4_ACCEL_RATIO_ERROR:
            ang_run += 1
            max_ang_run = max(max_ang_run, ang_run)
        else:
            ang_run = 0

    _update_feedback(feedback_list, "tb4_linear_accel_ratio", max_lin_ratio)
    _update_feedback(feedback_list, "tb4_angular_accel_ratio", max_ang_ratio)

    lin_error = (
        max_lin_ratio >= TB4_ACCEL_STRONG_RATIO_ERROR
        or max_lin_run >= TB4_ACCEL_SUSTAINED_SAMPLES
    )
    ang_error = (
        max_ang_ratio >= TB4_ACCEL_STRONG_RATIO_ERROR
        or max_ang_run >= TB4_ACCEL_SUSTAINED_SAMPLES
    )

    if lin_error:
        errs.append(
            "TB4 linear acceleration envelope violated: "
            f"accel={max_lin_acc:.3f} m/s^2 ratio={max_lin_ratio:.2f} "
            f"limit={TB4_MAX_LIN_ACCEL:.3f}"
        )
    if ang_error:
        errs.append(
            "TB4 angular acceleration envelope violated: "
            f"accel={max_ang_acc:.3f} rad/s^2 ratio={max_ang_ratio:.2f} "
            f"limit=[{TB4_MIN_ANG_ACCEL:.3f}, {TB4_MAX_ANG_ACCEL:.3f}]"
        )


def _check_tb4_publish_gap(odom_list, errs, feedback_list):
    samples = _odom_velocity_samples(odom_list)
    max_gap = 0.0
    for (prev_t, _, _), (cur_t, _, _) in zip(samples, samples[1:]):
        gap = cur_t - prev_t
        if gap > max_gap:
            max_gap = gap

    _update_feedback(feedback_list, "tb4_odom_publish_gap", max_gap)


def _check_tb4_cmd_timeout(state_dict, odom_list, errs, feedback_list,
                           expected_cmd_count=0):
    cmd_records = state_dict.get("/cmd_vel", [])
    if not cmd_records:
        _update_feedback(feedback_list, "tb4_cmd_timeout_motion", 0.0)
        return

    cmd_times = []
    for ts, _ in cmd_records:
        try:
            ts_sec = _ts_to_sec(ts)
        except (TypeError, ValueError):
            continue
        if math.isfinite(ts_sec):
            cmd_times.append(ts_sec)
    if not cmd_times:
        _update_feedback(feedback_list, "tb4_cmd_timeout_motion", 0.0)
        return

    latest_cmd_ts = max(cmd_times)
    max_stale_motion = 0.0
    violation_samples = []
    for ts_sec, lin_x, ang_z in _odom_velocity_samples(odom_list):
        stale_by = ts_sec - latest_cmd_ts - TB4_CMD_TIMEOUT
        if stale_by <= 0.0:
            continue
        lin_excess = max(abs(lin_x) - TB4_STALE_MOTION_THRESH_LIN, 0.0)
        ang_excess = max(abs(ang_z) - TB4_STALE_MOTION_THRESH_ANG, 0.0)
        stale_motion = max(lin_excess, ang_excess)
        if stale_motion > max_stale_motion:
            max_stale_motion = stale_motion
        if stale_motion > 0.0 and stale_by >= TB4_CMD_TIMEOUT_ERROR_GRACE:
            violation_samples.append((stale_by, lin_x, ang_z, stale_motion))

    _update_feedback(feedback_list, "tb4_cmd_timeout_motion",
                     max_stale_motion)

    if expected_cmd_count > 1:
        # Timeout is sensitive to the final command timestamp. A high but
        # incomplete rosbag coverage (for example 7/8 samples) can miss the
        # last command and turn normal post-command motion into a false timeout.
        if len(cmd_records) < expected_cmd_count:
            return

    if len(violation_samples) >= TB4_CMD_TIMEOUT_ERROR_SAMPLES:
        stale_by, lin_x, ang_z, _ = max(
            violation_samples, key=lambda sample: sample[3])
        errs.append(
            "TB4 cmd_vel timeout violated: robot still moving "
            f"{stale_by:.3f}s after timeout "
            f"(odom linear.x={lin_x:.3f}, angular.z={ang_z:.3f})"
        )


def _wheel_velocity_samples(wheel_list):
    samples = []
    for ts, msg in wheel_list:
        try:
            left = float(msg.velocity_left)
            right = float(msg.velocity_right)
            ts_sec = _ts_to_sec(ts)
        except (AttributeError, TypeError, ValueError):
            continue
        if math.isfinite(ts_sec) and math.isfinite(left) and math.isfinite(right):
            samples.append((ts_sec, left, right))
    samples.sort(key=lambda item: item[0])
    return samples


def _check_tb4_wheel_odom_consistency(state_dict, odom_list, errs,
                                      feedback_list):
    wheel_samples = _wheel_velocity_samples(state_dict.get("/wheel_vels", []))
    odom_samples = _odom_velocity_samples(odom_list)
    if not wheel_samples or not odom_samples:
        _update_feedback(
            feedback_list, "tb4_wheel_odom_consistency_error", 0.0
        )
        return

    mean_wheel_forward = _mean([(left + right) / 2.0
                                for _, left, right in wheel_samples])
    mean_odom_linear = _mean([lin_x for _, lin_x, _ in odom_samples])
    wheel_sign = _consistent_sign(
        [(left + right) / 2.0 for _, left, right in wheel_samples],
        TB4_WHEEL_SIGN_THRESH,
    )
    odom_sign = _consistent_sign(
        [lin_x for _, lin_x, _ in odom_samples], TB4_ODOM_SIGN_THRESH
    )

    conflict = (
        wheel_sign is not None
        and odom_sign is not None
        and wheel_sign != 0
        and odom_sign != 0
        and wheel_sign != odom_sign
    )
    error_value = abs(mean_wheel_forward) + abs(mean_odom_linear) if conflict else 0.0
    _update_feedback(
        feedback_list, "tb4_wheel_odom_consistency_error", error_value
    )

    if conflict:
        errs.append(
            "TB4 wheel/odom direction conflict: "
            f"mean wheel velocity={mean_wheel_forward:.3f} rad/s, "
            f"mean odom linear.x={mean_odom_linear:.3f} m/s"
        )


def _check_turtlebot4_smoke(config, msg_list, state_dict, feedback_list):
    errs = []
    odom_list = state_dict.get("/odom", [])
    scan_list = state_dict.get("/scan", [])

    if not odom_list:
        errs.append("missing required TurtleBot4 topic /odom")
    # A missing scan is a sensor/readiness or recording-quality issue, not a
    # TurtleBot4 motion-control logic bug. Scan content is checked when present.

    _check_scan_sanity(scan_list, errs, feedback_list)
    _check_odom_sanity(odom_list, errs)
    _check_cmd_odom_consistency(msg_list, odom_list, errs, feedback_list)
    _check_tb4_command_velocity_envelope(msg_list, state_dict, errs,
                                         feedback_list)
    _check_tb4_velocity_envelope(odom_list, errs, feedback_list)
    _check_tb4_acceleration_envelope(odom_list, errs, feedback_list)
    _check_tb4_publish_gap(odom_list, errs, feedback_list)
    _check_tb4_cmd_timeout(
        state_dict, odom_list, errs, feedback_list, len(msg_list)
    )
    _check_tb4_wheel_odom_consistency(state_dict, odom_list, errs,
                                      feedback_list)

    # Optional Create3 topics: only sanity-check if present. Missing
    # optional topics are NOT treated as bugs (recorded as optional elsewhere).
    optional_topics = [
        "/hazard_detection",
        "/slip_status",
        "/stall_status",
        "/kidnap_status",
        "/wheel_status",
    ]
    for topic in optional_topics:
        for _, msg in state_dict.get(topic, []):
            if msg is None:
                errs.append(f"{topic} contains an empty message")
                break

    return errs


def check(config, msg_list, state_dict, feedback_list):
    errs = list()

    if getattr(config, "oracle_mode", "") == "turtlebot4_jazzy":
        return _check_turtlebot4_smoke(
            config, msg_list, state_dict, feedback_list
        )

    try:
        imu_list = state_dict["/imu"]
    except KeyError:
        print("[checker] no imu data available")
        imu_list = list()

    try:
        odom_list = state_dict["/odom"]
    except KeyError:
        print("[checker] no odom data available")
        odom_list = list()

    try:
        scan_list = state_dict["/scan"]
    except KeyError:
        print("[checker] no lidar data available")
        scan_list = list()

    imu_angles = list() # for deviation

    ts0 = imu_list[0][0]
    for (ts, imu) in imu_list:
        # Check NaN
        if math.isnan(imu.linear_acceleration.x):
            errs.append("imu.linear_acceleration.x is NaN")

        if math.isnan(imu.linear_acceleration.y):
            errs.append("imu.linear_acceleration.y is NaN")

        if math.isnan(imu.linear_acceleration.z):
            errs.append("imu.linear_acceleration.z is NaN")

        if math.isnan(imu.angular_velocity.x):
            errs.append("imu.angular_velocity.x is NaN")

        if math.isnan(imu.angular_velocity.y):
            errs.append("imu.angular_velocity.y is NaN")

        if math.isnan(imu.angular_velocity.z):
            errs.append("imu.angular_velocity.z is NaN")

        # Check INF
        if math.isinf(imu.linear_acceleration.x):
            errs.append("imu.linear_acceleration.x is INF")

        if math.isinf(imu.linear_acceleration.y):
            errs.append("imu.linear_acceleration.y is INF")

        if math.isinf(imu.linear_acceleration.z):
            errs.append("imu.linear_acceleration.z is INF")

        if math.isinf(imu.angular_velocity.x):
            errs.append("imu.angular_velocity.x is INF")

        if math.isinf(imu.angular_velocity.y):
            errs.append("imu.angular_velocity.y is INF")

        if math.isinf(imu.angular_velocity.z):
            errs.append("imu.angular_velocity.z is INF")

        # Check max
        if imu.angular_velocity.z > TB3_MAX_ANG_VEL + TB3_VEL_TOLERANCE:
            errs.append(f"imu.angular_velocity.z ({imu.angular_velocity.z}) exceeded max ({TB3_MAX_ANG_VEL})")
        if imu.angular_velocity.z < -(TB3_MAX_ANG_VEL + TB3_VEL_TOLERANCE):
            errs.append(f"imu.angular_velocity.z ({imu.angular_velocity.z}) exceeded min (-{TB3_MAX_ANG_VEL})")

        # 2D ground constraint for IMU roll/pitch
        if abs(imu.angular_velocity.x) > TB3_IMU_ROLL_PITCH_TOL:
            errs.append(f"imu.angular_velocity.x ({imu.angular_velocity.x}) violates ground constraint")
        if abs(imu.angular_velocity.y) > TB3_IMU_ROLL_PITCH_TOL:
            errs.append(f"imu.angular_velocity.y ({imu.angular_velocity.y}) violates ground constraint")

        # robot_pose_2 = np.arctan2(siny_cosp, cosy_cosp)
        imu_angle_ = np.arctan2(
            imu.orientation.x * imu.orientation.y + imu.orientation.w * imu.orientation.z,
            0.5 - imu.orientation.y * imu.orientation.y - imu.orientation.z * imu.orientation.z);
        imu_angles.append((ts, imu_angle_))

    robot_poses_odom = list()

    ts0 = odom_list[0][0]
    for (ts, odom) in odom_list:
        # Check NaN
        if math.isnan(odom.pose.pose.position.x):
            errs.append("odom.pose.pose.position.x is NaN")

        if math.isnan(odom.pose.pose.position.y):
            errs.append("odom.pose.pose.position.y is NaN")

        if math.isnan(odom.pose.pose.position.z):
            errs.append("odom.pose.pose.position.z is NaN")

        if math.isnan(odom.pose.pose.orientation.x):
            errs.append("odom.pose.pose.orientation.x is NaN")

        if math.isnan(odom.pose.pose.orientation.y):
            errs.append("odom.pose.pose.orientation.y is NaN")

        if math.isnan(odom.pose.pose.orientation.z):
            errs.append("odom.pose.pose.orientation.z is NaN")

        if math.isnan(odom.pose.pose.orientation.w):
            errs.append("odom.pose.pose.orientation.w is NaN")

        if math.isnan(odom.twist.twist.linear.x):
            errs.append("odom.twist.twist.linear.x is NaN")

        if math.isnan(odom.twist.twist.linear.y):
            errs.append("odom.twist.twist.linear.y is NaN")

        if math.isnan(odom.twist.twist.linear.z):
            errs.append("odom.twist.twist.linear.z is NaN")

        if math.isnan(odom.twist.twist.angular.x):
            errs.append("odom.twist.twist.angular.x is NaN")

        if math.isnan(odom.twist.twist.angular.y):
            errs.append("odom.twist.twist.angular.y is NaN")

        if math.isnan(odom.twist.twist.angular.z):
            errs.append("odom.twist.twist.angular.z is NaN")

        # Check INF
        if math.isinf(odom.pose.pose.position.x):
            errs.append("odom.pose.pose.position.x is INF")

        if math.isinf(odom.pose.pose.position.y):
            errs.append("odom.pose.pose.position.y is INF")

        if math.isinf(odom.pose.pose.position.z):
            errs.append("odom.pose.pose.position.z is INF")

        if math.isinf(odom.pose.pose.orientation.x):
            errs.append("odom.pose.pose.orientation.x is INF")

        if math.isinf(odom.pose.pose.orientation.y):
            errs.append("odom.pose.pose.orientation.y is INF")

        if math.isinf(odom.pose.pose.orientation.z):
            errs.append("odom.pose.pose.orientation.z is INF")

        if math.isinf(odom.pose.pose.orientation.w):
            errs.append("odom.pose.pose.orientation.w is INF")

        if math.isinf(odom.twist.twist.linear.x):
            errs.append("odom.twist.twist.linear.x is INF")

        if math.isinf(odom.twist.twist.linear.y):
            errs.append("odom.twist.twist.linear.y is INF")

        if math.isinf(odom.twist.twist.linear.z):
            errs.append("odom.twist.twist.linear.z is INF")

        if math.isinf(odom.twist.twist.angular.x):
            errs.append("odom.twist.twist.angular.x is INF")

        if math.isinf(odom.twist.twist.angular.y):
            errs.append("odom.twist.twist.angular.y is INF")

        if math.isinf(odom.twist.twist.angular.z):
            errs.append("odom.twist.twist.angular.z is INF")

        # check max (with tolerance for simulation numerical error)
        lin_vel = odom.twist.twist.linear.x
        if lin_vel > TB3_MAX_LIN_VEL + TB3_VEL_TOLERANCE:
            errs.append(f"linear velocity ({lin_vel}) exceeded max ({TB3_MAX_LIN_VEL})")

        ang_vel = odom.twist.twist.angular.z
        if ang_vel > TB3_MAX_ANG_VEL + TB3_VEL_TOLERANCE:
            errs.append(f"angular velocity ({ang_vel}) exceeded max ({TB3_MAX_ANG_VEL})")

        # check negative velocity bounds (symmetric hardware limits)
        if lin_vel < -(TB3_MAX_LIN_VEL + TB3_VEL_TOLERANCE):
            errs.append(f"linear velocity ({lin_vel}) exceeded min (-{TB3_MAX_LIN_VEL})")
        if ang_vel < -(TB3_MAX_ANG_VEL + TB3_VEL_TOLERANCE):
            errs.append(f"angular velocity ({ang_vel}) exceeded min (-{TB3_MAX_ANG_VEL})")

        # quaternion unit norm validation
        qx = odom.pose.pose.orientation.x
        qy = odom.pose.pose.orientation.y
        qz = odom.pose.pose.orientation.z
        qw = odom.pose.pose.orientation.w
        quat_norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if abs(quat_norm - 1.0) > TB3_QUAT_NORM_TOL:
            errs.append(f"odom quaternion norm ({quat_norm}) deviates from 1.0")

        # 2D ground robot constraints
        if abs(odom.pose.pose.position.z) > TB3_Z_POS_TOL:
            errs.append(f"odom.position.z ({odom.pose.pose.position.z}) violates ground constraint")
        if abs(odom.twist.twist.linear.y) > TB3_LATERAL_VEL_TOL:
            errs.append(f"odom.linear.y ({odom.twist.twist.linear.y}) violates diff-drive constraint")
        if abs(odom.twist.twist.linear.z) > TB3_LATERAL_VEL_TOL:
            errs.append(f"odom.linear.z ({odom.twist.twist.linear.z}) violates ground constraint")
        if abs(odom.twist.twist.angular.x) > TB3_ROLL_PITCH_TOL:
            errs.append(f"odom.angular.x ({odom.twist.twist.angular.x}) violates ground constraint")
        if abs(odom.twist.twist.angular.y) > TB3_ROLL_PITCH_TOL:
            errs.append(f"odom.angular.y ({odom.twist.twist.angular.y}) violates ground constraint")

        # rev-compute theta (check Odometry::publish())
        robot_pose_0 = odom.pose.pose.position.x
        robot_pose_1 = odom.pose.pose.position.y

        x = odom.pose.pose.orientation.x
        y = odom.pose.pose.orientation.y
        z = odom.pose.pose.orientation.z
        w = odom.pose.pose.orientation.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        robot_pose_2 = np.arctan2(siny_cosp, cosy_cosp)

        robot_pose_ = [robot_pose_0, robot_pose_1, robot_pose_2]
        robot_poses_odom.append((ts, robot_pose_2))

    last_theta = 0
    robot_poses_imu = []
    delta_theta = 0.0
    for (ts, theta) in imu_angles:
        delta_theta += theta - last_theta
        last_theta = theta
        robot_poses_imu.append((ts, delta_theta))

    # imu data is published approx. 4x faster than odom data
    # need to match the granularity
    theta_matched = list() # ts, odom_theta, imu_theta
    skip = 0
    for (ts_odom, odom) in robot_poses_odom:
        ts_diff = 9999999999999999999
        last_updated = 0
        updated = 0

        for i in range(skip, len(robot_poses_imu)):
            ts_imu = robot_poses_imu[i][0]
            imu = robot_poses_imu[i][1]

            ts_diff_last = abs(ts_odom - ts_imu)
            if ts_diff_last <= ts_diff:
                ts_diff = ts_diff_last
                last_updated = updated
                updated = 1
            else:
                last_updated = updated
                updated = 0

            if last_updated == 1 and updated == 0:
                skip = i
                break

        theta_matched.append((ts_odom, ts_imu, odom, imu))

    theta_diff = list()
    for tup in theta_matched:
        # Normalize angle difference to handle 2*pi wraparound
        raw_diff = abs(tup[2] - tup[3])
        normalized_diff = min(raw_diff, 2 * math.pi - raw_diff)
        theta_diff.append(normalized_diff)

    # todo: normalize diff, get anomaly and translate to feedback!!
    if theta_diff:
        max_diff = max(theta_diff)

        for feedback in feedback_list:
            if feedback.name == "theta_diff":
                feedback.update_value(max_diff)
                break

        if max_diff > 5.0:
            errs.append(f"theta estimation error is too huge: {max_diff}")

    # NOTE: LiDAR scan basic range check disabled due to known specification
    # violation bug in the LDS driver node (scan.range inf is always triggered).
    # See README for details. The temporal consistency check below still works.
    #
    # for (ts, scan) in scan_list:
    #     found_nan = False
    #     range_error = False
    #
    #     for scan_range in scan.ranges:
    #         if math.isnan(scan_range):
    #             found_nan = True
    #             break
    #
    #         elif scan_range < 0 or scan_range > 65.535:
    #             range_error = True
    #             break
    #
    #     if found_nan:
    #         errs.append("scan.range contains NaN")
    #
    #     if range_error:
    #         errs.append(f"scan.range {scan_range} is out of range")

    # ===================================================================
    # Deep Oracle Checks (derived from TurtleBot3 hardware specifications)
    # ===================================================================

    # --- Check: Acceleration Limits ---
    max_lin_accel_val = 0.0
    max_ang_accel_val = 0.0
    if len(odom_list) >= 2:
        for i in range(1, len(odom_list)):
            ts_prev = _ts_to_sec(odom_list[i-1][0])
            ts_curr = _ts_to_sec(odom_list[i][0])
            dt = ts_curr - ts_prev
            if dt <= 0:
                continue

            v_prev = odom_list[i-1][1].twist.twist.linear.x
            v_curr = odom_list[i][1].twist.twist.linear.x
            lin_accel = abs(v_curr - v_prev) / dt

            w_prev = odom_list[i-1][1].twist.twist.angular.z
            w_curr = odom_list[i][1].twist.twist.angular.z
            ang_accel = abs(w_curr - w_prev) / dt

            max_lin_accel_val = max(max_lin_accel_val, lin_accel)
            max_ang_accel_val = max(max_ang_accel_val, ang_accel)

            if lin_accel > TB3_MAX_LIN_ACCEL:
                errs.append(
                    f"linear acceleration ({lin_accel:.2f} m/s^2) "
                    f"exceeded physical limit ({TB3_MAX_LIN_ACCEL})")
            if ang_accel > TB3_MAX_ANG_ACCEL:
                errs.append(
                    f"angular acceleration ({ang_accel:.2f} rad/s^2) "
                    f"exceeded physical limit ({TB3_MAX_ANG_ACCEL})")

    for feedback in feedback_list:
        if feedback.name == "max_linear_accel":
            feedback.update_value(max_lin_accel_val)
        elif feedback.name == "max_angular_accel":
            feedback.update_value(max_ang_accel_val)

    # --- Check: Quaternion norm deviation feedback ---
    max_quat_dev = 0.0
    for (ts, odom) in odom_list:
        qx = odom.pose.pose.orientation.x
        qy = odom.pose.pose.orientation.y
        qz = odom.pose.pose.orientation.z
        qw = odom.pose.pose.orientation.w
        dev = abs(math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw) - 1.0)
        max_quat_dev = max(max_quat_dev, dev)

    for feedback in feedback_list:
        if feedback.name == "quat_norm_deviation":
            feedback.update_value(max_quat_dev)
            break

    # --- Check: Velocity-Position Consistency ---
    max_vel_pos_err = 0.0
    if len(odom_list) >= 2:
        for i in range(1, len(odom_list)):
            ts_prev = _ts_to_sec(odom_list[i-1][0])
            ts_curr = _ts_to_sec(odom_list[i][0])
            dt = ts_curr - ts_prev
            if dt <= 0 or dt > 1.0:
                continue

            odom_prev = odom_list[i-1][1]
            odom_curr = odom_list[i][1]

            vx = odom_prev.twist.twist.linear.x
            # Get theta from previous odom quaternion
            qz_p = odom_prev.pose.pose.orientation.z
            qw_p = odom_prev.pose.pose.orientation.w
            theta = 2.0 * np.arctan2(qz_p, qw_p)

            expected_dx = vx * math.cos(theta) * dt
            expected_dy = vx * math.sin(theta) * dt

            actual_dx = odom_curr.pose.pose.position.x - odom_prev.pose.pose.position.x
            actual_dy = odom_curr.pose.pose.position.y - odom_prev.pose.pose.position.y

            pos_err = math.sqrt(
                (expected_dx - actual_dx)**2 + (expected_dy - actual_dy)**2)
            max_vel_pos_err = max(max_vel_pos_err, pos_err)

            if pos_err > TB3_VEL_POS_TOL:
                errs.append(
                    f"velocity-position inconsistency ({pos_err:.4f} m) "
                    f"exceeded threshold ({TB3_VEL_POS_TOL})")

    for feedback in feedback_list:
        if feedback.name == "vel_pos_inconsistency":
            feedback.update_value(max_vel_pos_err)
            break

    # --- Check: IMU-Odometry Linear Acceleration Cross-Validation ---
    max_accel_diff = 0.0
    accel_diffs = []
    if len(odom_list) >= 2 and len(imu_list) >= 1:
        # Compute odom-derived accelerations with timestamps
        odom_accels = []
        for i in range(1, len(odom_list)):
            ts_prev = _ts_to_sec(odom_list[i-1][0])
            ts_curr = _ts_to_sec(odom_list[i][0])
            dt = ts_curr - ts_prev
            if dt <= 0:
                continue
            v_prev = odom_list[i-1][1].twist.twist.linear.x
            v_curr = odom_list[i][1].twist.twist.linear.x
            a_odom = (v_curr - v_prev) / dt
            ts_mid = (odom_list[i-1][0] + odom_list[i][0]) / 2
            odom_accels.append((ts_mid, a_odom))

        # Match IMU readings to odom acceleration timestamps
        for (ts_oa, a_odom) in odom_accels:
            best_imu_accel = None
            best_ts_diff = float('inf')
            for (ts_imu, imu_msg) in imu_list:
                td = abs(ts_oa - ts_imu)
                if td < best_ts_diff:
                    best_ts_diff = td
                    best_imu_accel = imu_msg.linear_acceleration.x
            if best_imu_accel is not None:
                diff = abs(best_imu_accel - a_odom)
                accel_diffs.append(diff)
                max_accel_diff = max(max_accel_diff, diff)

    if accel_diffs and statistics.mean(accel_diffs) > TB3_IMU_ODOM_ACCEL_TOL:
        errs.append(
            f"IMU-odom acceleration mean discrepancy "
            f"({statistics.mean(accel_diffs):.2f} m/s^2) "
            f"exceeded threshold ({TB3_IMU_ODOM_ACCEL_TOL})")

    for feedback in feedback_list:
        if feedback.name == "imu_odom_accel_diff":
            feedback.update_value(max_accel_diff)
            break

    # --- Check: LiDAR Temporal Consistency ---
    if len(scan_list) >= 2:
        for i in range(1, len(scan_list)):
            prev_ranges = scan_list[i-1][1].ranges
            curr_ranges = scan_list[i][1].ranges

            if len(prev_ranges) != len(curr_ranges):
                continue

            diffs = []
            for j in range(len(prev_ranges)):
                r_prev = prev_ranges[j]
                r_curr = curr_ranges[j]
                if (math.isnan(r_prev) or math.isnan(r_curr)
                        or math.isinf(r_prev) or math.isinf(r_curr)):
                    continue
                diffs.append(abs(r_curr - r_prev))

            if diffs:
                mean_diff = statistics.mean(diffs)
                if mean_diff > TB3_LIDAR_TEMPORAL_TOL:
                    errs.append(
                        f"LiDAR temporal inconsistency: mean beam change "
                        f"({mean_diff:.3f} m) exceeded threshold "
                        f"({TB3_LIDAR_TEMPORAL_TOL})")

    # --- Check: cmd_vel Tracking (commanded vs achieved velocity) ---
    # In sequence mode: check each command's tracking within its active window.
    # In single mode: check the last command in a 0.5~2.0s response window.
    try:
        cmd_vel_list = state_dict["/cmd_vel"]
    except KeyError:
        cmd_vel_list = []

    if not cmd_vel_list and msg_list:
        cmd_vel_list = [(0, msg) for msg in msg_list]

    if cmd_vel_list and len(odom_list) >= 2:
        # Build list of (timestamp, cmd) pairs for valid commands
        valid_cmds = []
        for (ts, cmd) in cmd_vel_list:
            cmd_lin = cmd.linear.x
            cmd_ang = cmd.angular.z
            if (abs(cmd_lin) <= TB3_MAX_LIN_VEL + TB3_VEL_TOLERANCE
                    and abs(cmd_ang) <= TB3_MAX_ANG_VEL + TB3_VEL_TOLERANCE):
                valid_cmds.append((_ts_to_sec(ts), cmd_lin, cmd_ang))

        # For each valid command, check tracking in its active window
        # (from 0.5s after it's sent until the next command arrives)
        for i, (cmd_ts, cmd_lin, cmd_ang) in enumerate(valid_cmds):
            # Determine end of this command's active window
            if i + 1 < len(valid_cmds):
                window_end = valid_cmds[i + 1][0]
            else:
                window_end = cmd_ts + 2.0

            # Collect odom samples in [cmd_ts+0.5, window_end]
            response_odom = []
            for (ts, odom) in odom_list:
                t = _ts_to_sec(ts)
                if cmd_ts + 0.5 <= t <= window_end:
                    response_odom.append(odom)

            if len(response_odom) >= 3:
                achieved_lin = statistics.mean(
                    [o.twist.twist.linear.x for o in response_odom])
                achieved_ang = statistics.mean(
                    [o.twist.twist.angular.z for o in response_odom])

                lin_track_err = abs(cmd_lin - achieved_lin)
                ang_track_err = abs(cmd_ang - achieved_ang)

                if lin_track_err > TB3_CMD_VEL_LIN_TOL:
                    errs.append(
                        f"cmd_vel tracking error: linear "
                        f"(cmd={cmd_lin:.3f}, achieved={achieved_lin:.3f}, "
                        f"err={lin_track_err:.3f} m/s)")
                if ang_track_err > TB3_CMD_VEL_ANG_TOL:
                    errs.append(
                        f"cmd_vel tracking error: angular "
                        f"(cmd={cmd_ang:.3f}, achieved={achieved_ang:.3f}, "
                        f"err={ang_track_err:.3f} rad/s)")

    return errs
