import math
import statistics


# =========================================================================
# PX4 Default Parameter Thresholds (used when not dynamically mutated)
# =========================================================================

PX4_DEFAULTS = {
    "MPC_XY_VEL_MAX": 12.0,       # m/s
    "MPC_Z_VEL_MAX_UP": 3.0,      # m/s
    "MPC_Z_VEL_MAX_DN": 1.5,      # m/s
    "MPC_VEL_MANUAL": 10.0,       # m/s
    "MPC_ACC_HOR": 3.0,           # m/s²
    "MPC_ACC_HOR_MAX": 5.0,       # m/s²
    "MPC_ACC_UP_MAX": 4.0,        # m/s²
    "MPC_ACC_DOWN_MAX": 3.0,      # m/s²
    "MPC_TILTMAX_AIR": 45.0,      # deg
    "MPC_MAN_TILT_MAX": 35.0,     # deg
    "MPC_MAN_Y_MAX": 150.0,       # deg/s
    "MPC_YAWRAUTO_MAX": 45.0,     # deg/s
    "MPC_JERK_MAX": 8.0,          # m/s³
    "MPC_JERK_AUTO": 4.0,         # m/s³
    "MC_ROLLRATE_MAX": 220.0,     # deg/s
    "MC_PITCHRATE_MAX": 220.0,    # deg/s
    "MC_YAWRATE_MAX": 200.0,      # deg/s
    "FD_FAIL_R": 60.0,            # deg - FailureDetector max roll
    "FD_FAIL_P": 60.0,            # deg - FailureDetector max pitch
    "WV_YRATE_MAX": 90.0,         # deg/s - Weathervane max yaw rate
    "LNDMC_ALT_MAX": -1.0,        # m - Max altitude (-1 = disabled)
}

# Simulation tolerances (to avoid false positives from numerical noise)
TOL_VEL = 0.5          # m/s
TOL_VEL_Z = 1.0        # m/s - vertical velocity needs larger tolerance (PID transient)
TOL_ACC = 1.5          # m/s² - raised from 1.0; PID transient overshoot is ~4%
TOL_TILT = 2.0         # deg
TOL_RATE = 5.0         # deg/s
TOL_JERK = 2.0         # m/s³
TOL_QUAT_NORM = 0.01
TOL_VEL_POS = 1.0      # m - raised from 0.1; GPS correction causes discrete position jumps
TOL_FD = 5.0           # deg - FailureDetector tolerance
TOL_WV_YRATE = 10.0    # deg/s - Weathervane yaw rate tolerance
TOL_ALT = 5.0          # m - Altitude tolerance
GROUND_DIST = 0.15     # m - threshold for ground contact filtering
GROUND_TS_WINDOW = 0.25e9  # nanoseconds

# Duration filter: ignore violations shorter than this (seconds)
MIN_VIOLATION_DURATION = 0.1  # 100ms - filters out normal PID transients


# =========================================================================
# Helper Functions
# =========================================================================

def _get_threshold(param_name, msg_list, config):
    """Return the effective threshold for a parameter.

    In PGFUZZ mode, if the currently mutated parameter matches param_name,
    use the mutated value as the threshold. Otherwise use the default.
    """
    default = PX4_DEFAULTS.get(param_name, 0.0)
    if getattr(config, "exp_pgfuzz", False) and msg_list:
        param = msg_list[0]
        if hasattr(param, 'param_name') and param.param_name == param_name:
            return param.value
    return default


def _is_on_ground(ts, filter_ts_list):
    """Check if a timestamp is within GROUND_TS_WINDOW of any ground-contact ts."""
    if not filter_ts_list:
        return False
    ts_diff = [abs(ts - f_ts) for f_ts in filter_ts_list]
    return min(ts_diff) <= GROUND_TS_WINDOW


def _quat_to_tilt_deg(q):
    """Compute tilt angle (degrees) from PX4 quaternion [w, x, y, z]."""
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    # cos(tilt) = component of body-z along world-z = 1 - 2*(qx²+qy²)
    cos_tilt = 1.0 - 2.0 * (qx * qx + qy * qy)
    cos_tilt = max(-1.0, min(1.0, cos_tilt))
    return math.degrees(math.acos(cos_tilt))


def _quat_to_euler_deg(q):
    """Compute roll and pitch (degrees) from PX4 quaternion [w, x, y, z]."""
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll_deg = abs(math.degrees(math.atan2(sinr_cosp, cosr_cosp)))
    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch_deg = abs(math.degrees(math.asin(sinp)))
    return roll_deg, pitch_deg


def _is_valid(value):
    """Check that a float value is neither NaN nor Inf."""
    return not (math.isnan(value) or math.isinf(value))


def _safe_get(state_dict, topic):
    """Safely retrieve a topic's data from state_dict."""
    try:
        return state_dict[topic]
    except KeyError:
        return []


def _finite_float(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(value) or math.isinf(value):
        return None
    return value


def _msg_scalar(msg, *field_names):
    for field_name in field_names:
        try:
            value = getattr(msg, field_name)
        except AttributeError:
            continue
        value = _finite_float(value)
        if value is not None:
            return value
    return None


def _msg_vector(msg, field_name, length=3):
    try:
        vector = getattr(msg, field_name)
    except AttributeError:
        return None

    values = []
    for idx in range(length):
        try:
            value = _finite_float(vector[idx])
        except (IndexError, KeyError, TypeError):
            return None
        if value is None:
            return None
        values.append(value)
    return values


def _nearest_msg(samples, ts, max_gap_ns=250_000_000):
    best = None
    best_gap = None
    for sample_ts, msg in samples:
        gap = abs(sample_ts - ts)
        if best_gap is None or gap < best_gap:
            best_gap = gap
            best = msg
    if best_gap is None or best_gap > max_gap_ns:
        return None
    return best


def _update_feedback(feedback_list, metrics):
    for feedback in feedback_list:
        if feedback.name in metrics and metrics[feedback.name] is not None:
            feedback.update_value(metrics[feedback.name])


# =========================================================================
# PX4 v1.17 Jazzy layered ULG oracle
# =========================================================================

PX4_V117_NAV_OFFBOARD = 14
PX4_V117_ARMED = 2
PX4_V117_COM_OF_LOSS_T = 1.0
PX4_V117_FAILSAFE_EXPLANATORY_FLAGS = (
    "offboard_control_signal_lost",
    "manual_control_signal_lost",
    "gcs_connection_lost",
    "local_position_invalid",
    "local_position_invalid_relaxed",
    "local_velocity_invalid",
    "local_altitude_invalid",
    "attitude_invalid",
    "angular_velocity_invalid",
    "global_position_invalid",
    "global_position_invalid_relaxed",
    "home_position_invalid",
    "battery_unhealthy",
    "geofence_breached",
    "fd_critical_failure",
    "fd_esc_arming_failure",
    "fd_imbalanced_prop",
    "mission_failure",
    "navigator_failure",
)


def _raw_finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(value) or math.isinf(value):
        return None
    return value


def _raw_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _raw_vector(msg, field_name, length=3):
    try:
        vector = getattr(msg, field_name)
    except AttributeError:
        return None

    values = []
    for idx in range(length):
        try:
            value = _raw_float(vector[idx])
        except (IndexError, KeyError, TypeError):
            return None
        if value is None:
            return None
        values.append(value)
    return values


def _all_nan(values):
    return bool(values) and all(math.isnan(value) for value in values)


def _all_finite(values):
    return bool(values) and all(
        not math.isnan(value) and not math.isinf(value)
        for value in values
    )


def _msg_bool(msg, field_name):
    try:
        return bool(getattr(msg, field_name))
    except AttributeError:
        return False


def _record_ts_ns(ts, msg):
    timestamp = _msg_scalar(msg, "timestamp")
    if timestamp is not None and timestamp > 0:
        # PX4 message timestamps are in microseconds; parser tuple timestamps
        # are nanoseconds. Keep both paths usable for synthetic unit tests.
        return int(timestamp * 1000)
    return int(ts)


def _sample_is_armed(status):
    return getattr(status, "arming_state", None) == PX4_V117_ARMED


def _sample_is_offboard(status):
    return getattr(status, "nav_state", None) == PX4_V117_NAV_OFFBOARD


def _status_failsafe(status):
    return bool(getattr(status, "failsafe", False))


def _failsafe_has_explanatory_flag(flags_msg):
    if flags_msg is None:
        return False
    return any(
        _msg_bool(flags_msg, field_name)
        for field_name in PX4_V117_FAILSAFE_EXPLANATORY_FLAGS
    )


def _heartbeat_fresh_at(hb_samples, ts_ns):
    if not hb_samples:
        return False
    max_gap_ns = int((PX4_V117_COM_OF_LOSS_T + 0.05) * 1e9)
    return min(abs(hb_ts - ts_ns) for hb_ts in hb_samples) <= max_gap_ns


def _smooth_velocity_samples(velocities):
    if len(velocities) < 3:
        return []
    smoothed = []
    for idx in range(1, len(velocities) - 1):
        prev = velocities[idx - 1]
        curr = velocities[idx]
        nxt = velocities[idx + 1]
        smoothed.append((
            curr[0],
            (prev[1] + curr[1] + nxt[1]) / 3.0,
            (prev[2] + curr[2] + nxt[2]) / 3.0,
            (prev[3] + curr[3] + nxt[3]) / 3.0,
        ))
    return smoothed


def _velocity_accels(velocities):
    accels = []
    for idx in range(1, len(velocities)):
        prev = velocities[idx - 1]
        curr = velocities[idx]
        dt = (curr[0] - prev[0]) / 1e9
        if dt <= 0.01 or dt > 0.5:
            continue
        accels.append((
            curr[0],
            (curr[1] - prev[1]) / dt,
            (curr[2] - prev[2]) / dt,
            (curr[3] - prev[3]) / dt,
        ))
    return accels


def _has_sustained_true(values, required=2):
    run = 0
    for value in values:
        if value:
            run += 1
            if run >= required:
                return True
        else:
            run = 0
    return False


def _offboard_mode_has_selector_fields(msg):
    return hasattr(msg, "velocity") and hasattr(msg, "position")


def _offboard_mode_is_velocity_only(msg):
    if not _offboard_mode_has_selector_fields(msg):
        return False
    return (
        _msg_bool(msg, "velocity")
        and not _msg_bool(msg, "position")
        and not _msg_bool(msg, "acceleration")
        and not _msg_bool(msg, "attitude")
        and not _msg_bool(msg, "body_rate")
        and not _msg_bool(msg, "thrust_and_torque")
        and not _msg_bool(msg, "direct_actuator")
    )


def _after_velocity_mode_start(metrics, ts, msg):
    start_ts = metrics.get("_velocity_mode_start_ns")
    if start_ts is None:
        return False
    return _record_ts_ns(ts, msg) >= start_ts


def collect_px4_v117_evidence(state_dict):
    return {
        "status": _safe_get(state_dict, "/VehicleStatus_PubSubTopic"),
        "local_position": _safe_get(
            state_dict, "/VehicleLocalPosition_PubSubTopic"
        ),
        "attitude": _safe_get(state_dict, "/VehicleAttitude_PubSubTopic"),
        "angular_velocity": _safe_get(
            state_dict, "/VehicleAngularVelocity_PubSubTopic"
        ),
        "odometry": _safe_get(state_dict, "/VehicleOdometry_PubSubTopic"),
        "control_mode": _safe_get(
            state_dict, "/VehicleControlMode_PubSubTopic"
        ),
        "command_ack": _safe_get(
            state_dict, "/VehicleCommandAck_PubSubTopic"
        ),
        "offboard_heartbeat": _safe_get(
            state_dict, "/OffboardControlMode_PubSubTopic"
        ),
        "failsafe_flags": _safe_get(state_dict, "/FailsafeFlags_PubSubTopic"),
        "trajectory_setpoint": _safe_get(
            state_dict, "/TrajectorySetpoint_PubSubTopic"
        ),
        "local_position_setpoint": _safe_get(
            state_dict, "/VehicleLocalPositionSetpoint_PubSubTopic"
        ),
        "attitude_setpoint": _safe_get(
            state_dict, "/VehicleAttitudeSetpoint_PubSubTopic"
        ),
        "rates_setpoint": _safe_get(
            state_dict, "/VehicleRatesSetpoint_PubSubTopic"
        ),
        "actuator_outputs": _safe_get(
            state_dict, "/ActuatorOutputs_PubSubTopic"
        ),
        "land_detected": _safe_get(
            state_dict, "/VehicleLandDetected_PubSubTopic"
        ),
    }


def _initial_px4_v117_metrics():
    metrics = {
        "max_xy_velocity": 0.0,
        "max_tilt_angle": 0.0,
        "max_angular_rate": 0.0,
        "max_jerk": 0.0,
        "vel_pos_inconsistency": 0.0,
        "max_altitude": 0.0,
        "combined_tilt_velocity": 0.0,
        "actuator_saturation": 0.0,
        "odom_pos_divergence": 0.0,
        "control_loop_gap": 0.0,
        "px4_v117_velocity_tracking_error": 0.0,
        "px4_v117_rate_tracking_error": 0.0,
        "px4_v117_setpoint_sample_count": 0.0,
        "px4_v117_state_sample_count": 0.0,
        "px4_v117_horizontal_speed_ratio": 0.0,
        "px4_v117_climb_speed_ratio": 0.0,
        "px4_v117_descent_speed_ratio": 0.0,
        "px4_v117_horizontal_accel_ratio": 0.0,
        "px4_v117_vertical_accel_ratio": 0.0,
        "px4_v117_jerk_ratio": 0.0,
        "px4_v117_tilt_ratio": 0.0,
        "px4_v117_roll_rate_ratio": 0.0,
        "px4_v117_pitch_rate_ratio": 0.0,
        "px4_v117_yaw_rate_ratio": 0.0,
        "px4_v117_offboard_heartbeat_gap_ratio": 0.0,
        "px4_v117_control_mode_cascade_pressure": 0.0,
        "px4_v117_failsafe_pressure": 0.0,
        "px4_v117_command_ack_anomaly_score": 0.0,
        "px4_v117_actuator_saturation_ratio": 0.0,
        "px4_v117_offboard_mode_mismatch_score": 0.0,
        "px4_v117_setpoint_contract_pressure": 0.0,
        "px4_v117_local_setpoint_contract_pressure": 0.0,
        "px4_v117_attitude_setpoint_pressure": 0.0,
        "px4_v117_quality_score": 0.0,
        "_velocity_mode_start_ns": None,
        "_armed_seen": False,
        "_offboard_seen": False,
        "_airborne_seen": False,
        "_landed_seen": False,
        "_status_failsafe_seen": False,
        "_horizontal_velocity_over": [],
        "_climb_velocity_over": [],
        "_descent_velocity_over": [],
        "_horizontal_accel_over": [],
        "_vertical_accel_over": [],
        "_jerk_over": [],
        "_tilt_over": [],
        "_rate_over": [],
        "_control_cascade_over": [],
        "_offboard_enabled_false_while_nav_offboard": [],
        "_heartbeat_gap_over": [],
        "_spurious_failsafe_over": [],
        "_offboard_mode_mismatch_over": [],
        "_trajectory_setpoint_contract_over": [],
        "_local_setpoint_contract_over": [],
        "_attitude_setpoint_invalid_over": [],
        "_actuator_saturation_duration": 0.0,
        "_command_ack_anomaly": False,
    }
    return metrics


def _append_state_invalid_errors(evidence, errs):
    for _, pos in evidence["local_position"]:
        for field_name in ("x", "y", "z", "vx", "vy", "vz", "heading"):
            if not hasattr(pos, field_name):
                continue
            try:
                value = float(getattr(pos, field_name))
            except (TypeError, ValueError):
                continue
            if math.isnan(value) or math.isinf(value):
                errs.append(
                    "px4_v117_state_invalid: "
                    f"vehicle_local_position.{field_name} is NaN/INF"
                )
        heading = _msg_scalar(pos, "heading")
        if heading is not None and abs(heading) > math.pi + 0.0001:
            errs.append(
                "px4_v117_state_invalid: "
                f"vehicle_local_position.heading {heading:.3f} outside [-pi, pi]"
            )

    for _, att in evidence["attitude"]:
        q = getattr(att, "q", None)
        if q is None:
            continue
        values = []
        invalid = False
        for idx in range(4):
            try:
                value = float(q[idx])
            except (IndexError, TypeError, ValueError):
                invalid = True
                break
            if math.isnan(value) or math.isinf(value):
                invalid = True
                break
            values.append(value)
        if invalid:
            errs.append("px4_v117_state_invalid: vehicle_attitude.q is NaN/INF")
            continue
        norm = math.sqrt(sum(value * value for value in values))
        if norm < 0.5 or norm > 1.5:
            errs.append(
                "px4_v117_state_invalid: "
                f"vehicle_attitude.q has implausible norm {norm:.3f}"
            )

    for _, odom in evidence["odometry"]:
        for field_name in ("x", "y", "z"):
            if not hasattr(odom, field_name):
                continue
            try:
                value = float(getattr(odom, field_name))
            except (TypeError, ValueError):
                continue
            if math.isnan(value) or math.isinf(value):
                errs.append(
                    "px4_v117_state_invalid: "
                    f"vehicle_odometry.{field_name} is NaN/INF"
                )


def _compute_px4_v117_local_position_metrics(evidence, metrics):
    local_position_list = evidence["local_position"]
    velocities = []
    prev_ts = None
    prev_pos = None
    for ts, pos in local_position_list:
        x = _msg_scalar(pos, "x")
        y = _msg_scalar(pos, "y")
        z = _msg_scalar(pos, "z")
        vx = _msg_scalar(pos, "vx")
        vy = _msg_scalar(pos, "vy")
        vz = _msg_scalar(pos, "vz")
        if z is not None:
            altitude = abs(z)
            metrics["max_altitude"] = max(metrics["max_altitude"], altitude)
            if altitude >= 1.0:
                metrics["_airborne_seen"] = True
        if vx is not None and vy is not None:
            xy_vel = math.sqrt(vx * vx + vy * vy)
            metrics["max_xy_velocity"] = max(
                metrics["max_xy_velocity"], xy_vel
            )
            ratio = xy_vel / PX4_DEFAULTS["MPC_XY_VEL_MAX"]
            metrics["px4_v117_horizontal_speed_ratio"] = max(
                metrics["px4_v117_horizontal_speed_ratio"], ratio
            )
            metrics["_horizontal_velocity_over"].append(
                xy_vel > PX4_DEFAULTS["MPC_XY_VEL_MAX"] + TOL_VEL
            )
        if vz is not None:
            climb = max(-vz, 0.0)
            descent = max(vz, 0.0)
            metrics["px4_v117_climb_speed_ratio"] = max(
                metrics["px4_v117_climb_speed_ratio"],
                climb / PX4_DEFAULTS["MPC_Z_VEL_MAX_UP"],
            )
            metrics["px4_v117_descent_speed_ratio"] = max(
                metrics["px4_v117_descent_speed_ratio"],
                descent / PX4_DEFAULTS["MPC_Z_VEL_MAX_DN"],
            )
            metrics["_climb_velocity_over"].append(
                climb > PX4_DEFAULTS["MPC_Z_VEL_MAX_UP"] + 0.5
            )
            metrics["_descent_velocity_over"].append(
                descent > PX4_DEFAULTS["MPC_Z_VEL_MAX_DN"] + 0.5
            )
        if None not in (vx, vy, vz):
            velocities.append((ts, vx, vy, vz))
        if prev_ts is not None and prev_pos is not None:
            dt = (ts - prev_ts) / 1e9
            if 0.0 < dt <= 1.0:
                pvx = _msg_scalar(prev_pos, "vx")
                pvy = _msg_scalar(prev_pos, "vy")
                pvz = _msg_scalar(prev_pos, "vz")
                px = _msg_scalar(prev_pos, "x")
                py = _msg_scalar(prev_pos, "y")
                pz = _msg_scalar(prev_pos, "z")
                if None not in (pvx, pvy, pvz, px, py, pz, x, y, z):
                    expected = [px + pvx * dt, py + pvy * dt, pz + pvz * dt]
                    actual = [x, y, z]
                    err = math.sqrt(sum(
                        (expected[i] - actual[i]) ** 2 for i in range(3)
                    ))
                    metrics["vel_pos_inconsistency"] = max(
                        metrics["vel_pos_inconsistency"], err
                    )
            if dt > 0:
                metrics["control_loop_gap"] = max(
                    metrics["control_loop_gap"], dt
                )
        prev_ts = ts
        prev_pos = pos

    accels = _velocity_accels(velocities)
    for _, ax, ay, az in accels:
        hor_acc = math.sqrt(ax * ax + ay * ay)
        vert_acc = abs(az)
        metrics["px4_v117_horizontal_accel_ratio"] = max(
            metrics["px4_v117_horizontal_accel_ratio"],
            hor_acc / PX4_DEFAULTS["MPC_ACC_HOR"],
        )
        metrics["px4_v117_vertical_accel_ratio"] = max(
            metrics["px4_v117_vertical_accel_ratio"],
            max(
                max(-az, 0.0) / PX4_DEFAULTS["MPC_ACC_UP_MAX"],
                max(az, 0.0) / PX4_DEFAULTS["MPC_ACC_DOWN_MAX"],
            ),
        )
        metrics["_horizontal_accel_over"].append(
            hor_acc / PX4_DEFAULTS["MPC_ACC_HOR_MAX"] > 1.10
        )
        metrics["_vertical_accel_over"].append(
            vert_acc / min(
                PX4_DEFAULTS["MPC_ACC_UP_MAX"],
                PX4_DEFAULTS["MPC_ACC_DOWN_MAX"],
            ) > 1.10
        )

    for idx in range(1, len(accels)):
        prev = accels[idx - 1]
        curr = accels[idx]
        dt = (curr[0] - prev[0]) / 1e9
        if dt <= 0.01 or dt > 0.5:
            continue
        jerk = math.sqrt(sum(
            ((curr[i] - prev[i]) / dt) ** 2 for i in range(1, 4)
        ))
        metrics["max_jerk"] = max(metrics["max_jerk"], jerk)
        ratio = jerk / PX4_DEFAULTS["MPC_JERK_MAX"]
        metrics["px4_v117_jerk_ratio"] = max(
            metrics["px4_v117_jerk_ratio"], ratio
        )

    smoothed_accels = _velocity_accels(_smooth_velocity_samples(velocities))
    for idx in range(1, len(smoothed_accels)):
        prev = smoothed_accels[idx - 1]
        curr = smoothed_accels[idx]
        dt = (curr[0] - prev[0]) / 1e9
        if dt <= 0.01 or dt > 0.5:
            continue
        jerk = math.sqrt(sum(
            ((curr[i] - prev[i]) / dt) ** 2 for i in range(1, 4)
        ))
        ratio = jerk / PX4_DEFAULTS["MPC_JERK_MAX"]
        metrics["_jerk_over"].append(ratio > 1.20)


def _compute_px4_v117_attitude_metrics(evidence, metrics):
    for _, att in evidence["attitude"]:
        q = _msg_vector(att, "q", length=4)
        if q is None:
            continue
        tilt = _quat_to_tilt_deg(q)
        metrics["max_tilt_angle"] = max(metrics["max_tilt_angle"], tilt)
        ratio = tilt / PX4_DEFAULTS["MPC_TILTMAX_AIR"]
        metrics["px4_v117_tilt_ratio"] = max(
            metrics["px4_v117_tilt_ratio"], ratio
        )
        metrics["_tilt_over"].append(ratio > 1.05)

    rate_samples = []
    for ts, ang in evidence["angular_velocity"]:
        xyz = _msg_vector(ang, "xyz", length=3)
        if xyz is not None:
            rate_samples.append((ts, xyz))
    if not rate_samples:
        for ts, odom in evidence["odometry"]:
            xyz = _msg_vector(odom, "angular_velocity", length=3)
            if xyz is not None:
                rate_samples.append((ts, xyz))

    for _, xyz in rate_samples:
        roll = abs(xyz[0]) * 180.0 / math.pi
        pitch = abs(xyz[1]) * 180.0 / math.pi
        yaw = abs(xyz[2]) * 180.0 / math.pi
        metrics["max_angular_rate"] = max(
            metrics["max_angular_rate"], roll, pitch, yaw
        )
        roll_ratio = roll / PX4_DEFAULTS["MC_ROLLRATE_MAX"]
        pitch_ratio = pitch / PX4_DEFAULTS["MC_PITCHRATE_MAX"]
        yaw_ratio = yaw / PX4_DEFAULTS["MC_YAWRATE_MAX"]
        metrics["px4_v117_roll_rate_ratio"] = max(
            metrics["px4_v117_roll_rate_ratio"], roll_ratio
        )
        metrics["px4_v117_pitch_rate_ratio"] = max(
            metrics["px4_v117_pitch_rate_ratio"], pitch_ratio
        )
        metrics["px4_v117_yaw_rate_ratio"] = max(
            metrics["px4_v117_yaw_rate_ratio"], yaw_ratio
        )
        metrics["_rate_over"].append(
            roll_ratio > 1.05 or pitch_ratio > 1.05 or yaw_ratio > 1.05
        )


def _compute_px4_v117_mode_metrics(evidence, metrics):
    status_by_ts = evidence["status"]
    hb_records = sorted(
        (
            (_record_ts_ns(ts, hb), ts, hb)
            for ts, hb in evidence["offboard_heartbeat"]
        ),
        key=lambda record: record[0],
    )
    hb_samples = [record[0] for record in hb_records]

    for hb_ts, _, hb in hb_records:
        if metrics["_velocity_mode_start_ns"] is None:
            if _offboard_mode_is_velocity_only(hb):
                metrics["_velocity_mode_start_ns"] = hb_ts
            continue
        if _offboard_mode_has_selector_fields(hb):
            mismatch = not _offboard_mode_is_velocity_only(hb)
            metrics["_offboard_mode_mismatch_over"].append(mismatch)
            if mismatch:
                metrics["px4_v117_offboard_mode_mismatch_score"] = 1.0

    for ts, status in status_by_ts:
        if _sample_is_armed(status):
            metrics["_armed_seen"] = True
        if _sample_is_offboard(status):
            metrics["_offboard_seen"] = True
        if _status_failsafe(status):
            metrics["_status_failsafe_seen"] = True
            metrics["px4_v117_failsafe_pressure"] = 1.0
            status_ts = _record_ts_ns(ts, status)
            flags_msg = _nearest_msg(
                evidence["failsafe_flags"],
                status_ts,
                max_gap_ns=500_000_000,
            )
            metrics["_spurious_failsafe_over"].append(
                _sample_is_offboard(status)
                and _heartbeat_fresh_at(hb_samples, status_ts)
                and not _failsafe_has_explanatory_flag(flags_msg)
            )

    for _, landed in evidence["land_detected"]:
        if _msg_bool(landed, "landed"):
            metrics["_landed_seen"] = True

    for ts, control in evidence["control_mode"]:
        pos_enabled = _msg_bool(control, "flag_control_position_enabled")
        vel_enabled = _msg_bool(control, "flag_control_velocity_enabled")
        att_enabled = _msg_bool(control, "flag_control_attitude_enabled")
        cascade_bad = (pos_enabled and not vel_enabled) or (
            vel_enabled and not att_enabled
        )
        metrics["_control_cascade_over"].append(cascade_bad)
        if cascade_bad:
            metrics["px4_v117_control_mode_cascade_pressure"] = 1.0

        status = _nearest_msg(status_by_ts, ts)
        if status is not None and _sample_is_offboard(status):
            offboard_enabled = _msg_bool(
                control, "flag_control_offboard_enabled"
            )
            metrics["_offboard_enabled_false_while_nav_offboard"].append(
                not offboard_enabled
            )

    for idx in range(1, len(hb_samples)):
        gap = (hb_samples[idx] - hb_samples[idx - 1]) / 1e9
        if gap <= 0:
            continue
        ratio = gap / PX4_V117_COM_OF_LOSS_T
        metrics["px4_v117_offboard_heartbeat_gap_ratio"] = max(
            metrics["px4_v117_offboard_heartbeat_gap_ratio"], ratio
        )
        metrics["_heartbeat_gap_over"].append(
            gap > PX4_V117_COM_OF_LOSS_T + 0.05
        )

    for _, ack in evidence["command_ack"]:
        result = _msg_scalar(ack, "result")
        if result is not None and int(result) != 0:
            metrics["_command_ack_anomaly"] = True
            metrics["px4_v117_command_ack_anomaly_score"] = 1.0


def _compute_px4_v117_setpoint_contract_metrics(evidence, metrics):
    for ts, setpoint in evidence["trajectory_setpoint"]:
        if not _after_velocity_mode_start(metrics, ts, setpoint):
            continue
        violation = False

        position = _raw_vector(setpoint, "position", length=3)
        if position is not None and not _all_nan(position):
            violation = True

        acceleration = _raw_vector(setpoint, "acceleration", length=3)
        if acceleration is not None and not _all_nan(acceleration):
            violation = True

        velocity = _raw_vector(setpoint, "velocity", length=3)
        if velocity is not None and not _all_finite(velocity):
            violation = True

        yaw = _raw_float(getattr(setpoint, "yaw", None))
        if yaw is not None:
            if math.isnan(yaw) or math.isinf(yaw) or abs(yaw) > math.pi + 0.01:
                violation = True

        yawspeed = _raw_float(getattr(setpoint, "yawspeed", None))
        if yawspeed is not None:
            if math.isnan(yawspeed) or math.isinf(yawspeed):
                violation = True

        metrics["_trajectory_setpoint_contract_over"].append(violation)
        if violation:
            metrics["px4_v117_setpoint_contract_pressure"] = 1.0

    for ts, setpoint in evidence["local_position_setpoint"]:
        if not _after_velocity_mode_start(metrics, ts, setpoint):
            continue
        violation = False

        position = [
            _raw_float(getattr(setpoint, field_name, None))
            for field_name in ("x", "y", "z")
        ]
        if all(value is not None for value in position):
            if not _all_nan(position):
                violation = True

        velocity = [
            _raw_float(getattr(setpoint, field_name, None))
            for field_name in ("vx", "vy", "vz")
        ]
        if all(value is not None for value in velocity):
            if not _all_finite(velocity):
                violation = True

        yaw = _raw_float(getattr(setpoint, "yaw", None))
        if yaw is not None:
            if math.isnan(yaw) or math.isinf(yaw) or abs(yaw) > math.pi + 0.01:
                violation = True

        yawspeed = _raw_float(getattr(setpoint, "yawspeed", None))
        if yawspeed is not None:
            if math.isnan(yawspeed) or math.isinf(yawspeed):
                violation = True

        metrics["_local_setpoint_contract_over"].append(violation)
        if violation:
            metrics["px4_v117_local_setpoint_contract_pressure"] = 1.0


def _compute_px4_v117_attitude_setpoint_metrics(evidence, metrics):
    yawrate_limit = math.radians(PX4_DEFAULTS["MC_YAWRATE_MAX"])
    for ts, setpoint in evidence["attitude_setpoint"]:
        if not _after_velocity_mode_start(metrics, ts, setpoint):
            continue
        violation = False

        q_d = _raw_vector(setpoint, "q_d", length=4)
        if q_d is not None:
            if not _all_finite(q_d):
                violation = True
            else:
                norm = math.sqrt(sum(value * value for value in q_d))
                if norm < 0.8 or norm > 1.2:
                    violation = True

        thrust = _raw_vector(setpoint, "thrust_body", length=3)
        if thrust is not None:
            if not _all_finite(thrust):
                violation = True
            elif any(abs(value) > 1.0 + 1e-3 for value in thrust):
                violation = True

        yaw_sp_move_rate = _raw_float(
            getattr(setpoint, "yaw_sp_move_rate", None)
        )
        if yaw_sp_move_rate is not None:
            if math.isnan(yaw_sp_move_rate) or math.isinf(yaw_sp_move_rate):
                violation = True
            elif abs(yaw_sp_move_rate) > yawrate_limit * 1.05:
                violation = True

        metrics["_attitude_setpoint_invalid_over"].append(violation)
        if violation:
            metrics["px4_v117_attitude_setpoint_pressure"] = 1.0


def _compute_px4_v117_tracking_metrics(evidence, metrics):
    local_position_list = evidence["local_position"]
    angular_velocity_list = evidence["angular_velocity"]
    for ts, setpoint in evidence["trajectory_setpoint"]:
        desired = _msg_vector(setpoint, "velocity", length=3)
        if desired is None:
            continue
        actual_msg = _nearest_msg(local_position_list, ts)
        if actual_msg is None:
            continue
        actual = [
            _msg_scalar(actual_msg, "vx"),
            _msg_scalar(actual_msg, "vy"),
            _msg_scalar(actual_msg, "vz"),
        ]
        if any(value is None for value in actual):
            continue
        err = math.sqrt(sum((desired[i] - actual[i]) ** 2 for i in range(3)))
        metrics["px4_v117_velocity_tracking_error"] = max(
            metrics["px4_v117_velocity_tracking_error"], err
        )

    for ts, setpoint in evidence["rates_setpoint"]:
        desired = [
            _msg_scalar(setpoint, "roll", "roll_body"),
            _msg_scalar(setpoint, "pitch", "pitch_body"),
            _msg_scalar(setpoint, "yaw", "yaw_body"),
        ]
        if any(value is None for value in desired):
            continue
        actual_msg = _nearest_msg(angular_velocity_list, ts)
        actual = _msg_vector(actual_msg, "xyz", length=3) if actual_msg else None
        if actual is None:
            continue
        err = math.sqrt(sum((desired[i] - actual[i]) ** 2 for i in range(3)))
        metrics["px4_v117_rate_tracking_error"] = max(
            metrics["px4_v117_rate_tracking_error"], err
        )


def _compute_px4_v117_actuator_metrics(evidence, metrics):
    saturation_start_ts = None
    saturation_kind = None
    for ts, act in evidence["actuator_outputs"]:
        outputs = _msg_vector(act, "output", length=4)
        if outputs is None:
            saturation_start_ts = None
            saturation_kind = None
            continue
        all_max = all(output >= 1900.0 for output in outputs)
        all_min = all(output <= 1100.0 for output in outputs)
        if not (all_max or all_min):
            saturation_start_ts = None
            saturation_kind = None
            continue
        kind = "MAX" if all_max else "MIN"
        if saturation_start_ts is None or saturation_kind != kind:
            saturation_start_ts = ts
            saturation_kind = kind
            continue
        duration = (ts - saturation_start_ts) / 1e9
        if kind == "MAX":
            metrics["actuator_saturation"] = max(
                metrics["actuator_saturation"], duration
            )
            metrics["_actuator_saturation_duration"] = max(
                metrics["_actuator_saturation_duration"], duration
            )
            metrics["px4_v117_actuator_saturation_ratio"] = max(
                metrics["px4_v117_actuator_saturation_ratio"],
                duration / 0.2,
            )


def compute_px4_v117_metrics(config, evidence):
    metrics = _initial_px4_v117_metrics()
    metrics["px4_v117_state_sample_count"] = float(
        len(evidence["local_position"]) + len(evidence["attitude"])
    )
    metrics["px4_v117_setpoint_sample_count"] = float(
        len(evidence["trajectory_setpoint"]) + len(evidence["rates_setpoint"])
    )
    _compute_px4_v117_mode_metrics(evidence, metrics)
    _compute_px4_v117_local_position_metrics(evidence, metrics)
    _compute_px4_v117_attitude_metrics(evidence, metrics)
    _compute_px4_v117_setpoint_contract_metrics(evidence, metrics)
    _compute_px4_v117_attitude_setpoint_metrics(evidence, metrics)
    _compute_px4_v117_tracking_metrics(evidence, metrics)
    _compute_px4_v117_actuator_metrics(evidence, metrics)
    metrics["combined_tilt_velocity"] = (
        metrics["max_tilt_angle"] * metrics["max_xy_velocity"]
    )
    quality = 0.0
    if metrics["_armed_seen"]:
        quality += 0.25
    if metrics["_offboard_seen"]:
        quality += 0.25
    if metrics["_airborne_seen"]:
        quality += 0.25
    if metrics["px4_v117_state_sample_count"] >= 3.0:
        quality += 0.25
    metrics["px4_v117_quality_score"] = quality
    return metrics


def classify_px4_v117_errors(config, evidence, metrics):
    errs = []
    _append_state_invalid_errors(evidence, errs)

    offboard_airborne = (
        metrics["_armed_seen"]
        and metrics["_offboard_seen"]
        and metrics["_airborne_seen"]
    )
    if not offboard_airborne:
        return errs

    if _has_sustained_true(metrics["_horizontal_velocity_over"]):
        errs.append(
            "px4_v117_velocity_bound_violation: horizontal "
            f"ratio={metrics['px4_v117_horizontal_speed_ratio']:.2f}"
        )
    if _has_sustained_true(metrics["_climb_velocity_over"]):
        errs.append(
            "px4_v117_velocity_bound_violation: climb "
            f"ratio={metrics['px4_v117_climb_speed_ratio']:.2f}"
        )
    if _has_sustained_true(metrics["_descent_velocity_over"]):
        errs.append(
            "px4_v117_velocity_bound_violation: descent "
            f"ratio={metrics['px4_v117_descent_speed_ratio']:.2f}"
        )

    if (
        len(evidence["local_position"]) >= 5
        and _has_sustained_true(metrics["_horizontal_accel_over"])
    ):
        errs.append(
            "px4_v117_accel_bound_violation: horizontal "
            f"ratio={metrics['px4_v117_horizontal_accel_ratio']:.2f}"
        )
    if (
        len(evidence["local_position"]) >= 5
        and _has_sustained_true(metrics["_vertical_accel_over"])
    ):
        errs.append(
            "px4_v117_accel_bound_violation: vertical "
            f"ratio={metrics['px4_v117_vertical_accel_ratio']:.2f}"
        )
    # MPC_JERK_MAX is mode-gated in the PX4 parameter reference. For the
    # current Offboard velocity-setpoint profile, position/acceleration are
    # intentionally NaN and the input can contain external velocity steps.
    # Keep jerk as feedback pressure, but do not classify jerk-only response
    # as a high-confidence target bug.

    if _has_sustained_true(metrics["_tilt_over"]):
        errs.append(
            "px4_v117_attitude_rate_violation: tilt "
            f"ratio={metrics['px4_v117_tilt_ratio']:.2f}"
        )
    if _has_sustained_true(metrics["_rate_over"]):
        axis, max_rate_ratio = max(
            (
                ("roll", metrics["px4_v117_roll_rate_ratio"]),
                ("pitch", metrics["px4_v117_pitch_rate_ratio"]),
                ("yaw", metrics["px4_v117_yaw_rate_ratio"]),
            ),
            key=lambda item: item[1],
        )
        errs.append(
            f"px4_v117_attitude_rate_violation: angular_rate axis={axis} "
            f"ratio={max_rate_ratio:.2f}"
        )

    if any(metrics["_control_cascade_over"]):
        errs.append(
            "px4_v117_control_mode_cascade_violation: "
            "higher-level control enabled without lower-level cascade"
        )

    if _has_sustained_true(
        metrics["_offboard_mode_mismatch_over"], required=1
    ):
        errs.append(
            "px4_v117_offboard_input_contract_violation: "
            "expected velocity-only OffboardControlMode during fuzz phase"
        )

    if _has_sustained_true(
        metrics["_trajectory_setpoint_contract_over"], required=1
    ):
        errs.append(
            "px4_v117_setpoint_contract_violation: velocity mode "
            "TrajectorySetpoint position/acceleration/yaw contract broken"
        )

    if _has_sustained_true(
        metrics["_local_setpoint_contract_over"], required=1
    ):
        errs.append(
            "px4_v117_setpoint_contract_violation: velocity mode "
            "VehicleLocalPositionSetpoint position/velocity contract broken"
        )

    if _has_sustained_true(
        metrics["_attitude_setpoint_invalid_over"], required=1
    ):
        errs.append(
            "px4_v117_attitude_setpoint_invalid: generated attitude setpoint "
            "has invalid quaternion, thrust, or yaw-rate command"
        )

    heartbeat_observed = bool(evidence["offboard_heartbeat"])
    if (
        heartbeat_observed
        and _has_sustained_true(metrics["_heartbeat_gap_over"], required=1)
        and not metrics["_status_failsafe_seen"]
    ):
        errs.append(
            "px4_v117_offboard_state_contradiction: heartbeat gap "
            f"ratio={metrics['px4_v117_offboard_heartbeat_gap_ratio']:.2f} "
            "while nav_state remained OFFBOARD"
        )
    if _has_sustained_true(
        metrics["_offboard_enabled_false_while_nav_offboard"], required=1
    ):
        errs.append(
            "px4_v117_offboard_state_contradiction: nav_state OFFBOARD but "
            "flag_control_offboard_enabled is false"
        )

    if _has_sustained_true(metrics["_spurious_failsafe_over"], required=1):
        errs.append(
            "px4_v117_failsafe_anomaly: spurious failsafe in healthy "
            "Offboard evidence window"
        )

    if metrics["_command_ack_anomaly"]:
        errs.append(
            "px4_v117_command_ack_anomaly: command ack rejected during "
            "Offboard evidence window"
        )

    if (
        metrics["_actuator_saturation_duration"] >= 0.2
        and not metrics["_landed_seen"]
    ):
        errs.append(
            "px4_v117_actuator_saturation: all motors saturated for "
            f"{metrics['_actuator_saturation_duration']:.3f}s "
            f"ratio={metrics['px4_v117_actuator_saturation_ratio']:.2f}"
        )

    return errs


def _px4_v117_ulg_metrics(config, state_dict):
    evidence = collect_px4_v117_evidence(state_dict)
    metrics = compute_px4_v117_metrics(config, evidence)
    errs = classify_px4_v117_errors(config, evidence, metrics)
    return metrics, errs


def _check_px4_v117_smoke(config, msg_list, state_dict, feedback_list):
    errs = []
    status_list = _safe_get(state_dict, "/VehicleStatus_PubSubTopic")
    local_position_list = _safe_get(
        state_dict, "/VehicleLocalPosition_PubSubTopic"
    )
    attitude_list = _safe_get(state_dict, "/VehicleAttitude_PubSubTopic")

    if not status_list:
        errs.append(
            "px4_v117_transport_missing: /fmu/out/vehicle_status_v1"
        )
    if not local_position_list:
        errs.append(
            "px4_v117_transport_missing: "
            "/fmu/out/vehicle_local_position_v1"
        )
    if not attitude_list:
        errs.append(
            "px4_v117_transport_missing: /fmu/out/vehicle_attitude"
        )

    finite_position_seen = False
    for _, msg in local_position_list:
        values = []
        for attr in ("x", "y", "z", "vx", "vy", "vz"):
            if hasattr(msg, attr):
                values.append(getattr(msg, attr))
        if values and any(math.isinf(v) for v in values):
            errs.append(
                "px4_v117_state_invalid: vehicle_local_position contains INF"
            )
            break
        if values and any(not math.isnan(v) for v in values):
            finite_position_seen = True
    if local_position_list and not finite_position_seen:
        errs.append(
            "px4_v117_state_invalid: "
            "vehicle_local_position never produced a finite value"
        )

    for _, msg in attitude_list:
        q = getattr(msg, "q", None)
        if q is None:
            continue
        if any(math.isnan(v) or math.isinf(v) for v in q):
            errs.append(
                "px4_v117_state_invalid: "
                "vehicle_attitude.q contains NaN/INF"
            )
            break
        norm = math.sqrt(sum(v * v for v in q))
        if norm < 0.5 or norm > 1.5:
            errs.append(
                "px4_v117_state_invalid: "
                f"vehicle_attitude.q has implausible norm {norm}"
            )
            break

    metrics, semantic_errs = _px4_v117_ulg_metrics(config, state_dict)
    _update_feedback(feedback_list, metrics)
    errs.extend(semantic_errs)

    return errs


# =========================================================================
# Main Oracle Check
# =========================================================================

def check(config, msg_list, state_dict, feedback_list):
    errs = list()

    if getattr(config, "oracle_mode", "") == "px4_v117_jazzy":
        return _check_px4_v117_smoke(
            config, msg_list, state_dict, feedback_list
        )

    # --- Retrieve state data ---
    vehicle_acceleration_list = _safe_get(state_dict, "/VehicleAcceleration_PubSubTopic")
    vehicle_angular_velocity_list = _safe_get(state_dict, "/VehicleAngularVelocity_PubSubTopic")
    vehicle_angular_acceleration_list = _safe_get(state_dict, "/VehicleAngularAcceleration_PubSubTopic")
    vehicle_attitude_list = _safe_get(state_dict, "/VehicleAttitude_PubSubTopic")
    vehicle_local_position_list = _safe_get(state_dict, "/VehicleLocalPosition_PubSubTopic")
    vehicle_odometry_list = _safe_get(state_dict, "/VehicleOdometry_PubSubTopic")
    sensor_imu_list = _safe_get(state_dict, "/SensorsStatusImu_PubSubTopic")
    vehicle_gps_list = _safe_get(state_dict, "/VehicleGpsPosition_PubSubTopic")
    vehicle_global_position_list = _safe_get(state_dict, "/VehicleGlobalPosition_PubSubTopic")

    # --- Resolve dynamic thresholds ---
    thr = {}
    for param_name in PX4_DEFAULTS:
        thr[param_name] = _get_threshold(param_name, msg_list, config)

    # --- Ground contact filtering ---
    filter_ts_list = []
    local_x_list = []
    local_y_list = []
    local_z_list = []
    for (ts, pos) in vehicle_local_position_list:
        if hasattr(pos, 'dist_bottom') and pos.dist_bottom < GROUND_DIST:
            filter_ts_list.append(ts)
        local_x_list.append(pos.x)
        local_y_list.append(pos.y)
        local_z_list.append(pos.z)

    # =================================================================
    # Section 1: NaN / INF Checks
    # =================================================================
    for (ts, acc) in vehicle_acceleration_list:
        for i in range(3):
            if math.isnan(acc.xyz[i]):
                errs.append(f"{ts} VehicleAcceleration.xyz[{i}] is NaN")
            if math.isinf(acc.xyz[i]):
                errs.append(f"{ts} VehicleAcceleration.xyz[{i}] is INF")

    for (ts, ang) in vehicle_angular_velocity_list:
        for i in range(3):
            if math.isnan(ang.xyz[i]):
                errs.append(f"{ts} VehicleAngularVelocity.xyz[{i}] is NaN")
            if math.isinf(ang.xyz[i]):
                errs.append(f"{ts} VehicleAngularVelocity.xyz[{i}] is INF")

    for (ts, att) in vehicle_attitude_list:
        for i in range(4):
            if math.isnan(att.q[i]):
                errs.append(f"{ts} VehicleAttitude.q[{i}] is NaN")
            if math.isinf(att.q[i]):
                errs.append(f"{ts} VehicleAttitude.q[{i}] is INF")

    for (ts, pos) in vehicle_local_position_list:
        for field in ('x', 'y', 'z', 'vx', 'vy', 'vz'):
            val = getattr(pos, field, None)
            if val is not None:
                if math.isnan(val):
                    errs.append(f"{ts} VehicleLocalPosition.{field} is NaN")
                if math.isinf(val):
                    errs.append(f"{ts} VehicleLocalPosition.{field} is INF")

    # =================================================================
    # Section 2: Quaternion Norm Check
    # =================================================================
    for (ts, att) in vehicle_attitude_list:
        qw, qx, qy, qz = att.q[0], att.q[1], att.q[2], att.q[3]
        if not all(_is_valid(v) for v in (qw, qx, qy, qz)):
            continue
        quat_norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if abs(quat_norm - 1.0) > TOL_QUAT_NORM:
            errs.append(f"{ts} VehicleAttitude quaternion norm ({quat_norm:.6f}) deviates from 1.0")

    # =================================================================
    # Section 3: Acceleration Limits
    # Note: MPC_ACC_DOWN/UP_MAX are setpoint constraints in PX4's
    # trajectory planner. In POSCTL mode they don't directly limit
    # actual acceleration. We only report sustained violations (>100ms)
    # to filter normal PID transients.
    # =================================================================
    acc_violation_start = {}  # key: violation_type, value: first_ts
    for (ts, acc) in vehicle_acceleration_list:
        if _is_on_ground(ts, filter_ts_list):
            continue
        acc_x = acc.xyz[0]
        acc_y = acc.xyz[1]
        acc_z = acc.xyz[2] + 9.8  # offset gravity; +down, -up
        if not all(_is_valid(v) for v in (acc_x, acc_y, acc_z)):
            continue
        hor_acc = math.sqrt(acc_x * acc_x + acc_y * acc_y)

        if config.flight_mode in ("POSCTL", "OFFBOARD", "LOITER"):
            limit = thr["MPC_ACC_HOR_MAX"] + TOL_ACC
            if hor_acc > limit:
                errs.append(f"{ts} MPC_ACC_HOR_MAX violated: {hor_acc:.2f} > {limit:.1f}")

            up_limit = thr["MPC_ACC_UP_MAX"] + TOL_ACC
            if acc_z < -up_limit:
                key = "ACC_UP"
                if key not in acc_violation_start:
                    acc_violation_start[key] = ts
                elif (ts - acc_violation_start[key]) / 1e9 >= MIN_VIOLATION_DURATION:
                    errs.append(f"{ts} MPC_ACC_UP_MAX violated: {acc_z:.2f} < -{up_limit:.1f} (sustained)")
            else:
                acc_violation_start.pop("ACC_UP", None)

            dn_limit = thr["MPC_ACC_DOWN_MAX"] + TOL_ACC
            if acc_z > dn_limit:
                key = "ACC_DN"
                if key not in acc_violation_start:
                    acc_violation_start[key] = ts
                elif (ts - acc_violation_start[key]) / 1e9 >= MIN_VIOLATION_DURATION:
                    errs.append(f"{ts} MPC_ACC_DOWN_MAX violated: {acc_z:.2f} > {dn_limit:.1f} (sustained)")
            else:
                acc_violation_start.pop("ACC_DN", None)

    # =================================================================
    # Section 4: Velocity Limits
    # =================================================================
    max_xy_vel_val = 0.0
    for (ts, pos) in vehicle_local_position_list:
        if _is_on_ground(ts, filter_ts_list):
            continue
        vx, vy, vz = pos.vx, pos.vy, pos.vz
        if not all(_is_valid(v) for v in (vx, vy, vz)):
            continue
        hor_vel = math.sqrt(vx * vx + vy * vy)
        max_xy_vel_val = max(max_xy_vel_val, hor_vel)

        # XY velocity (all modes)
        xy_limit = thr["MPC_XY_VEL_MAX"] + TOL_VEL
        if hor_vel > xy_limit:
            errs.append(f"{ts} MPC_XY_VEL_MAX violated: {hor_vel:.2f} > {xy_limit:.1f}")

        # Manual mode horizontal velocity
        if config.flight_mode == "MANUAL":
            man_limit = thr["MPC_VEL_MANUAL"] + TOL_VEL
            if hor_vel > man_limit:
                errs.append(f"{ts} MPC_VEL_MANUAL violated: {hor_vel:.2f} > {man_limit:.1f}")

        # Vertical velocity
        if config.flight_mode in ("POSCTL", "ALTCTL", "OFFBOARD", "LOITER"):
            up_lim = thr["MPC_Z_VEL_MAX_UP"] + TOL_VEL_Z
            dn_lim = thr["MPC_Z_VEL_MAX_DN"] + TOL_VEL_Z
            # PX4 NED: vz > 0 = going down, vz < 0 = going up
            if vz < 0 and abs(vz) > up_lim:
                errs.append(f"{ts} MPC_Z_VEL_MAX_UP violated: {vz:.2f}")
            if vz > 0 and vz > dn_lim:
                errs.append(f"{ts} MPC_Z_VEL_MAX_DN violated: {vz:.2f}")

    # =================================================================
    # Section 5: Tilt Angle Check
    # =================================================================
    max_tilt_val = 0.0
    for (ts, att) in vehicle_attitude_list:
        if _is_on_ground(ts, filter_ts_list):
            continue
        qw, qx, qy, qz = att.q[0], att.q[1], att.q[2], att.q[3]
        if not all(_is_valid(v) for v in (qw, qx, qy, qz)):
            continue
        tilt_deg = _quat_to_tilt_deg(att.q)
        max_tilt_val = max(max_tilt_val, tilt_deg)

        if config.flight_mode in ("MANUAL", "ALTCTL"):
            tilt_limit = thr["MPC_MAN_TILT_MAX"] + TOL_TILT
        else:
            tilt_limit = thr["MPC_TILTMAX_AIR"] + TOL_TILT

        if tilt_deg > tilt_limit:
            errs.append(
                f"{ts} Tilt angle violated: {tilt_deg:.1f} deg > {tilt_limit:.1f} deg"
            )

    # =================================================================
    # Section 6: Angular Rate Limits
    # =================================================================
    max_angular_rate_val = 0.0
    max_rollpitch_rate_val = 0.0  # roll/pitch only (excludes commanded yaw)
    for (ts, ang) in vehicle_angular_velocity_list:
        if _is_on_ground(ts, filter_ts_list):
            continue
        if not all(_is_valid(ang.xyz[i]) for i in range(3)):
            continue
        # Convert rad/s to deg/s for comparison with PX4 parameters
        roll_rate_dps = abs(ang.xyz[0]) * 180.0 / math.pi
        pitch_rate_dps = abs(ang.xyz[1]) * 180.0 / math.pi
        yaw_rate_dps = abs(ang.xyz[2]) * 180.0 / math.pi
        max_rate = max(roll_rate_dps, pitch_rate_dps, yaw_rate_dps)
        max_angular_rate_val = max(max_angular_rate_val, max_rate)
        max_rollpitch_rate_val = max(max_rollpitch_rate_val,
                                     roll_rate_dps, pitch_rate_dps)

        # Roll rate
        roll_limit = thr["MC_ROLLRATE_MAX"] + TOL_RATE
        if roll_rate_dps > roll_limit:
            errs.append(f"{ts} MC_ROLLRATE_MAX violated: {roll_rate_dps:.1f} > {roll_limit:.1f} deg/s")

        # Pitch rate
        pitch_limit = thr["MC_PITCHRATE_MAX"] + TOL_RATE
        if pitch_rate_dps > pitch_limit:
            errs.append(f"{ts} MC_PITCHRATE_MAX violated: {pitch_rate_dps:.1f} > {pitch_limit:.1f} deg/s")

        # Yaw rate (mode-dependent)
        if config.flight_mode in ("MANUAL", "ALTCTL", "POSCTL"):
            yaw_limit = thr["MPC_MAN_Y_MAX"] + TOL_RATE
        elif config.flight_mode == "OFFBOARD":
            # OFFBOARD: pilot/companion commands yawspeed directly.
            # PX4 only clips at MC_YAWRATE_MAX (rate controller limit).
            # MPC_YAWRAUTO_MAX is irrelevant here — it's for AUTO mission
            # trajectory planning, not for commanded yawspeed.
            yaw_limit = thr["MC_YAWRATE_MAX"] + TOL_RATE
        else:
            # AUTO/LOITER: autonomous yaw rate limited by MPC_YAWRAUTO_MAX
            yaw_limit = thr["MPC_YAWRAUTO_MAX"] + TOL_RATE
        if yaw_rate_dps > yaw_limit:
            errs.append(f"{ts} Yaw rate violated: {yaw_rate_dps:.1f} > {yaw_limit:.1f} deg/s")

        # Overall yaw rate hardware limit (skip in OFFBOARD — already checked above)
        if config.flight_mode != "OFFBOARD":
            yaw_hw_limit = thr["MC_YAWRATE_MAX"] + TOL_RATE
            if yaw_rate_dps > yaw_hw_limit:
                errs.append(f"{ts} MC_YAWRATE_MAX violated: {yaw_rate_dps:.1f} > {yaw_hw_limit:.1f} deg/s")

    # =================================================================
    # Section 7: Jerk Limit (derivative of acceleration)
    # MPC_JERK_MAX is a trajectory planner constraint that ONLY applies
    # in AUTO modes (FlightTaskAutoLineSmoothVel). In POSCTL/MANUAL/ALTCTL
    # there is no jerk limiting — stick input maps directly to velocity
    # setpoint. We only check jerk in AUTO/OFFBOARD modes.
    # In other modes, we still compute max_jerk for feedback but use a
    # much higher threshold (100 m/s³) to only catch true anomalies.
    # =================================================================
    max_jerk_val = 0.0
    if len(vehicle_acceleration_list) >= 4:
        # Determine jerk threshold based on flight mode
        if config.flight_mode in ("AUTO", "OFFBOARD"):
            jerk_threshold = thr["MPC_JERK_MAX"] + TOL_JERK
        else:
            # POSCTL/MANUAL/ALTCTL: no firmware jerk limit, only flag crashes
            jerk_threshold = 100.0  # m/s³ - only true anomalies

        # 3-sample moving average to smooth noise before differentiation
        acc_smooth = []
        for i in range(1, len(vehicle_acceleration_list) - 1):
            ts_mid = vehicle_acceleration_list[i][0]
            ax = (vehicle_acceleration_list[i-1][1].xyz[0]
                  + vehicle_acceleration_list[i][1].xyz[0]
                  + vehicle_acceleration_list[i+1][1].xyz[0]) / 3.0
            ay = (vehicle_acceleration_list[i-1][1].xyz[1]
                  + vehicle_acceleration_list[i][1].xyz[1]
                  + vehicle_acceleration_list[i+1][1].xyz[1]) / 3.0
            if _is_valid(ax) and _is_valid(ay):
                acc_smooth.append((ts_mid, ax, ay))

        for i in range(1, len(acc_smooth)):
            ts_prev, ax_prev, ay_prev = acc_smooth[i - 1]
            ts_curr, ax_curr, ay_curr = acc_smooth[i]
            dt = (ts_curr - ts_prev) / 1e9  # ns to s
            if dt < 0.001:
                continue
            if _is_on_ground(ts_curr, filter_ts_list):
                continue
            jerk_x = (ax_curr - ax_prev) / dt
            jerk_y = (ay_curr - ay_prev) / dt
            jerk_hor = math.sqrt(jerk_x * jerk_x + jerk_y * jerk_y)
            max_jerk_val = max(max_jerk_val, jerk_hor)

            if jerk_hor > jerk_threshold:
                errs.append(
                    f"{ts_curr} MPC_JERK_MAX violated: "
                    f"{jerk_hor:.1f} > {jerk_threshold:.1f} m/s³"
                )

    # =================================================================
    # Section 8: Velocity-Position Consistency
    # Note: EKF2 fuses GPS at 5-10Hz causing discrete position jumps.
    # Use velocity-scaled tolerance: faster flight = larger acceptable error.
    # TOL_VEL_POS=1.0m base + 0.2*|v|*dt to account for GPS corrections.
    # =================================================================
    max_vel_pos_err = 0.0
    if len(vehicle_local_position_list) >= 2:
        for i in range(1, len(vehicle_local_position_list)):
            ts_prev = vehicle_local_position_list[i - 1][0]
            ts_curr = vehicle_local_position_list[i][0]
            pos_prev = vehicle_local_position_list[i - 1][1]
            pos_curr = vehicle_local_position_list[i][1]
            dt = (ts_curr - ts_prev) / 1e9
            if dt <= 0 or dt > 1.0:
                continue
            if not all(_is_valid(v) for v in (pos_prev.vx, pos_prev.vy, pos_prev.vz,
                                              pos_curr.x, pos_curr.y, pos_curr.z,
                                              pos_prev.x, pos_prev.y, pos_prev.z)):
                continue

            expected_dx = pos_prev.vx * dt
            expected_dy = pos_prev.vy * dt
            expected_dz = pos_prev.vz * dt
            actual_dx = pos_curr.x - pos_prev.x
            actual_dy = pos_curr.y - pos_prev.y
            actual_dz = pos_curr.z - pos_prev.z

            pos_err = math.sqrt(
                (expected_dx - actual_dx) ** 2
                + (expected_dy - actual_dy) ** 2
                + (expected_dz - actual_dz) ** 2
            )
            max_vel_pos_err = max(max_vel_pos_err, pos_err)

            # Velocity-scaled tolerance: base + proportional to speed
            vel_mag = math.sqrt(pos_prev.vx**2 + pos_prev.vy**2 + pos_prev.vz**2)
            adaptive_tol = TOL_VEL_POS + 0.2 * vel_mag * dt
            if pos_err > adaptive_tol:
                errs.append(
                    f"{ts_curr} Velocity-position inconsistency: "
                    f"{pos_err:.4f} m (tol={adaptive_tol:.3f})"
                )

    # =================================================================
    # Section 9: Angular Velocity - Attitude Consistency
    # =================================================================
    if len(vehicle_attitude_list) >= 2 and len(vehicle_angular_velocity_list) >= 1:
        for i in range(1, len(vehicle_attitude_list)):
            ts_prev = vehicle_attitude_list[i - 1][0]
            ts_curr = vehicle_attitude_list[i][0]
            att_prev = vehicle_attitude_list[i - 1][1]
            att_curr = vehicle_attitude_list[i][1]
            dt = (ts_curr - ts_prev) / 1e9
            if dt <= 0 or dt > 0.5:
                continue
            q_prev = [att_prev.q[j] for j in range(4)]
            q_curr = [att_curr.q[j] for j in range(4)]
            if not all(_is_valid(v) for v in q_prev + q_curr):
                continue

            # Rotation angle from quaternion difference
            q_diff_w = (q_curr[0]*q_prev[0] + q_curr[1]*q_prev[1]
                        + q_curr[2]*q_prev[2] + q_curr[3]*q_prev[3])
            q_diff_w = max(-1.0, min(1.0, q_diff_w))
            actual_angle = 2.0 * math.acos(abs(q_diff_w))

            # Find closest angular velocity sample
            best_ang = None
            best_dt = float('inf')
            ts_mid = (ts_prev + ts_curr) / 2
            for (ts_a, ang) in vehicle_angular_velocity_list:
                d = abs(ts_a - ts_mid)
                if d < best_dt:
                    best_dt = d
                    best_ang = ang
            if best_ang is None:
                continue
            if not all(_is_valid(best_ang.xyz[j]) for j in range(3)):
                continue

            omega = math.sqrt(sum(best_ang.xyz[j]**2 for j in range(3)))
            expected_angle = omega * dt
            consistency_err = abs(actual_angle - expected_angle)

            # Only flag large discrepancies (> 0.1 rad ≈ 5.7 deg)
            if consistency_err > 0.1:
                errs.append(
                    f"{ts_curr} Attitude-angular velocity inconsistency: "
                    f"{math.degrees(consistency_err):.1f} deg"
                )

    # =================================================================
    # Section 9b: Failure Detector - Roll/Pitch Limits (FD_FAIL_R/P)
    # =================================================================
    for (ts, att) in vehicle_attitude_list:
        if _is_on_ground(ts, filter_ts_list):
            continue
        q = [att.q[j] for j in range(4)]
        if not all(_is_valid(v) for v in q):
            continue
        roll_deg, pitch_deg = _quat_to_euler_deg(q)
        fd_roll_limit = thr["FD_FAIL_R"] + TOL_FD
        fd_pitch_limit = thr["FD_FAIL_P"] + TOL_FD
        if roll_deg > fd_roll_limit:
            errs.append(
                f"{ts} FD_FAIL_R violated: roll {roll_deg:.1f} > {fd_roll_limit:.1f} deg"
            )
        if pitch_deg > fd_pitch_limit:
            errs.append(
                f"{ts} FD_FAIL_P violated: pitch {pitch_deg:.1f} > {fd_pitch_limit:.1f} deg"
            )

    # =================================================================
    # Section 9c: Weathervane Yaw Rate Limit (WV_YRATE_MAX)
    # WV_YRATE_MAX only constrains the Weathervane module's yaw output.
    # In POSCTL/MANUAL/ALTCTL, yaw is controlled by MPC_MAN_Y_MAX
    # (already checked in Section 6). Only check WV_YRATE_MAX in
    # AUTO/LOITER modes where Weathervane is active and not overridden.
    # In OFFBOARD mode, explicit yawspeed commands override Weathervane,
    # so this check is not applicable.
    # =================================================================
    if config.flight_mode in ("AUTO", "LOITER"):
        for (ts, ang) in vehicle_angular_velocity_list:
            if _is_on_ground(ts, filter_ts_list):
                continue
            if not _is_valid(ang.xyz[2]):
                continue
            yaw_rate_dps = abs(ang.xyz[2]) * 180.0 / math.pi
            wv_limit = thr["WV_YRATE_MAX"] + TOL_WV_YRATE
            if yaw_rate_dps > wv_limit:
                errs.append(
                    f"{ts} WV_YRATE_MAX violated: {yaw_rate_dps:.1f} > {wv_limit:.1f} deg/s"
                )

    # =================================================================
    # Section 9d: Maximum Altitude Check (LNDMC_ALT_MAX)
    # =================================================================
    max_altitude_val = 0.0
    for (ts, pos) in vehicle_local_position_list:
        # PX4 NED: z is negative when above ground
        altitude = abs(pos.z) if _is_valid(pos.z) else 0.0
        max_altitude_val = max(max_altitude_val, altitude)
        alt_limit = thr["LNDMC_ALT_MAX"]
        if alt_limit > 0 and altitude > alt_limit + TOL_ALT:
            errs.append(
                f"{ts} LNDMC_ALT_MAX violated: {altitude:.1f} > {alt_limit + TOL_ALT:.1f} m"
            )

    # =================================================================
    # Section 10: IMU Sensor Inconsistency (feedback-driven)
    # =================================================================
    acc_inconsistency_list = []
    gyro_inconsistency_list = []
    for (ts, imu_state) in sensor_imu_list:
        acc_inconsistency_list.append(imu_state.accel_inconsistency_m_s_s[0])
        gyro_inconsistency_list.append(imu_state.gyro_inconsistency_rad_s[0])

    # =================================================================
    # Section 11: GPS Discrepancy (feedback-driven)
    # =================================================================
    lat_diff = []
    lon_diff = []
    skip = 0
    if vehicle_global_position_list:
        for (ts_gps_raw, gps_raw) in vehicle_gps_list:
            ts_diff = 9999999999999999999
            last_updated = 0
            updated = 0
            gps_estim = vehicle_global_position_list[0][1]

            for i in range(skip, len(vehicle_global_position_list)):
                ts_gps_estim = vehicle_global_position_list[i][0]
                gps_estim = vehicle_global_position_list[i][1]

                ts_diff_last = abs(ts_gps_raw - ts_gps_estim)
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

            lat_diff.append(gps_raw.lat - int(gps_estim.lat * 10000000))
            lon_diff.append(gps_raw.lon - int(gps_estim.lon * 10000000))

    # =================================================================
    # Section 12: Position Hold Check (PGFUZZ / LOITER mode)
    # =================================================================
    if config.exp_pgfuzz and local_x_list:
        mean_x = statistics.mean(local_x_list)
        mean_y = statistics.mean(local_y_list)
        mean_z = statistics.mean(local_z_list)

        diff_x = [abs(x - mean_x) for x in local_x_list]
        diff_y = [abs(y - mean_y) for y in local_y_list]
        diff_z = [abs(z - mean_z) for z in local_z_list]

        thr_hold = 0.3  # meters
        if max(diff_x) > thr_hold or max(diff_y) > thr_hold or max(diff_z) > thr_hold:
            diff_str = f"x {max(diff_x):.2f} y {max(diff_y):.2f} z {max(diff_z):.2f}"
            errs.append(f"Position changed in hold mode: {diff_str}")

    # --- Retrieve additional state data for new checks ---
    actuator_outputs_list = _safe_get(state_dict, "/ActuatorOutputs_PubSubTopic")
    vehicle_odometry_list2 = _safe_get(state_dict, "/VehicleOdometry_PubSubTopic")
    vehicle_control_mode_list = _safe_get(state_dict, "/VehicleControlMode_PubSubTopic")
    vehicle_status_list = _safe_get(state_dict, "/VehicleStatus_PubSubTopic")

    # =================================================================
    # Section 14: Actuator Saturation Detection
    # If ALL motors are at max (or min) for >200ms, the drone has lost
    # control authority. This is a real safety issue — not a parameter
    # threshold, but a physical inability to control the vehicle.
    #
    # Important distinction:
    # - In-flight saturation (MAX): genuine loss of control authority
    # - Post-crash saturation (MIN): motors off after crash, expected
    # We track both for feedback but only report in-flight saturation
    # as errors. Post-crash MIN saturation is filtered using a duration
    # cap: if MIN saturation exceeds CRASH_DURATION_CAP, it's a crashed
    # drone on the ground, not a control failure in flight.
    # =================================================================
    max_saturation_duration = 0.0
    max_inflight_saturation = 0.0  # only in-flight events
    saturation_start_ts = None
    MOTOR_MAX = 1900  # PWM max (PX4 default: 1000-2000, saturated at ~1900+)
    MOTOR_MIN = 1100  # PWM min (idle ~1100)
    SATURATION_DURATION_LIMIT = 0.2  # seconds
    CRASH_DURATION_CAP = 5.0  # seconds — MIN saturation beyond this = on ground

    for (ts, act) in actuator_outputs_list:
        if not hasattr(act, 'output'):
            continue
        outputs = act.output[:4]  # first 4 motors for quadrotor
        if len(outputs) == 0 or not all(_is_valid(float(o)) for o in outputs):
            continue

        # Check if all motors saturated high or all saturated low
        all_max = all(o >= MOTOR_MAX for o in outputs)
        all_min = all(o <= MOTOR_MIN for o in outputs)

        if all_max or all_min:
            if saturation_start_ts is None:
                saturation_start_ts = ts
                saturation_type = "MAX" if all_max else "MIN"
            else:
                duration = (ts - saturation_start_ts) / 1e9
                max_saturation_duration = max(max_saturation_duration, duration)

                if saturation_type == "MAX":
                    # In-flight: all motors at max = genuine control loss
                    max_inflight_saturation = max(max_inflight_saturation, duration)
                    if duration >= SATURATION_DURATION_LIMIT:
                        errs.append(
                            f"{ts} Actuator saturation ({saturation_type}): "
                            f"all motors saturated for {duration:.3f}s"
                        )
                elif saturation_type == "MIN" and duration <= CRASH_DURATION_CAP:
                    # Short MIN saturation: could be brief control loss in flight
                    max_inflight_saturation = max(max_inflight_saturation, duration)
                    if duration >= SATURATION_DURATION_LIMIT:
                        errs.append(
                            f"{ts} Actuator saturation ({saturation_type}): "
                            f"all motors saturated for {duration:.3f}s"
                        )
                # else: MIN saturation > CRASH_DURATION_CAP = on ground, skip error
        else:
            saturation_start_ts = None
            saturation_type = None

    # =================================================================
    # Section 15: Odometry Cross-Validation
    # VehicleOdometry and VehicleLocalPosition should report consistent
    # position/velocity. Large divergence indicates EKF output routing
    # bug (similar to Bug 2 but for different topic pairs).
    # =================================================================
    max_odom_pos_err = 0.0
    if vehicle_odometry_list2 and vehicle_local_position_list:
        odom_idx = 0
        for (ts_pos, pos) in vehicle_local_position_list:
            # Find closest odometry sample
            while (odom_idx < len(vehicle_odometry_list2) - 1
                   and vehicle_odometry_list2[odom_idx + 1][0] <= ts_pos):
                odom_idx += 1
            if odom_idx >= len(vehicle_odometry_list2):
                break
            ts_odom, odom = vehicle_odometry_list2[odom_idx]
            if abs(ts_odom - ts_pos) > 0.1e9:  # >100ms apart, skip
                continue
            if not hasattr(odom, 'x') or not hasattr(pos, 'x'):
                continue
            if not all(_is_valid(v) for v in (odom.x, odom.y, odom.z,
                                               pos.x, pos.y, pos.z)):
                continue
            odom_err = math.sqrt(
                (odom.x - pos.x)**2 + (odom.y - pos.y)**2 + (odom.z - pos.z)**2
            )
            max_odom_pos_err = max(max_odom_pos_err, odom_err)
            # Threshold: 2m divergence between two topics that should agree
            if odom_err > 2.0:
                errs.append(
                    f"{ts_pos} Odometry-Position divergence: {odom_err:.2f}m"
                )

    # =================================================================
    # Section 16: Control Loop Timing Violation
    # PX4's position controller runs at 50Hz (20ms). If consecutive
    # VehicleLocalPosition samples have gaps >100ms, the control loop
    # may have stalled — indicating a real firmware issue.
    # =================================================================
    max_control_gap = 0.0
    CONTROL_GAP_LIMIT = 0.15 if getattr(config, 'use_ulg', False) else 0.1
    if len(vehicle_local_position_list) >= 2:
        for i in range(1, len(vehicle_local_position_list)):
            ts_prev = vehicle_local_position_list[i - 1][0]
            ts_curr = vehicle_local_position_list[i][0]
            gap = (ts_curr - ts_prev) / 1e9
            if gap > 0:
                max_control_gap = max(max_control_gap, gap)
            if gap > CONTROL_GAP_LIMIT:
                errs.append(
                    f"{ts_curr} Control loop gap: {gap*1000:.1f}ms "
                    f"(expected <20ms)"
                )
    for feedback in feedback_list:
        if feedback.name == "imu_accel_inconsistency":
            if acc_inconsistency_list:
                feedback.update_value(max(acc_inconsistency_list))
        elif feedback.name == "imu_gyro_inconsistency":
            if gyro_inconsistency_list:
                feedback.update_value(max(gyro_inconsistency_list))
        elif feedback.name == "gps_lat_inconsistency":
            if lat_diff:
                feedback.update_value(max(lat_diff))
        elif feedback.name == "gps_lon_inconsistency":
            if lon_diff:
                feedback.update_value(max(lon_diff))
        elif feedback.name == "max_tilt_angle":
            feedback.update_value(max_tilt_val)
        elif feedback.name == "max_xy_velocity":
            feedback.update_value(max_xy_vel_val)
        elif feedback.name == "max_angular_rate":
            # In OFFBOARD mode, yaw rate is directly commanded and dominates
            # the feedback signal. Use roll/pitch only to guide exploration
            # toward genuine control failures rather than commanded yaw.
            if config.flight_mode == "OFFBOARD":
                feedback.update_value(max_rollpitch_rate_val)
            else:
                feedback.update_value(max_angular_rate_val)
        elif feedback.name == "max_jerk":
            feedback.update_value(max_jerk_val)
        elif feedback.name == "vel_pos_inconsistency":
            feedback.update_value(max_vel_pos_err)
        elif feedback.name == "max_altitude":
            feedback.update_value(max_altitude_val)
        elif feedback.name == "combined_tilt_velocity":
            # Combined metric: tilt * velocity rewards simultaneous extremes
            # tilt_deg in [0,180], xy_vel in [0,~12], product rewards both high
            combined = max_tilt_val * max_xy_vel_val
            feedback.update_value(combined)
        elif feedback.name == "actuator_saturation":
            # Use in-flight saturation only (excludes post-crash ground time)
            feedback.update_value(max_inflight_saturation)
        elif feedback.name == "odom_pos_divergence":
            feedback.update_value(max_odom_pos_err)
        elif feedback.name == "control_loop_gap":
            feedback.update_value(max_control_gap)

    return errs
