import math
import statistics

import numpy as np


# =========================================================================
# PX4 v1.12 Default Parameter Thresholds (used when not dynamically mutated)
# =========================================================================

PX4_DEFAULTS = {
    "MPC_XY_VEL_MAX": 12.0,       # m/s
    "MPC_Z_VEL_MAX_UP": 3.0,      # m/s
    "MPC_Z_VEL_MAX_DN": 1.0,      # m/s
    "MPC_VEL_MANUAL": 10.0,       # m/s
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
}

# Simulation tolerances (to avoid false positives from numerical noise)
TOL_VEL = 0.5          # m/s
TOL_ACC = 1.0          # m/s²
TOL_TILT = 2.0         # deg
TOL_RATE = 5.0         # deg/s
TOL_JERK = 2.0         # m/s³
TOL_QUAT_NORM = 0.01
TOL_VEL_POS = 0.1      # m
GROUND_DIST = 0.15     # m - threshold for ground contact filtering
GROUND_TS_WINDOW = 0.25e9  # nanoseconds


# =========================================================================
# Helper Functions
# =========================================================================

def _get_threshold(param_name, msg_list, config):
    """Return the effective threshold for a parameter.

    In PGFUZZ mode, if the currently mutated parameter matches param_name,
    use the mutated value as the threshold. Otherwise use the default.
    """
    default = PX4_DEFAULTS.get(param_name, 0.0)
    if config.exp_pgfuzz and msg_list:
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


def _is_valid(value):
    """Check that a float value is neither NaN nor Inf."""
    return not (math.isnan(value) or math.isinf(value))


def _safe_get(state_dict, topic):
    """Safely retrieve a topic's data from state_dict."""
    try:
        return state_dict[topic]
    except KeyError:
        return []


# =========================================================================
# Main Oracle Check
# =========================================================================

def check(config, msg_list, state_dict, feedback_list):
    errs = list()

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
    # Section 3: Acceleration Limits (refactored with dynamic thresholds)
    # =================================================================
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
                errs.append(f"{ts} MPC_ACC_UP_MAX violated: {acc_z:.2f} < -{up_limit:.1f}")
            dn_limit = thr["MPC_ACC_DOWN_MAX"] + TOL_ACC
            if acc_z > dn_limit:
                errs.append(f"{ts} MPC_ACC_DOWN_MAX violated: {acc_z:.2f} > {dn_limit:.1f}")

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
            up_lim = thr["MPC_Z_VEL_MAX_UP"] + TOL_VEL
            dn_lim = thr["MPC_Z_VEL_MAX_DN"] + TOL_VEL
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
        else:
            yaw_limit = thr["MPC_YAWRAUTO_MAX"] + TOL_RATE
        if yaw_rate_dps > yaw_limit:
            errs.append(f"{ts} Yaw rate violated: {yaw_rate_dps:.1f} > {yaw_limit:.1f} deg/s")

        # Overall yaw rate hardware limit
        yaw_hw_limit = thr["MC_YAWRATE_MAX"] + TOL_RATE
        if yaw_rate_dps > yaw_hw_limit:
            errs.append(f"{ts} MC_YAWRATE_MAX violated: {yaw_rate_dps:.1f} > {yaw_hw_limit:.1f} deg/s")

    # =================================================================
    # Section 7: Jerk Limit (derivative of acceleration)
    # MPC_JERK_MAX is a trajectory planner constraint, only meaningful
    # in modes where the trajectory generator is active (OFFBOARD, AUTO).
    # In MANUAL/POSCTL/ALTCTL the pilot has direct stick control and
    # jerk is not rate-limited by the firmware.
    # =================================================================
    max_jerk_val = 0.0
    jerk_applicable = config.flight_mode in ("OFFBOARD", "AUTO_MISSION",
                                             "AUTO_LOITER", "AUTO_RTL")
    if len(vehicle_acceleration_list) >= 4:
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

            if jerk_applicable:
                jerk_limit = thr["MPC_JERK_MAX"] + TOL_JERK
                if jerk_hor > jerk_limit:
                    errs.append(
                        f"{ts_curr} MPC_JERK_MAX violated: "
                        f"{jerk_hor:.1f} > {jerk_limit:.1f} m/s³"
                    )

    # =================================================================
    # Section 8: Velocity-Position Consistency
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

            if pos_err > TOL_VEL_POS:
                errs.append(
                    f"{ts_curr} Velocity-position inconsistency: {pos_err:.4f} m"
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
    for (ts_gps_raw, gps_raw) in vehicle_gps_list:
        ts_diff = 9999999999999999999
        last_updated = 0
        updated = 0

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

    # =================================================================
    # Section 13: Update Feedback Metrics
    # =================================================================
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
            feedback.update_value(max_angular_rate_val)
        elif feedback.name == "max_jerk":
            feedback.update_value(max_jerk_val)
        elif feedback.name == "vel_pos_inconsistency":
            feedback.update_value(max_vel_pos_err)

    return errs
