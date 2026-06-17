import math
import statistics

import kinpy


# =========================================================================
# Panda Joint Specifications (from joint_limits.yaml + URDF safety_controller)
# Version: moveit2-2.2.3, moveit_resources_panda_description-2.0.3
# =========================================================================

# (min_pos_rad, max_pos_rad, max_vel_rad_s, max_acc_rad_s2)
PANDA_JOINT_LIMITS = {
    "panda_joint1": (-2.8973, 2.8973, 2.175, 3.75),
    "panda_joint2": (-1.7628, 1.7628, 2.175, 1.875),
    "panda_joint3": (-2.8973, 2.8973, 2.175, 2.5),
    "panda_joint4": (-3.0718, -0.0698, 2.175, 3.125),
    "panda_joint5": (-2.8973, 2.8973, 2.61, 3.75),
    "panda_joint6": (-0.0175, 3.7525, 2.61, 5.0),
    "panda_joint7": (-2.8973, 2.8973, 2.61, 5.0),
    "panda_finger_joint1": (0.0, 0.04, 0.1, 0.3),
    "panda_finger_joint2": (0.0, 0.04, 0.1, 0.3),
}

# Tolerances
TOL_POS = math.radians(0.1)  # rad — position margin for known spec bug suppression
TOL_VEL = 0.3                # rad/s — velocity tolerance (position derivative noise)
TOL_ACC = 2.0                # rad/s^2 — acceleration tolerance multiplier for errors
MAX_REASONABLE_JERK = 500.0  # rad/s^3 — cap for jerk from discrete differentiation
# Only report acceleration as ERROR when exceeding 5x the declared limit
# (MoveIt TOPP-RA trajectory parameterization routinely exceeds 1-3x at knot points)
ACC_ERROR_MULT = 5.0
TOL_ENDPOINT = 0.001         # m (1mm, ISO 9283 repeatability)
MIN_DT = 0.005               # s — minimum dt for velocity/acceleration (filter timestamp jitter)
MAX_DT = 0.1                 # s — maximum dt for acceleration (data gap detection)

# A margin of MARGIN degrees is set to suppress an existing bug (joint limits
# not matching the specification) from shadowing other bugs. To reproduce this
# bug, please set MARGIN = 0.0 instead.
MARGIN = 0.1
MARGIN_RAD = math.radians(MARGIN)

PANDA_URDF = "/opt/ros/foxy/share/moveit_resources_panda_description/urdf/panda.urdf"
READY_POS = (0.306890566, 0.0, 0.590282052)

# =========================================================================
# New Oracle Constants (Section 7-13)
# =========================================================================

TOL_TRACKING = 0.1        # rad, trajectory tracking tolerance per joint per sample
TOL_ABORT_DRIFT = 0.01    # rad, max joint drift from home after abort
WORKSPACE_RADIUS = 0.855  # m, Panda approximate workspace sphere radius
MAX_PLANNING_TIME = 60.0  # s, total execution timeout (3-goal sequence normal: 15-40s)
MIN_EXEC_SAMPLES = 5      # minimum controller state messages for valid execution

# Ready/home position from initial_positions.yaml (NOT all zeros)
PANDA_HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

# Jerk limits (rad/s^3) from Franka Emika datasheet
PANDA_JERK_LIMITS = {
    "panda_joint1": 7500.0,
    "panda_joint2": 3750.0,
    "panda_joint3": 5000.0,
    "panda_joint4": 6250.0,
    "panda_joint5": 7500.0,
    "panda_joint6": 10000.0,
    "panda_joint7": 10000.0,
}


# =========================================================================
# Helper Functions
# =========================================================================

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

    joint_states_list = _safe_get(state_dict, "/joint_states")
    controller_states_list = _safe_get(state_dict, "/panda_arm_controller/state")
    move_action_status_list = _safe_get(state_dict, "/move_action/_action/status")
    motion_plan_request_list = _safe_get(state_dict, "/motion_plan_request")

    if not joint_states_list:
        print("[checker] no /joint_states data available")
    if not controller_states_list:
        print("[checker] no /panda_arm_controller/state data available")
    if not move_action_status_list:
        print("[checker] no /move_action/_action/status data available")
    if not motion_plan_request_list:
        print("[checker] no /motion_plan_request data available")

    # =================================================================
    # Section 1: Joint Position & Existential Checks (from /joint_states)
    # =================================================================

    final_joint_state = None
    for (ts, joint_state) in joint_states_list:
        joint_name_list = joint_state.name
        joint_position_list = joint_state.position

        for pi, pos in enumerate(joint_position_list):
            joint_name = joint_name_list[pi]
            limits = PANDA_JOINT_LIMITS.get(joint_name)
            if limits is None:
                continue

            min_pos, max_pos, _, _ = limits

            if math.isnan(pos):
                errs.append(f"{ts} {joint_name}'s position is NaN")
            elif math.isinf(pos):
                errs.append(f"{ts} {joint_name}'s position is INF")
            else:
                if pos < (min_pos - MARGIN_RAD) or pos > (max_pos + MARGIN_RAD):
                    pos_deg = math.degrees(pos)
                    min_deg = math.degrees(min_pos) - MARGIN
                    max_deg = math.degrees(max_pos) + MARGIN
                    errs.append(
                        f"{ts} {joint_name}'s position {pos_deg:.4f} is not "
                        f"within {min_deg:.1f} ~ {max_deg:.1f}"
                    )

        final_joint_state = joint_state

    # =================================================================
    # Section 2: Controller-Level Position, Velocity & Acceleration Checks
    # =================================================================

    error_pos_values = list()
    error_vel_values = list()
    prev_positions = {}  # joint_name -> (ts, pos) for velocity computation
    prev_velocities = {}  # joint_name -> (ts, vel) for acceleration computation
    prev_accelerations = {}  # joint_name -> (ts, acc)
    min_vel_margin = float("inf")
    max_jerk_value = 0.0
    jerk_violation_errs = []

    # Track per-joint position range for joint_motion_range feedback
    joint_pos_min = {}  # joint_name -> min_pos
    joint_pos_max = {}  # joint_name -> max_pos

    # Track per-joint velocity for velocity_roughness feedback
    joint_vel_series = {}  # joint_name -> list of velocities

    for (ts, cont_state) in controller_states_list:
        num_joints = len(cont_state.joint_names)
        actual_pos_list = cont_state.actual.positions

        # aggregate abs() of pos errors for feedback
        error_pos_list = cont_state.error.positions
        error_pos_values.extend([abs(pos) for pos in error_pos_list])

        # consistency check
        if len(actual_pos_list) != num_joints:
            errs.append(f"{ts} num_joints mismatch: {num_joints} vs {len(actual_pos_list)}")

        # --- Position checks on controller data ---
        for pi, pos in enumerate(actual_pos_list):
            joint_name = cont_state.joint_names[pi]
            limits = PANDA_JOINT_LIMITS.get(joint_name)
            if limits is None:
                continue

            min_pos, max_pos, _, _ = limits

            if math.isnan(pos):
                errs.append(f"{ts} {joint_name}'s position is NaN")
            elif math.isinf(pos):
                errs.append(f"{ts} {joint_name}'s position is INF")
            else:
                if pos < (min_pos - MARGIN_RAD) or pos > (max_pos + MARGIN_RAD):
                    pos_deg = math.degrees(pos)
                    min_deg = math.degrees(min_pos) - MARGIN
                    max_deg = math.degrees(max_pos) + MARGIN
                    errs.append(
                        f"{ts} {joint_name}'s position {pos_deg:.4f} is not "
                        f"within {min_deg:.1f} ~ {max_deg:.1f}"
                    )

            # Track position range per joint
            if joint_name not in joint_pos_min:
                joint_pos_min[joint_name] = pos
                joint_pos_max[joint_name] = pos
            else:
                if pos < joint_pos_min[joint_name]:
                    joint_pos_min[joint_name] = pos
                if pos > joint_pos_max[joint_name]:
                    joint_pos_max[joint_name] = pos

        # --- Velocity via position derivative (Gazebo doesn't publish vel) ---
        desired_vel_list = cont_state.desired.velocities
        for pi, pos in enumerate(actual_pos_list):
            joint_name = cont_state.joint_names[pi]
            limits = PANDA_JOINT_LIMITS.get(joint_name)
            if limits is None:
                continue

            _, _, max_vel, max_acc = limits

            if joint_name in prev_positions:
                prev_ts, prev_pos = prev_positions[joint_name]
                dt = (ts - prev_ts) / 1e9  # ns to s
                if MIN_DT < dt < MAX_DT:
                    vel = (pos - prev_pos) / dt

                    # Skip trajectory-segment jumps (position discontinuity
                    # when transitioning between goals). Panda max physical
                    # velocity is 2.61 rad/s; anything >5 is a jump, not motion.
                    if abs(vel) > 5.0:
                        prev_positions[joint_name] = (ts, pos)
                        prev_velocities.pop(joint_name, None)
                        prev_accelerations.pop(joint_name, None)
                        continue

                    # Velocity limit check
                    if abs(vel) > max_vel + TOL_VEL:
                        errs.append(
                            f"{ts} {joint_name}'s velocity {vel:.4f} rad/s "
                            f"exceeds limit {max_vel} rad/s"
                        )

                    # Velocity margin (how close to limit)
                    margin = max_vel - abs(vel)
                    if margin < min_vel_margin:
                        min_vel_margin = margin

                    # Velocity tracking error (vs desired trajectory)
                    if desired_vel_list and pi < len(desired_vel_list):
                        vel_err = abs(vel - desired_vel_list[pi])
                        error_vel_values.append(vel_err)

                    # Track velocity series for roughness
                    if joint_name not in joint_vel_series:
                        joint_vel_series[joint_name] = []
                    joint_vel_series[joint_name].append(vel)

                    # Acceleration check (dvel/dt)
                    if joint_name in prev_velocities:
                        pv_ts, pv_vel = prev_velocities[joint_name]
                        dt_v = (ts - pv_ts) / 1e9
                        if MIN_DT < dt_v < MAX_DT:
                            acc = (vel - pv_vel) / dt_v

                            # Skip trajectory-start transients: when velocity
                            # changes from near-zero to moving, it's a new
                            # trajectory segment, not a real over-acceleration
                            is_traj_start = (abs(pv_vel) < 0.05
                                             and abs(vel) > 0.1)
                            if is_traj_start:
                                prev_accelerations.pop(joint_name, None)
                            else:
                                if abs(acc) > max_acc * ACC_ERROR_MULT:
                                    errs.append(
                                        f"{ts} {joint_name}'s acceleration "
                                        f"{acc:.4f} rad/s^2 exceeds limit "
                                        f"{max_acc} rad/s^2"
                                    )

                                # Jerk check (dacc/dt)
                                if joint_name in prev_accelerations:
                                    pa_ts, pa = prev_accelerations[joint_name]
                                    dt_a = (ts - pa_ts) / 1e9
                                    if MIN_DT < dt_a < MAX_DT:
                                        jerk = (acc - pa) / dt_a
                                        abs_jerk = abs(jerk)
                                        if abs_jerk > MAX_REASONABLE_JERK:
                                            pass
                                        else:
                                            if abs_jerk > max_jerk_value:
                                                max_jerk_value = abs_jerk
                                            jerk_limit = PANDA_JERK_LIMITS.get(
                                                joint_name)
                                            if (jerk_limit
                                                    and abs_jerk > jerk_limit):
                                                jerk_violation_errs.append(
                                                    f"{ts} {joint_name}'s jerk "
                                                    f"{jerk:.2f} rad/s^3 "
                                                    f"exceeds limit "
                                                    f"{jerk_limit} rad/s^3"
                                                )

                                prev_accelerations[joint_name] = (ts, acc)

                    prev_velocities[joint_name] = (ts, vel)

            prev_positions[joint_name] = (ts, pos)

    # =================================================================
    # Section 3: Action Status Validation
    # =================================================================

    # ROS2 action status: each GoalStatusArray message contains status_list
    # with the current status of ALL submitted goals. We need to:
    # 1. Determine how many goals were submitted (from motion_plan_request_list)
    # 2. Track each goal's terminal status from the final status message
    #
    # Status codes: 1=ACCEPTED, 2=EXECUTING, 4=SUCCEEDED, 5=CANCELING, 6=ABORTED
    #
    # For multi-goal sequences, the LAST status message contains the final
    # status of all goals. We extract terminal statuses from there.

    num_goals_detected = 0
    last_action_terminal = None
    goal_terminal_statuses = []  # terminal status per goal

    if move_action_status_list:
        # Take the last status message — it has all goals' final statuses
        (_, last_action_msg) = move_action_status_list[-1]
        if last_action_msg.status_list:
            goal_terminal_statuses = [
                gs.status for gs in last_action_msg.status_list
            ]
            num_goals_detected = len(goal_terminal_statuses)
            last_action_terminal = goal_terminal_statuses[-1]

            # Validate: each goal should end in 4 (SUCCESS) or 6 (ABORTED)
            for gi, gstatus in enumerate(goal_terminal_statuses):
                if gstatus not in (4, 6):
                    errs.append(
                        f"goal {gi}: unexpected terminal status {gstatus} "
                        f"(expected 4=SUCCESS or 6=ABORTED)"
                    )

            print(f"[oracle] {num_goals_detected} goals detected, "
                  f"terminal statuses: {goal_terminal_statuses}")
    else:
        errs.append("no action status received")

    # =================================================================
    # Section 4: Motion Plan Request Count
    # =================================================================

    num_motion_plan_request = len(motion_plan_request_list)
    if num_motion_plan_request < 1:
        errs.append(f"# Motion plan request < 1: ({num_motion_plan_request})")
    else:
        # Multi-goal: use the LAST motion plan request for endpoint verification
        # (the final goal determines where the robot should end up)
        (ts, mpr) = motion_plan_request_list[-1]
        goal_constraints = mpr.goal_constraints[0]
        goal_position = goal_constraints.position_constraints[0] \
            .constraint_region.primitive_poses[0].position
        goal_orientation = goal_constraints.orientation_constraints[0].orientation

        goal_x = goal_position.x
        goal_y = goal_position.y
        goal_z = goal_position.z
        goal_w = goal_orientation.w

        # =============================================================
        # Section 5: FK/IK Endpoint Verification
        # =============================================================

        if final_joint_state is not None:
            with open(PANDA_URDF) as f:
                urdf_content = f.read()
            chain = kinpy.build_chain_from_urdf(urdf_content)
            serial_chain = kinpy.build_serial_chain_from_urdf(
                urdf_content, "panda_hand"
            )

            joint_angle_map = dict()
            for i, name in enumerate(final_joint_state.name):
                joint_angle_map[name] = final_joint_state.position[i]

            fwd_kinematics_sol = chain.forward_kinematics(joint_angle_map)
            final_end_effector_pos = fwd_kinematics_sol["panda_hand"].pos

            final_pos_x = final_end_effector_pos[0]
            final_pos_y = final_end_effector_pos[1]
            final_pos_z = final_end_effector_pos[2]

            if last_action_terminal == 4:
                # Last goal succeeded — endpoint should be at the last goal
                dist_goal_to_final_pos = math.sqrt(
                    pow(goal_x - final_pos_x, 2)
                    + pow(goal_y - final_pos_y, 2)
                    + pow(goal_z - final_pos_z, 2)
                )

                # === Section 6: Feedback Updates (success path) ===
                for feedback in feedback_list:
                    if feedback.name == "end_point_deviation":
                        feedback.update_value(dist_goal_to_final_pos)
                    elif feedback.name == "mean_joint_pos_error":
                        if error_pos_values:
                            feedback.update_value(statistics.mean(error_pos_values))
                    elif feedback.name == "max_joint_pos_error":
                        if error_pos_values:
                            feedback.update_value(max(error_pos_values))
                    elif feedback.name == "mean_joint_vel_error":
                        if error_vel_values:
                            feedback.update_value(statistics.mean(error_vel_values))
                    elif feedback.name == "max_joint_vel_error":
                        if error_vel_values:
                            feedback.update_value(max(error_vel_values))
                    elif feedback.name == "max_velocity_margin":
                        if min_vel_margin < float("inf"):
                            feedback.update_value(min_vel_margin)
                    elif feedback.name == "trajectory_tracking_rms":
                        if error_pos_values:
                            rms = math.sqrt(
                                sum(v * v for v in error_pos_values)
                                / len(error_pos_values)
                            )
                            feedback.update_value(rms)
                    elif feedback.name == "max_joint_jerk":
                        if max_jerk_value > 0.0:
                            feedback.update_value(max_jerk_value)
                    elif feedback.name == "velocity_roughness":
                        max_cv = 0.0
                        for jname, vels in joint_vel_series.items():
                            if len(vels) < 2:
                                continue
                            mean_v = statistics.mean(vels)
                            if abs(mean_v) < 0.001:
                                continue
                            std_v = statistics.stdev(vels)
                            cv = std_v / abs(mean_v)
                            if cv > max_cv:
                                max_cv = cv
                        if max_cv > 0.0:
                            feedback.update_value(max_cv)
                    elif feedback.name == "joint_motion_range":
                        max_range = 0.0
                        for jname in joint_pos_min:
                            r = joint_pos_max[jname] - joint_pos_min[jname]
                            if r > max_range:
                                max_range = r
                        if max_range > 0.0:
                            feedback.update_value(max_range)

                print(f"D: {dist_goal_to_final_pos:.6f}")
                if dist_goal_to_final_pos > TOL_ENDPOINT:
                    errs.append(
                        f"goal and actual pos deviation too high: "
                        f"{dist_goal_to_final_pos}"
                    )

                # === Section 7: Trajectory Tracking Quality ===
                for (cs_ts, cs) in controller_states_list:
                    for ei, epos in enumerate(cs.error.positions):
                        if abs(epos) > TOL_TRACKING:
                            jn = cs.joint_names[ei] if ei < len(cs.joint_names) else f"joint{ei}"
                            errs.append(
                                f"{cs_ts} {jn}'s tracking error "
                                f"{epos:.4f} rad exceeds tolerance "
                                f"{TOL_TRACKING} rad"
                            )

                # === Section 11: Joint Jerk Violations ===
                errs.extend(jerk_violation_errs)

            elif last_action_terminal == 6:
                # Last goal aborted
                # Only check ready position if ALL goals aborted (single goal or
                # first goal failed). For multi-goal where some succeeded first,
                # the robot will be at the last successful goal, not ready pos.
                all_aborted = all(
                    s == 6 for s in goal_terminal_statuses
                )

                if all_aborted:
                    ready_pos_x, ready_pos_y, ready_pos_z = READY_POS

                    dist_ready_to_final_pos = math.sqrt(
                        pow(ready_pos_x - final_pos_x, 2)
                        + pow(ready_pos_y - final_pos_y, 2)
                        + pow(ready_pos_z - final_pos_z, 2)
                    )

                    print(f"D: {dist_ready_to_final_pos:.6f}")
                    if dist_ready_to_final_pos > TOL_ENDPOINT:
                        errs.append(
                            f"robot shouldn't have moved: {dist_ready_to_final_pos}"
                        )

                    # === Section 8: Initial Position Consistency (abort only) ===
                    if final_joint_state is not None:
                        arm_joint_names = [f"panda_joint{i}" for i in range(1, 8)]
                        max_drift = 0.0
                        for i, jname in enumerate(final_joint_state.name):
                            if jname in arm_joint_names:
                                idx = arm_joint_names.index(jname)
                                drift = abs(final_joint_state.position[i] - PANDA_HOME[idx])
                                if drift > max_drift:
                                    max_drift = drift
                                if drift > TOL_ABORT_DRIFT:
                                    errs.append(
                                        f"{jname} drifted {drift:.4f} rad from home "
                                        f"after abort (tolerance: {TOL_ABORT_DRIFT} rad)"
                                    )
                        # Update abort_joint_drift feedback
                        for feedback in feedback_list:
                            if feedback.name == "abort_joint_drift":
                                if max_drift > 0.0:
                                    feedback.update_value(max_drift)
                                break

                # Update path-independent feedbacks for abort case too
                for feedback in feedback_list:
                    if feedback.name == "max_joint_jerk":
                        if max_jerk_value > 0.0:
                            feedback.update_value(max_jerk_value)
                    elif feedback.name == "velocity_roughness":
                        max_cv = 0.0
                        for jname, vels in joint_vel_series.items():
                            if len(vels) < 2:
                                continue
                            mean_v = statistics.mean(vels)
                            if abs(mean_v) < 0.001:
                                continue
                            std_v = statistics.stdev(vels)
                            cv = std_v / abs(mean_v)
                            if cv > max_cv:
                                max_cv = cv
                        if max_cv > 0.0:
                            feedback.update_value(max_cv)
                    elif feedback.name == "joint_motion_range":
                        max_range = 0.0
                        for jname in joint_pos_min:
                            r = joint_pos_max[jname] - joint_pos_min[jname]
                            if r > max_range:
                                max_range = r
                        if max_range > 0.0:
                            feedback.update_value(max_range)

                # IK reachability check on the last goal
                tf_goal = kinpy.Transform()
                tf_goal.rot[0] = 0.0
                tf_goal.rot[1] = 0.0
                tf_goal.rot[2] = 0.0
                tf_goal.rot[3] = goal_w
                tf_goal.pos[0] = goal_x
                tf_goal.pos[1] = goal_y
                tf_goal.pos[2] = goal_z

                try:
                    ik = serial_chain.inverse_kinematics(tf_goal)
                except Exception:
                    return errs

                joints = serial_chain.get_joint_parameter_names()
                valid_cnt = 0
                checked_cnt = 0

                for i, joint_angle in enumerate(ik):
                    joint_name = joints[i]
                    limits = PANDA_JOINT_LIMITS.get(joint_name)
                    if limits is None:
                        continue
                    checked_cnt += 1
                    min_pos, max_pos, _, _ = limits
                    if min_pos <= joint_angle <= max_pos:
                        valid_cnt += 1

                if checked_cnt > 0 and valid_cnt == checked_cnt:
                    errs.append(
                        f"controller failed to find inverse kinematics "
                        f"solution: {ik}"
                    )

    # =================================================================
    # Section 9: Workspace Boundary Distance (unconditional)
    # =================================================================

    if motion_plan_request_list:
        # Multi-goal: compute max boundary proximity across all goals
        max_boundary_proximity = 0.0
        for (_, mpr_msg) in motion_plan_request_list:
            try:
                gp = mpr_msg.goal_constraints[0].position_constraints[0] \
                    .constraint_region.primitive_poses[0].position
                dist_from_origin = math.sqrt(gp.x ** 2 + gp.y ** 2 + gp.z ** 2)
                proximity = 1.0 / max(0.01, abs(dist_from_origin - WORKSPACE_RADIUS))
                if proximity > max_boundary_proximity:
                    max_boundary_proximity = proximity
            except (IndexError, AttributeError):
                continue
        if max_boundary_proximity > 0.0:
            for feedback in feedback_list:
                if feedback.name == "workspace_boundary_distance":
                    feedback.update_value(max_boundary_proximity)
                    break

    # =================================================================
    # Section 10: Execution Duration (first plan request to last status)
    # =================================================================

    if motion_plan_request_list and move_action_status_list:
        # Total execution time: from first planning request to final status
        plan_first_ts = motion_plan_request_list[0][0]
        status_last_ts = move_action_status_list[-1][0]
        exec_dur = (status_last_ts - plan_first_ts) / 1e9  # ns to s
        if exec_dur > 0:
            for feedback in feedback_list:
                if feedback.name == "planning_duration":
                    feedback.update_value(exec_dur)
                    break
            if exec_dur > MAX_PLANNING_TIME:
                errs.append(
                    f"execution duration {exec_dur:.2f}s exceeds "
                    f"threshold {MAX_PLANNING_TIME}s"
                )

    # =================================================================
    # Section 12: Goal Success Ratio (unconditional)
    # =================================================================

    if goal_terminal_statuses:
        n_success = sum(1 for s in goal_terminal_statuses if s == 4)
        n_total = len(goal_terminal_statuses)
        # ratio of failed goals: 1.0 = all failed, 0.0 = all succeeded
        # DEC type: lower value = more interesting (closer to all-succeed)
        fail_ratio = 1.0 - (n_success / n_total)
        for feedback in feedback_list:
            if feedback.name == "goal_success_ratio":
                feedback.update_value(fail_ratio)
                break

    num_controller_samples = len(controller_states_list)

    if last_action_terminal == 4:
        if num_controller_samples < MIN_EXEC_SAMPLES:
            errs.append(
                f"execution too short: only {num_controller_samples} controller "
                f"samples (minimum: {MIN_EXEC_SAMPLES})"
            )

    return errs
