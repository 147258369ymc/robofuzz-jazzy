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
TOL_VEL = 0.1                # rad/s — velocity tolerance for numerical noise
TOL_ACC = 0.5                # rad/s^2 — acceleration tolerance
TOL_ENDPOINT = 0.001         # m (1mm, ISO 9283 repeatability)
MIN_DT = 0.001               # s — minimum dt for acceleration (avoid div-by-zero)
MAX_DT = 0.1                 # s — maximum dt for acceleration (data gap detection)

# A margin of MARGIN degrees is set to suppress an existing bug (joint limits
# not matching the specification) from shadowing other bugs. To reproduce this
# bug, please set MARGIN = 0.0 instead.
MARGIN = 0.1
MARGIN_RAD = math.radians(MARGIN)

PANDA_URDF = "/opt/ros/foxy/share/moveit_resources_panda_description/urdf/panda.urdf"
READY_POS = (0.306890566, 0.0, 0.590282052)


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
    prev_velocities = {}  # joint_name -> (ts, vel)
    min_vel_margin = float("inf")

    for (ts, cont_state) in controller_states_list:
        num_joints = len(cont_state.joint_names)
        actual_pos_list = cont_state.actual.positions
        actual_vel_list = cont_state.actual.velocities

        # aggregate abs() of pos and vel errors for feedback
        error_pos_list = cont_state.error.positions
        error_vel_list = cont_state.error.velocities
        error_pos_values.extend([abs(pos) for pos in error_pos_list])
        error_vel_values.extend([abs(vel) for vel in error_vel_list])

        # consistency check
        if len(actual_pos_list) != num_joints:
            errs.append(f"{ts} num_joints mismatch: {num_joints} vs {len(actual_pos_list)}")
        if len(actual_vel_list) != num_joints:
            errs.append(f"{ts} num_joints mismatch: {num_joints} vs {len(actual_vel_list)}")

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

        # --- Velocity checks (from OracleIR specs: joint_limits.yaml values) ---
        for vi, vel in enumerate(actual_vel_list):
            joint_name = cont_state.joint_names[vi]
            limits = PANDA_JOINT_LIMITS.get(joint_name)
            if limits is None:
                continue

            _, _, max_vel, max_acc = limits

            if math.isnan(vel):
                errs.append(f"{ts} {joint_name}'s velocity is NaN")
            elif math.isinf(vel):
                errs.append(f"{ts} {joint_name}'s velocity is INF")
            else:
                # Velocity limit check
                if abs(vel) > max_vel + TOL_VEL:
                    errs.append(
                        f"{ts} {joint_name}'s velocity {vel:.4f} rad/s exceeds "
                        f"limit {max_vel} rad/s"
                    )

                # Track closest-to-limit velocity margin for feedback
                margin = max_vel - abs(vel)
                if margin < min_vel_margin:
                    min_vel_margin = margin

                # --- Acceleration check (time derivative of velocity) ---
                if joint_name in prev_velocities:
                    prev_ts, prev_vel = prev_velocities[joint_name]
                    dt = (ts - prev_ts) / 1e9  # ns to s
                    if MIN_DT < dt < MAX_DT:
                        acc = (vel - prev_vel) / dt
                        if abs(acc) > max_acc + TOL_ACC:
                            errs.append(
                                f"{ts} {joint_name}'s acceleration "
                                f"{acc:.4f} rad/s^2 exceeds limit {max_acc} rad/s^2"
                            )

                prev_velocities[joint_name] = (ts, vel)

    # =================================================================
    # Section 3: Action Status Validation
    # =================================================================

    action_status = list()
    for (ts, action) in move_action_status_list:
        if action.status_list:
            action_status.append(action.status_list[0].status)

    print(action_status)
    if len(action_status) == 2:
        if action_status[0] != 2:
            errs.append(f"action doesn't start with 2: {action_status[0]}")

        if action_status[-1] != 4 and action_status[-1] != 6:
            errs.append(f"action doesn't end with 4 or 6: {action_status[-1]}")
    else:
        errs.append(f"invalid goal action status: {str(action_status)}")

    # =================================================================
    # Section 4: Motion Plan Request Count
    # =================================================================

    num_motion_plan_request = len(motion_plan_request_list)
    if num_motion_plan_request != 1:
        errs.append(f"# Motion plan request != 1: ({num_motion_plan_request})")
    else:
        # retrieve the requested goal
        (ts, mpr) = motion_plan_request_list[0]
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

            if len(action_status) == 2 and action_status[-1] == 4:
                # Action succeeded — endpoint should be at the goal
                dist_goal_to_final_pos = math.sqrt(
                    pow(goal_x - final_pos_x, 2)
                    + pow(goal_y - final_pos_y, 2)
                    + pow(goal_z - final_pos_z, 2)
                )

                # === Section 6: Feedback Updates ===
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

                print(f"D: {dist_goal_to_final_pos:.6f}")
                if dist_goal_to_final_pos > TOL_ENDPOINT:
                    errs.append(
                        f"goal and actual pos deviation too high: "
                        f"{dist_goal_to_final_pos}"
                    )

            elif len(action_status) == 2 and action_status[-1] == 6:
                # Action aborted — endpoint should remain at ready position
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

                # IK reachability check
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

    return errs
