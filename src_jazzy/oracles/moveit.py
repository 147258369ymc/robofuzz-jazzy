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

# --- Tolerances ---
TOL_POS = math.radians(0.1)    # rad — position limit tolerance
TOL_ENDPOINT = 0.001           # m (1mm, ISO 9283 repeatability)
TOL_TRACKING = 0.1             # rad — tracking error tolerance per joint
TOL_ABORT_DRIFT = 0.05         # rad — max joint movement after abort (500ms window)

# Velocity/acceleration check tolerance. TOPP-RA's discrete time-parameterization
# slightly overshoots the configured limit at sample points; sub-2% overshoots are
# numerical/discretization noise, not bugs. Only flag overshoots beyond this ratio
# so genuine large violations (e.g. 2.3+ rad/s vs 2.175 limit) stand out.
VEL_TOL_RATIO = 1.02           # allow up to +2% over velocity limit
ACC_TOL_RATIO = 1.02           # allow up to +2% over acceleration limit

# Known spec bug suppression margin
MARGIN = 0.01
MARGIN_RAD = math.radians(MARGIN)

# Environment health thresholds
MAX_SAMPLE_GAP = 0.15          # s — max gap between controller samples
MIN_EXEC_SAMPLES = 5           # minimum samples for valid analysis
WORKSPACE_RADIUS = 0.855       # m — Panda approximate workspace sphere

# Tracking error: skip first N samples at trajectory start (initial transient)
TRACKING_SKIP_SAMPLES = 5

PANDA_URDF = "/opt/ros/foxy/share/moveit_resources_panda_description/urdf/panda.urdf"
READY_POS = (0.306890566, 0.0, 0.590282052)

# Home position (from initial_positions.yaml)
PANDA_HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]


# =========================================================================
# Helper Functions
# =========================================================================

def _safe_get(state_dict, topic):
    """Safely retrieve a topic's data from state_dict."""
    try:
        return state_dict[topic]
    except KeyError:
        return []


def _all_goals_unreachable(msg_list):
    """Check if ALL goals in msg_list are likely outside Panda reachable workspace.

    Uses a conservative inner radius (0.75m) — goals within this are definitely
    reachable; goals beyond it may or may not be reachable depending on joint
    limits, self-collision, and arm configuration.
    """
    if not msg_list:
        return False
    # Use a tighter radius: goals within 0.75m are reliably reachable.
    # The full workspace sphere is ~0.855m but boundary goals often fail
    # due to joint limits / self-collision, causing no-status false positives.
    RELIABLE_REACH = 0.75
    for msg in msg_list:
        try:
            # Handle both ROS Pose objects and OrderedDict (from pickle)
            if hasattr(msg, 'position'):
                px, py, pz = msg.position.x, msg.position.y, msg.position.z
            else:
                pos = msg['position']
                px, py, pz = pos['x'], pos['y'], pos['z']
            dist = math.sqrt(px**2 + py**2 + pz**2)
            if dist <= RELIABLE_REACH:
                return False
        except (AttributeError, KeyError, TypeError):
            return False  # can't parse → don't suppress
    return True


# =========================================================================
# Main Oracle Check — 4-Layer Architecture
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

    # =================================================================
    # Layer 4: Environment Health (annotations, checked first for context)
    # =================================================================

    has_gap = False
    if len(controller_states_list) >= 2:
        for i in range(1, len(controller_states_list)):
            ts_prev = controller_states_list[i - 1][0]
            ts_curr = controller_states_list[i][0]
            gap = (ts_curr - ts_prev) / 1e9
            if gap > MAX_SAMPLE_GAP:
                has_gap = True
                break

    data_sufficient = len(controller_states_list) >= MIN_EXEC_SAMPLES

    # =================================================================
    # Layer 1: Safety Invariants (direct comparison, zero computation)
    # =================================================================

    # --- 1.1 Joint Position Limits (from /joint_states) ---
    final_joint_state = None
    for (ts, joint_state) in joint_states_list:
        for pi, pos in enumerate(joint_state.position):
            joint_name = joint_state.name[pi]
            limits = PANDA_JOINT_LIMITS.get(joint_name)
            if limits is None:
                continue
            min_pos, max_pos, _, _ = limits

            if math.isnan(pos) or math.isinf(pos):
                errs.append(f"{ts} {joint_name}'s position is NaN/INF")
            elif pos < (min_pos - MARGIN_RAD) or pos > (max_pos + MARGIN_RAD):
                errs.append(
                    f"{ts} {joint_name}'s position {math.degrees(pos):.4f} deg "
                    f"outside [{math.degrees(min_pos):.1f}, "
                    f"{math.degrees(max_pos):.1f}]"
                )
        final_joint_state = joint_state

    # --- 1.2 & 1.3 Planning Layer Velocity/Acceleration Limits ---
    # Check desired.velocities and desired.accelerations directly against limits.
    # These are TOPP-RA outputs — if they exceed limits, it's a planner bug.
    desired_vel_max_ratio = 0.0
    desired_acc_max_ratio = 0.0

    for (ts, cont_state) in controller_states_list:
        # 1.2 Velocity
        if cont_state.desired.velocities:
            for i, name in enumerate(cont_state.joint_names):
                limits = PANDA_JOINT_LIMITS.get(name)
                if limits is None or i >= len(cont_state.desired.velocities):
                    continue
                _, _, max_vel, _ = limits
                v = abs(cont_state.desired.velocities[i])
                ratio = v / max_vel if max_vel > 0 else 0
                if ratio > desired_vel_max_ratio:
                    desired_vel_max_ratio = ratio
                if v > max_vel * VEL_TOL_RATIO:
                    errs.append(
                        f"{ts} TOPP-RA planned velocity for {name}: "
                        f"{v:.4f} > limit {max_vel} rad/s"
                    )

        # 1.3 Acceleration
        if cont_state.desired.accelerations:
            for i, name in enumerate(cont_state.joint_names):
                limits = PANDA_JOINT_LIMITS.get(name)
                if limits is None or i >= len(cont_state.desired.accelerations):
                    continue
                _, _, _, max_acc = limits
                a = abs(cont_state.desired.accelerations[i])
                ratio = a / max_acc if max_acc > 0 else 0
                if ratio > desired_acc_max_ratio:
                    desired_acc_max_ratio = ratio
                if a > max_acc * ACC_TOL_RATIO:
                    errs.append(
                        f"{ts} TOPP-RA planned acceleration for {name}: "
                        f"{a:.4f} > limit {max_acc} rad/s^2"
                    )

        # 1.4 NaN/INF on actual positions (controller-level)
        for i, pos in enumerate(cont_state.actual.positions):
            if math.isnan(pos) or math.isinf(pos):
                name = cont_state.joint_names[i] if i < len(cont_state.joint_names) else f"joint{i}"
                errs.append(f"{ts} {name}'s actual position is NaN/INF")

        # 1.5 Planner desired.positions outside joint limits
        if cont_state.desired.positions:
            for i, name in enumerate(cont_state.joint_names):
                limits = PANDA_JOINT_LIMITS.get(name)
                if limits is None or i >= len(cont_state.desired.positions):
                    continue
                min_pos, max_pos, _, _ = limits
                dpos = cont_state.desired.positions[i]
                if dpos < (min_pos - MARGIN_RAD) or dpos > (max_pos + MARGIN_RAD):
                    errs.append(
                        f"{ts} planner desired position for {name}: "
                        f"{dpos:.4f} outside [{min_pos:.4f}, {max_pos:.4f}] rad"
                    )

    # =================================================================
    # Layer 2: Planning-Execution Gap (tracking error + endpoint)
    # =================================================================

    # --- 2.1 Trajectory Tracking Error ---
    max_tracking_error = 0.0
    tracking_error_values = []
    sample_idx = 0

    if data_sufficient:
        for (ts, cont_state) in controller_states_list:
            if not cont_state.error.positions:
                continue
            sample_idx += 1
            if sample_idx <= TRACKING_SKIP_SAMPLES:
                continue
            for ei, epos in enumerate(cont_state.error.positions):
                abs_err = abs(epos)
                tracking_error_values.append(abs_err)
                if abs_err > max_tracking_error:
                    max_tracking_error = abs_err
                if abs_err > TOL_TRACKING:
                    jn = (cont_state.joint_names[ei]
                          if ei < len(cont_state.joint_names)
                          else f"joint{ei}")
                    errs.append(
                        f"{ts} {jn}'s tracking error {epos:.4f} rad "
                        f"exceeds tolerance {TOL_TRACKING} rad"
                    )

    # --- 2.2 & 2.3 Endpoint Verification (FK) ---
    # Determine action terminal status
    num_goals_detected = 0
    last_action_terminal = None
    goal_terminal_statuses = []

    if move_action_status_list:
        (_, last_action_msg) = move_action_status_list[-1]
        if last_action_msg.status_list:
            goal_terminal_statuses = [
                gs.status for gs in last_action_msg.status_list
            ]
            num_goals_detected = len(goal_terminal_statuses)
            last_action_terminal = goal_terminal_statuses[-1]
            print(f"[oracle] {num_goals_detected} goals, "
                  f"statuses: {goal_terminal_statuses}")

    # Find the LAST SUCCEEDED goal's index (used by tracking-error check 3.2)
    last_success_idx = None
    for i in range(len(goal_terminal_statuses) - 1, -1, -1):
        if goal_terminal_statuses[i] == 4:
            last_success_idx = i
            break

    # FK endpoint computation — compare final EE pos against the goal MoveIt
    # ACTUALLY planned for last, taken from the last /motion_plan_request.
    #
    # Why MPR instead of msg_list[idx]: MoveIt2 silently skips unreachable goals
    # (no planning request → no action status), so action-status indices drift
    # from msg_list indices. Comparing the final end-effector pose against the
    # LAST MPR's goal sidesteps the drift entirely: the last MPR is, by
    # definition, the last goal the planner committed to.
    dist_goal_to_final_pos = None
    last_mpr_goal = None
    if motion_plan_request_list:
        try:
            (_, last_mpr) = motion_plan_request_list[-1]
            gc = last_mpr.goal_constraints[0]
            gp = gc.position_constraints[0] \
                .constraint_region.primitive_poses[0].position
            last_mpr_goal = (gp.x, gp.y, gp.z)
        except (IndexError, AttributeError):
            last_mpr_goal = None

    # Endpoint deviation is meaningful only when the final action terminated in
    # SUCCESS — a successful goal MUST leave the EE at the planned target.
    #
    # Source of truth = /panda_arm_controller/state .actual (the controller's
    # driven joint positions), NOT /joint_states. Two reasons:
    #  1) /joint_states suffers a multi-publisher collision (the persistent
    #     move_group's static state publisher interleaves with the live one),
    #     so consecutive samples flip between the real arm pose and the ready
    #     pose — a single-sample pick can grab the wrong one (false positive).
    #  2) The controller state is single-source and reflects what actually drove
    #     the arm. We take the sample at/just before goal-completion time so we
    #     read the arm AT the goal, before the next launch moves it away.
    endpoint_arm_positions = None   # dict: arm joint name -> position
    if move_action_status_list and controller_states_list:
        last_status_ts = move_action_status_list[-1][0]
        chosen = None
        for (ts, cont_state) in controller_states_list:
            if ts <= last_status_ts:
                chosen = cont_state
            else:
                break
        if chosen is None and controller_states_list:
            chosen = controller_states_list[-1][1]
        if chosen is not None:
            endpoint_arm_positions = {
                name: chosen.actual.positions[i]
                for i, name in enumerate(chosen.joint_names)
                if i < len(chosen.actual.positions)
            }

    endpoint_check_valid = (
        last_mpr_goal is not None
        and last_action_terminal == 4
        and endpoint_arm_positions is not None
    )
    if endpoint_check_valid:
        try:
            goal_x, goal_y, goal_z = last_mpr_goal

            with open(PANDA_URDF) as f:
                urdf_content = f.read()
            chain = kinpy.build_chain_from_urdf(urdf_content)

            # Arm joints from the controller; finger joints are not part of the
            # arm controller, default them to 0 for FK of panda_hand.
            joint_angle_map = dict(endpoint_arm_positions)
            joint_angle_map.setdefault("panda_finger_joint1", 0.0)
            joint_angle_map.setdefault("panda_finger_joint2", 0.0)

            fwd_kin = chain.forward_kinematics(joint_angle_map)
            final_ee_pos = fwd_kin["panda_hand"].pos

            dist_goal_to_final_pos = math.sqrt(
                pow(goal_x - final_ee_pos[0], 2)
                + pow(goal_y - final_ee_pos[1], 2)
                + pow(goal_z - final_ee_pos[2], 2)
            )
            print(f"[oracle] endpoint deviation: {dist_goal_to_final_pos:.6f}m "
                  f"(last MPR goal=({goal_x:.3f},{goal_y:.3f},{goal_z:.3f}), "
                  f"actual=({final_ee_pos[0]:.3f},{final_ee_pos[1]:.3f},{final_ee_pos[2]:.3f}))")
        except (IndexError, AttributeError, FileNotFoundError, TypeError):
            pass

    # =================================================================
    # Layer 3: Behavioral Consistency (semantic logic checks)
    # =================================================================

    # --- 3.1 Success-but-not-there ---
    # The last goal MoveIt planned for SUCCEEDED, yet the end-effector did not
    # arrive at that goal. This now correctly catches the real bug "reported
    # SUCCESS on a goal the arm never reached", because the comparison target is
    # the planner's own last goal (no index drift).
    if endpoint_check_valid and dist_goal_to_final_pos is not None:
        if dist_goal_to_final_pos > TOL_ENDPOINT:
            errs.append(
                f"goal and actual pos deviation too high: "
                f"{dist_goal_to_final_pos}"
            )

    # --- 3.2 Success-but-high-tracking-error ---
    if last_success_idx is not None and tracking_error_values:
        mean_err = statistics.mean(tracking_error_values)
        if mean_err > 0.05:
            errs.append(
                f"goal succeeded but mean tracking error high: "
                f"{mean_err:.4f} rad"
            )

    # --- 3.3 Abort-but-still-moving ---
    if (last_action_terminal == 6
            and move_action_status_list
            and len(controller_states_list) >= 2):
        prev_terminal_count = 0
        for as_ts, as_msg in move_action_status_list:
            statuses = [s.status for s in as_msg.status_list]
            curr_terminal = sum(1 for s in statuses if s in (4, 6))
            if curr_terminal > prev_terminal_count:
                new_status = (statuses[curr_terminal - 1]
                              if curr_terminal <= len(statuses) else 0)
                if new_status == 6:
                    abort_idx = 0
                    for ci, (cts, _) in enumerate(controller_states_list):
                        if cts <= as_ts:
                            abort_idx = ci
                    if abort_idx < len(controller_states_list) - 1:
                        abort_pos = list(
                            controller_states_list[abort_idx][1].actual.positions
                        )
                        abort_ts_ns = controller_states_list[abort_idx][0]
                        for ci in range(abort_idx + 1, len(controller_states_list)):
                            dt = (controller_states_list[ci][0] - abort_ts_ns) / 1e9
                            if dt > 0.5:
                                break
                            post_pos = list(
                                controller_states_list[ci][1].actual.positions
                            )
                            for j in range(min(len(abort_pos), len(post_pos))):
                                drift = abs(post_pos[j] - abort_pos[j])
                                if drift > TOL_ABORT_DRIFT:
                                    errs.append(
                                        f"abort drift: joint moved {drift:.4f} "
                                        f"rad within {dt:.3f}s after abort"
                                    )
                                    break
                            else:
                                continue
                            break
            prev_terminal_count = curr_terminal

    # --- 3.4 Action Status Anomaly ---
    if goal_terminal_statuses:
        for gi, gstatus in enumerate(goal_terminal_statuses):
            if gstatus not in (4, 6):
                errs.append(
                    f"goal {gi}: unexpected terminal status {gstatus} "
                    f"(expected 4=SUCCESS or 6=ABORTED)"
                )
    elif move_action_status_list:
        pass  # status received but no terminal — still executing
    else:
        # No action status at all. Only report as bug if:
        # 1) At least one goal appears reachable (within reliable workspace)
        # 2) AND motion_plan_requests were received (system was responsive)
        # If no MPRs exist either, the system wasn't ready → environment issue.
        if (not _all_goals_unreachable(msg_list)
                and motion_plan_request_list):
            errs.append("no action status received")

    # =================================================================
    # Feedback Updates (compatible with fuzzer.py's feedback definitions)
    # =================================================================

    # Compute additional metrics for feedback (no error reporting)
    joint_pos_min = {}
    joint_pos_max = {}
    for (ts, cont_state) in controller_states_list:
        for pi, pos in enumerate(cont_state.actual.positions):
            jname = cont_state.joint_names[pi] if pi < len(cont_state.joint_names) else ""
            if jname not in PANDA_JOINT_LIMITS:
                continue
            if jname not in joint_pos_min:
                joint_pos_min[jname] = pos
                joint_pos_max[jname] = pos
            else:
                if pos < joint_pos_min[jname]:
                    joint_pos_min[jname] = pos
                if pos > joint_pos_max[jname]:
                    joint_pos_max[jname] = pos

    for feedback in feedback_list:
        if feedback.name == "end_point_deviation":
            if dist_goal_to_final_pos is not None:
                feedback.update_value(dist_goal_to_final_pos)

        elif feedback.name in ("max_tracking_error", "max_joint_pos_error"):
            if max_tracking_error > 0.0:
                feedback.update_value(max_tracking_error)

        elif feedback.name in ("mean_tracking_error", "mean_joint_pos_error",
                               "trajectory_tracking_rms"):
            if tracking_error_values:
                if "rms" in feedback.name:
                    rms = math.sqrt(
                        sum(v * v for v in tracking_error_values)
                        / len(tracking_error_values)
                    )
                    feedback.update_value(rms)
                else:
                    feedback.update_value(statistics.mean(tracking_error_values))

        elif feedback.name == "goal_success_ratio":
            if goal_terminal_statuses:
                n_success = sum(1 for s in goal_terminal_statuses if s == 4)
                fail_ratio = 1.0 - (n_success / len(goal_terminal_statuses))
                feedback.update_value(fail_ratio)

        elif feedback.name == "workspace_boundary_distance":
            if motion_plan_request_list:
                max_prox = 0.0
                for (_, mpr_msg) in motion_plan_request_list:
                    try:
                        gp = mpr_msg.goal_constraints[0].position_constraints[0] \
                            .constraint_region.primitive_poses[0].position
                        dist = math.sqrt(gp.x ** 2 + gp.y ** 2 + gp.z ** 2)
                        prox = 1.0 / max(0.01, abs(dist - WORKSPACE_RADIUS))
                        if prox > max_prox:
                            max_prox = prox
                    except (IndexError, AttributeError):
                        continue
                if max_prox > 0.0:
                    feedback.update_value(max_prox)

        elif feedback.name == "abort_joint_drift":
            if last_action_terminal == 6 and final_joint_state is not None:
                arm_joints = [f"panda_joint{i}" for i in range(1, 8)]
                max_drift = 0.0
                for i, jname in enumerate(final_joint_state.name):
                    if jname in arm_joints:
                        idx = arm_joints.index(jname)
                        drift = abs(final_joint_state.position[i] - PANDA_HOME[idx])
                        if drift > max_drift:
                            max_drift = drift
                if max_drift > 0.0:
                    feedback.update_value(max_drift)

        elif feedback.name == "planning_duration":
            if motion_plan_request_list and move_action_status_list:
                plan_first_ts = motion_plan_request_list[0][0]
                status_last_ts = move_action_status_list[-1][0]
                dur = (status_last_ts - plan_first_ts) / 1e9
                if dur > 0:
                    feedback.update_value(dur)

        elif feedback.name in ("desired_vel_max_ratio", "max_velocity_margin"):
            # max_velocity_margin (DEC type): how close to limit
            # desired_vel_max_ratio (INC type): ratio of max vel to limit
            if feedback.name == "max_velocity_margin":
                if desired_vel_max_ratio > 0:
                    margin = 1.0 - desired_vel_max_ratio  # closer to 0 = closer to limit
                    feedback.update_value(margin)
            else:
                if desired_vel_max_ratio > 0:
                    feedback.update_value(desired_vel_max_ratio)

        elif feedback.name == "desired_acc_max_ratio":
            if desired_acc_max_ratio > 0:
                feedback.update_value(desired_acc_max_ratio)

        elif feedback.name == "joint_motion_range":
            max_range = 0.0
            for jname in joint_pos_min:
                r = joint_pos_max[jname] - joint_pos_min[jname]
                if r > max_range:
                    max_range = r
            if max_range > 0.0:
                feedback.update_value(max_range)

        elif feedback.name == "goal_transition_error":
            # Use abort drift if available
            if last_action_terminal == 6 and final_joint_state is not None:
                arm_joints = [f"panda_joint{i}" for i in range(1, 8)]
                max_d = 0.0
                for i, jname in enumerate(final_joint_state.name):
                    if jname in arm_joints:
                        idx = arm_joints.index(jname)
                        d = abs(final_joint_state.position[i] - PANDA_HOME[idx])
                        if d > max_d:
                            max_d = d
                if max_d > 0.0:
                    feedback.update_value(max_d)

    return errs
