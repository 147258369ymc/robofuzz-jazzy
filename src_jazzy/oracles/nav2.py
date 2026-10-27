"""nav2 system-level semantic oracle (see checker.run_checks dispatch).

Tests a correctness property RoFER's crash oracle cannot express: with
localization pinned to simulator ground truth (map->odom = truth), does the
nav2 planning+control+behavior-tree stack navigate correctly? Criteria:
arrival honesty, collision-freedom, liveness (no hang/stuck), and no
over-conservative failure. Thresholds are calibrated from a clean campaign.

Architecture note: This oracle focuses on coarse-grained semantic criteria.
The 42 OracleIR specs in oracle_ir/specs/nav2_jazzy/ encode fine-grained
parameter constraints (MPPI controller bounds, costmap inflation, planner
tolerances) and per-sample checks that require additional topic streams
(/cmd_vel, /plan, action feedback). These OracleIR specs serve as a reference
for future expansion—if per-sample velocity/acceleration/clearance checks are
needed, their observation bindings and threshold formulas are documented there.
The feedback metrics below are derived from OracleIR insights but computed from
the minimal topic set (goal/result/truth/collision) that the current bag
recording captures.
"""

import math
import numpy as np  # reserved for later feedback stats; keep import stable

from oracles.slam import (
    _extract_pose, _message_time_sec, _wrap_to_pi, _pose_track,
)

NAV2_STATUS_SUCCEEDED = 4
NAV2_STATUS_CANCELED = 5
NAV2_STATUS_ABORTED = 6
NAV2_STATUS_TIMEOUT = -6

NAV2_ARRIVAL_XY_TOL = 0.30   # m, goal_checker xy(0.25)+0.05 margin; clean max 0.134 well under
NAV2_ARRIVAL_YAW_TOL = 0.35  # rad, yaw_goal_tolerance(0.25) x ~1.4
NAV2_MIN_SAMPLES = 10
NAV2_STUCK_DISP_MIN = 0.10  # m, min net displacement below which "stuck"

# New oracle (Criterion 5): navigation time efficiency — the only process-level
# oracle retained after clean-campaign calibration. Path-efficiency and
# pose-oscillation oracles were REMOVED: calibration over 270 clean rounds showed
# path_efficiency max=1.60 (never exceeds a meaningful threshold — nav2 paths are
# direct) and pose_oscillation was identically 0 (nav2 does not oscillate in this
# arena). They produced no signal and were dropped.
#
# Time-efficiency threshold CALIBRATED from 270 clean rounds: distribution was
# median=2.14, p95=5.60, p99=6.68, max=8.02. The old hand-picked 3.0 flagged ~40%
# of NORMAL rounds as bugs (the 62 "bugs" were mostly false positives). New
# threshold = p95(5.60) x 1.3 ≈ 7.3 only flags genuinely outlier-slow navigation.
NAV2_TIME_EFFICIENCY_FACTOR = 7.3     # calibrated: clean p95(5.6) x 1.3
NAV2_EXPECTED_SPEED = 0.3             # m/s, baseline for time-efficiency ratio

GOAL_TOPIC = "/robofuzz/nav2_goal"
RESULT_TOPIC = "/robofuzz/nav2_result"
GROUND_TRUTH_TOPIC = "/sim_ground_truth_pose"
COLLISION_TOPIC = "/bumper_contact"

# Extension point (NOT implemented this spike): a second localization source
# (e.g. amcl) for the future Q2=A drift oracle. Left as a constant so the
# migration path exists at zero cost.
SECONDARY_LOCALIZATION_TOPIC = None

# Costmap inflation geometry (tb4 nav2.yaml: inflation_radius 0.45, robot_radius
# 0.22). A goal closer than this to the nearest occupied/unknown cell lies
# inside nav2's inflated costmap, where nav2 legitimately ABORTs — so a
# FALSE_FAILURE there is a judge false-positive (BFS-reachable != costmap-
# plannable), not a real over-conservatism bug. Used to gate criterion 4.
NAV2_INFLATION_RADIUS = 0.45
NAV2_ROBOT_RADIUS = 0.22
NAV2_FALSE_FAILURE_CLEARANCE = NAV2_INFLATION_RADIUS + NAV2_ROBOT_RADIUS  # 0.67 m

# Lazily-loaded occupancy map for the inflation-clearance gate. None if the map
# cannot be loaded (then the gate degrades to "not in inflation" = permissive,
# preserving prior behavior rather than crashing).
_MAP_CTX = None
_MAP_LOAD_TRIED = False


def _get_map_ctx():
    """Lazily load the occupancy map once (env NAV2_MAP or repo default)."""
    global _MAP_CTX, _MAP_LOAD_TRIED
    if _MAP_LOAD_TRIED:
        return _MAP_CTX
    _MAP_LOAD_TRIED = True
    try:
        import os
        import nav2_map
        path = os.environ.get(
            "NAV2_MAP",
            os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(
                    os.path.abspath(__file__)))),
                "maps", "slam_arena.yaml"),
        )
        _MAP_CTX = nav2_map.load_occupancy_map(path)
    except Exception:
        _MAP_CTX = None
    return _MAP_CTX


def _goal_clearance(gx, gy, max_r=1.0):
    """Distance (m) from goal to nearest occupied/unknown cell, capped at max_r.

    Returns max_r if the map is unavailable or the goal is out of bounds (both
    treated as 'open space' so the gate stays permissive rather than crashing).
    """
    m = _get_map_ctx()
    if m is None:
        return max_r
    gc, gr = m.world_to_cell(gx, gy)
    if not m._in_bounds(gc, gr):
        return max_r
    steps = int(max_r / m.resolution)
    best = max_r
    for dr in range(-steps, steps + 1):
        for dc in range(-steps, steps + 1):
            c, r = gc + dc, gr + dr
            if m._in_bounds(c, r) and not m.is_free(c, r):
                d = math.hypot(dc, dr) * m.resolution
                if d < best:
                    best = d
    return best

NAV2_FEEDBACK_NAMES = (
    # Oracle status and input signals
    "nav2_oracle_valid",
    "nav2_result_code",
    "nav2_truth_sample_count",

    # Arrival errors (absolute, in meters/radians)
    "nav2_arrival_xy_error",
    "nav2_arrival_yaw_error",

    # Arrival margins (tolerance - error; positive = within budget, negative = violated)
    # Derived from OracleIR feedback specs: guides fuzzer toward boundary conditions
    "nav2_arrival_xy_margin",
    "nav2_arrival_yaw_margin",

    # Arrival pressure (error / tolerance; >1.0 = violation)
    # Normalized metric for fuzzer gradient: maximize to find boundary-crossing inputs
    "nav2_arrival_xy_pressure",
    "nav2_arrival_yaw_pressure",

    # Collision and motion metrics
    "nav2_collision_detected",
    "nav2_net_displacement",
    "nav2_navigation_duration",

    # Trajectory efficiency metrics (derived from OracleIR insights)
    "nav2_avg_velocity",          # net_displacement / duration
    "nav2_stuck_pressure",        # 1.0 - (net_disp / STUCK_DISP_MIN); >1.0 = stuck

    # Process-level metric (Oracle 5, calibrated). Path-efficiency and
    # pose-oscillation feedbacks removed: no signal in this arena (see check()).
    "nav2_time_efficiency",       # duration / expected_time
)


def _update_feedback(feedback_list, name, value):
    for fbk in feedback_list or []:
        if fbk.name == name:
            fbk.update_value(value)
            return


def _latest_goal(records):
    track = _pose_track(records)
    if not track:
        return None
    last = track[-1]
    return (last[1], last[2], last[3])


def _latest_result(records):
    best_t, best_val = None, None
    for ts, msg in records or []:
        try:
            val = int(msg.data)
            t = _message_time_sec(msg, ts)
        except (AttributeError, TypeError, ValueError):
            continue
        if best_t is None or t >= best_t:
            best_t, best_val = t, val
    return best_val


def _arrival_error(goal, truth_track):
    if goal is None or len(truth_track) < NAV2_MIN_SAMPLES:
        return None, 0.0, 0.0
    tx, ty, tyaw = truth_track[-1][1], truth_track[-1][2], truth_track[-1][3]
    xy = math.hypot(tx - goal[0], ty - goal[1])
    yaw = abs(_wrap_to_pi(tyaw - goal[2]))
    if xy > NAV2_ARRIVAL_XY_TOL or yaw > NAV2_ARRIVAL_YAW_TOL:
        return (
            f"NAV2_ARRIVAL_ERROR:pose source=nav2 xy_m={xy:.3f} "
            f"yaw_rad={yaw:.3f} xy_limit={NAV2_ARRIVAL_XY_TOL} "
            f"yaw_limit={NAV2_ARRIVAL_YAW_TOL}"
        ), xy, yaw
    return None, xy, yaw


def _round_has_collision(state_dict):
    for _, msg in state_dict.get(COLLISION_TOPIC, []) or []:
        if getattr(msg, "contacts", None):
            return True
    return False


def _net_displacement(truth_track):
    if len(truth_track) < 2:
        return 0.0
    a, b = truth_track[0], truth_track[-1]
    return math.hypot(b[1] - a[1], b[2] - a[2])


def _actual_path_length(truth_track):
    """Compute total path length by summing distances between consecutive poses."""
    if len(truth_track) < 2:
        return 0.0
    total = 0.0
    for i in range(len(truth_track) - 1):
        dx = truth_track[i+1][1] - truth_track[i][1]
        dy = truth_track[i+1][2] - truth_track[i][2]
        total += math.hypot(dx, dy)
    return total


def _pose_oscillation_count(truth_track):
    """Count yaw direction reversals (high-frequency oscillation indicator)."""
    if len(truth_track) < 3:
        return 0
    yaw_changes = []
    for i in range(len(truth_track) - 1):
        dyaw = _wrap_to_pi(truth_track[i+1][3] - truth_track[i][3])
        if abs(dyaw) > 0.05:  # Filter noise: only count significant changes
            yaw_changes.append(dyaw)

    if len(yaw_changes) < 2:
        return 0

    # Count direction reversals: sign(yaw_changes[i]) != sign(yaw_changes[i-1])
    reversals = 0
    for i in range(1, len(yaw_changes)):
        if yaw_changes[i] * yaw_changes[i-1] < 0:  # Opposite signs
            reversals += 1
    return reversals


def check(config, msg_list, state_dict, feedback_list):
    errs = []
    for name in NAV2_FEEDBACK_NAMES:
        _update_feedback(feedback_list, name, 0.0)

    goal = _latest_goal(state_dict.get(GOAL_TOPIC, []))
    result = _latest_result(state_dict.get(RESULT_TOPIC, []))
    truth_track = _pose_track(state_dict.get(GROUND_TRUTH_TOPIC, []))

    _update_feedback(feedback_list, "nav2_truth_sample_count",
                     float(len(truth_track)))
    if result is not None:
        _update_feedback(feedback_list, "nav2_result_code", float(result))

    # Missing goal/result/truth -> readiness/recording issue, not a bug.
    if goal is None or result is None or not truth_track:
        return errs

    _update_feedback(feedback_list, "nav2_oracle_valid", 1.0)

    net_disp = _net_displacement(truth_track)
    _update_feedback(feedback_list, "nav2_net_displacement", net_disp)
    duration = truth_track[-1][0] - truth_track[0][0]
    _update_feedback(feedback_list, "nav2_navigation_duration", duration)

    # Compute arrival errors and derived metrics (margin/pressure from OracleIR)
    _, arr_xy, arr_yaw = _arrival_error(goal, truth_track)
    reached = (
        len(truth_track) >= NAV2_MIN_SAMPLES
        and arr_xy <= NAV2_ARRIVAL_XY_TOL
    )

    # Arrival margins: tolerance - error (positive = safe, negative = violated)
    # These guide the fuzzer to explore boundary conditions around the threshold
    xy_margin = NAV2_ARRIVAL_XY_TOL - arr_xy
    yaw_margin = NAV2_ARRIVAL_YAW_TOL - arr_yaw
    _update_feedback(feedback_list, "nav2_arrival_xy_margin", xy_margin)
    _update_feedback(feedback_list, "nav2_arrival_yaw_margin", yaw_margin)

    # Arrival pressure: normalized error / tolerance (>1.0 = violation)
    # Maximize direction: fuzzer tries to push error toward and over the threshold
    xy_pressure = arr_xy / NAV2_ARRIVAL_XY_TOL if NAV2_ARRIVAL_XY_TOL > 0 else 0.0
    yaw_pressure = arr_yaw / NAV2_ARRIVAL_YAW_TOL if NAV2_ARRIVAL_YAW_TOL > 0 else 0.0
    _update_feedback(feedback_list, "nav2_arrival_xy_pressure", xy_pressure)
    _update_feedback(feedback_list, "nav2_arrival_yaw_pressure", yaw_pressure)

    # Motion efficiency metrics (insight from OracleIR trajectory analysis)
    avg_velocity = net_disp / duration if duration > 0 else 0.0
    _update_feedback(feedback_list, "nav2_avg_velocity", avg_velocity)

    # Stuck pressure: 1.0 - (net_disp / STUCK_MIN); >1.0 = stuck condition
    # Derived from NAV2_STUCK_DISP_MIN oracle threshold (0.10 m)
    stuck_pressure = 1.0 - (net_disp / NAV2_STUCK_DISP_MIN) if NAV2_STUCK_DISP_MIN > 0 else 0.0
    _update_feedback(feedback_list, "nav2_stuck_pressure", max(0.0, stuck_pressure))

    # Shared judgment evidence appended to every error so truth can be assessed
    # from the error file alone, without the (per-round-overwritten) bag. goal
    # is the ACTUAL sent goal read from /robofuzz/nav2_goal (post-snap), NOT the
    # raw mutated value; truth_end is the ground-truth final pose; clearance is
    # the goal's distance to the nearest obstacle (for the inflation gate).
    tend = truth_track[-1]
    clearance = _goal_clearance(goal[0], goal[1])
    evidence = (
        f"goal_x={goal[0]:.3f} goal_y={goal[1]:.3f} goal_yaw={goal[2]:.3f} "
        f"truth_end_x={tend[1]:.3f} truth_end_y={tend[2]:.3f} "
        f"result={result} truth_samples={len(truth_track)} "
        f"net_disp_m={net_disp:.3f} duration_s={duration:.1f} "
        f"arr_xy_m={arr_xy:.3f} clearance_m={clearance:.3f}"
    )

    # Criterion 2: collision. Unlike slam (excludes the round), here contact
    # IS the bug: goal is in free space and localization is truth, so hitting
    # an obstacle can only be a costmap/controller avoidance failure.
    if _round_has_collision(state_dict):
        _update_feedback(feedback_list, "nav2_collision_detected", 1.0)
        errs.append(f"NAV2_COLLISION_ERROR:pose source=nav2 contact=1 {evidence}")

    # Criterion 3: hang/stuck (liveness). Goal passed the reachability gate.
    if result == NAV2_STATUS_TIMEOUT:
        errs.append(f"NAV2_TIMEOUT_ERROR:pose source=nav2 {evidence}")
    elif result != NAV2_STATUS_SUCCEEDED and not reached \
            and net_disp < NAV2_STUCK_DISP_MIN:
        errs.append(
            f"NAV2_STUCK_ERROR:pose source=nav2 limit_m={NAV2_STUCK_DISP_MIN} "
            f"{evidence}"
        )

    # Criterion 4: false failure. Reachable goal (gate) + real motion but
    # ABORTED -> over-conservative. Gated on inflation clearance: a goal inside
    # nav2's inflated costmap (clearance < inflation+robot_radius) is
    # legitimately abortable (BFS-reachable != costmap-plannable), so it is NOT
    # a false failure. Only open-space ABORTs with real motion are flagged.
    if result == NAV2_STATUS_ABORTED and net_disp >= NAV2_STUCK_DISP_MIN \
            and not reached:
        if clearance >= NAV2_FALSE_FAILURE_CLEARANCE:
            errs.append(f"NAV2_FALSE_FAILURE:pose source=nav2 {evidence}")
        else:
            # Suppressed: goal inside inflation layer, nav2 ABORT is legitimate.
            errs.append(
                f"NAV2_ABORT_IN_INFLATION:info source=nav2 "
                f"clearance_limit_m={NAV2_FALSE_FAILURE_CLEARANCE:.2f} {evidence}"
            )

    if result == NAV2_STATUS_SUCCEEDED:
        arr_err, xy, yaw = _arrival_error(goal, truth_track)
        _update_feedback(feedback_list, "nav2_arrival_xy_error", xy)
        _update_feedback(feedback_list, "nav2_arrival_yaw_error", yaw)
        if arr_err:
            errs.append(f"{arr_err} {evidence}")

        # Criterion 5: Navigation time efficiency (calibrated). Duration vs
        # expected time. Detects genuinely outlier-slow navigation (progress
        # checker too loose, controller stalls, repeated recovery). Threshold
        # calibrated from 270 clean rounds (p95=5.6 x 1.3 = 7.3); the old 3.0
        # flagged ~40% of normal rounds. Path-efficiency and pose-oscillation
        # oracles were removed after calibration showed they produce no signal
        # in this arena (paths are direct, no oscillation).
        straight_line = math.hypot(goal[0] - truth_track[0][1],
                                   goal[1] - truth_track[0][2])
        if straight_line > 0.5:  # Only check non-trivial goals
            expected_time = straight_line / NAV2_EXPECTED_SPEED
            time_efficiency = duration / expected_time
            _update_feedback(feedback_list, "nav2_time_efficiency", time_efficiency)
            if time_efficiency > NAV2_TIME_EFFICIENCY_FACTOR:
                errs.append(
                    f"NAV2_NAVIGATION_TIME_ERROR:pose source=nav2 "
                    f"time_efficiency={time_efficiency:.2f} threshold={NAV2_TIME_EFFICIENCY_FACTOR:.2f} "
                    f"expected_time_s={expected_time:.1f} {evidence}"
                )

    return errs
