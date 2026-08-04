"""nav2 system-level semantic oracle (see checker.run_checks dispatch).

Tests a correctness property RoFER's crash oracle cannot express: with
localization pinned to simulator ground truth (map->odom = truth), does the
nav2 planning+control+behavior-tree stack navigate correctly? Criteria:
arrival honesty, collision-freedom, liveness (no hang/stuck), and no
over-conservative failure. Thresholds are calibrated from a clean campaign.
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

NAV2_ARRIVAL_XY_TOL = 0.35   # m, nav2 goal_checker xy_goal_tolerance(0.25) x ~1.4
NAV2_ARRIVAL_YAW_TOL = 0.35  # rad, yaw_goal_tolerance(0.25) x ~1.4
NAV2_MIN_SAMPLES = 10

GOAL_TOPIC = "/robofuzz/nav2_goal"
RESULT_TOPIC = "/robofuzz/nav2_result"
GROUND_TRUTH_TOPIC = "/sim_ground_truth_pose"
COLLISION_TOPIC = "/bumper_contact"

# Extension point (NOT implemented this spike): a second localization source
# (e.g. amcl) for the future Q2=A drift oracle. Left as a constant so the
# migration path exists at zero cost.
SECONDARY_LOCALIZATION_TOPIC = None

NAV2_FEEDBACK_NAMES = (
    "nav2_oracle_valid",
    "nav2_result_code",
    "nav2_truth_sample_count",
    "nav2_arrival_xy_error",
    "nav2_arrival_yaw_error",
    "nav2_collision_detected",
    "nav2_net_displacement",
    "nav2_navigation_duration",
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

    if result == NAV2_STATUS_SUCCEEDED:
        arr_err, xy, yaw = _arrival_error(goal, truth_track)
        _update_feedback(feedback_list, "nav2_arrival_xy_error", xy)
        _update_feedback(feedback_list, "nav2_arrival_yaw_error", yaw)
        if arr_err:
            errs.append(arr_err)

    return errs
