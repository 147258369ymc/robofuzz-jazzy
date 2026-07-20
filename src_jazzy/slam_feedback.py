"""Pure helpers for SLAM-specific feedback quality control."""

import math


SLAM_COLLISION_FEEDBACK_NAME = "slam_collision_detected"


def _numeric_feedback_value(feedback_list, name):
    """Return a finite numeric feedback value, defaulting invalid data to 0."""
    for fbk in feedback_list or []:
        if getattr(fbk, "name", None) != name:
            continue
        try:
            value = float(getattr(fbk, "value", 0.0))
        except (TypeError, ValueError):
            return 0.0
        return value if math.isfinite(value) else 0.0
    return 0.0


def format_slam_feedback_status(feedback_list):
    """Format one stable, human-readable summary of the current SLAM round."""
    valid = int(
        _numeric_feedback_value(feedback_list, "slam_oracle_valid") >= 1.0
    )
    collision = int(
        _numeric_feedback_value(
            feedback_list, SLAM_COLLISION_FEEDBACK_NAME
        ) >= 1.0
    )
    matched_pairs = _numeric_feedback_value(
        feedback_list, "slam_pose_sample_count"
    )
    rpe = _numeric_feedback_value(feedback_list, "slam_max_rpe_drift")
    ate = _numeric_feedback_value(feedback_list, "slam_ate_rmse")
    yaw = _numeric_feedback_value(feedback_list, "slam_max_yaw_drift")

    if collision:
        status = "collision"
    elif valid:
        status = "judged"
    else:
        status = "insufficient"

    return (
        f"[slam] status={status} valid={valid} collision={collision} "
        f"matched_pairs={matched_pairs:.0f} rpe_m={rpe:.3f} "
        f"ate_m={ate:.3f} yaw_rad={yaw:.3f}"
    )


def round_has_slam_collision(feedback_list):
    """Return whether the current round's feedback reports a collision."""
    for fbk in feedback_list or []:
        if fbk.name != SLAM_COLLISION_FEEDBACK_NAME:
            continue
        try:
            return float(fbk.value) >= 1.0
        except (TypeError, ValueError):
            return False
    return False


def apply_slam_feedback_quality_gate(is_interesting, feedback_list):
    """Reject an otherwise-interesting SLAM round that collided."""
    return bool(is_interesting) and not round_has_slam_collision(feedback_list)


def evaluate_slam_feedback_interest(feedback_list):
    """Evaluate SLAM feedback without admitting collisions into history."""
    if round_has_slam_collision(feedback_list):
        return False

    is_interesting = False
    for fbk in feedback_list or []:
        cur_interesting = fbk.is_interesting()
        is_interesting = is_interesting or cur_interesting
    return is_interesting
