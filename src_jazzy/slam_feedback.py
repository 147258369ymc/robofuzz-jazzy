"""Pure helpers for SLAM-specific feedback quality control."""

import hashlib
import math


SLAM_COLLISION_FEEDBACK_NAME = "slam_collision_detected"

# (name, minimum current value eligible for INC interestingness).  The
# precision thresholds deliberately resolve the 0.001--0.02 ranges observed
# in clean SLAM campaigns; the previous blanket 0.05 floor suppressed every
# trajectory-quality improvement.
SLAM_FEEDBACK_DEFINITIONS = (
    ("slam_oracle_valid", 1.0),
    ("slam_collision_detected", 1.0),
    ("slam_pose_sample_count", 1.0),
    ("slam_match_ratio", 0.01),
    ("slam_max_rpe_drift", 0.001),
    ("slam_displacement_magnitude_error", 0.001),
    ("slam_rpe_translation_rmse", 0.001),
    ("slam_rpe_rotation_max", 0.001),
    ("slam_rpe_rotation_rmse", 0.001),
    ("slam_ate_rmse", 0.001),
    ("slam_ate_max", 0.001),
    ("slam_ate_median", 0.001),
    ("slam_max_yaw_drift", 0.001),
    ("slam_odom_rpe_drift", 0.001),
    ("slam_pose_odom_excess_rpe", 0.001),
    ("slam_translation_excitation", 0.05),
    ("slam_rotation_excitation", 0.05),
    ("slam_scan_finite_ratio", 0.01),
    ("slam_scan_gate_opportunities", 1.0),
    ("slam_map_known_area", 0.001),
    ("slam_map_growth", 0.001),
    ("slam_revisit_count", 1.0),
    ("slam_loop_closure_residual", 0.01),
    ("slam_tf_coverage", 0.25),
    ("slam_tf_max_age", 0.05),
)


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
    rpe_rotation = _numeric_feedback_value(
        feedback_list, "slam_rpe_rotation_max"
    )
    odom_rpe = _numeric_feedback_value(
        feedback_list, "slam_odom_rpe_drift"
    )
    map_known_area = _numeric_feedback_value(
        feedback_list, "slam_map_known_area"
    )
    revisits = _numeric_feedback_value(
        feedback_list, "slam_revisit_count"
    )
    tf_coverage = _numeric_feedback_value(
        feedback_list, "slam_tf_coverage"
    )

    if collision:
        status = "collision"
    elif valid:
        status = "judged"
    else:
        status = "insufficient"

    return (
        f"[slam] status={status} valid={valid} collision={collision} "
        f"matched_pairs={matched_pairs:.0f} rpe_m={rpe:.3f} "
        f"ate_m={ate:.3f} yaw_rad={yaw:.3f} "
        f"rpe_rot_rad={rpe_rotation:.3f} odom_rpe_m={odom_rpe:.3f} "
        f"map_known_m2={map_known_area:.3f} revisits={revisits:.0f} "
        f"tf_coverage={tf_coverage:.3f}"
    )


def slam_error_signature(errs):
    """Return deterministic SLAM error categories for queue novelty."""
    signatures = set()
    for error in errs or []:
        text = str(error)
        category, separator, detail = text.partition(":")
        if separator and category.startswith("SLAM_"):
            source = detail.split(None, 1)[0]
            signatures.add((category.lower(), source))
            continue
        digest = hashlib.sha256(text.encode("utf-8")).hexdigest()[:12]
        signatures.add(("slam_other", digest))
    return signatures


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
