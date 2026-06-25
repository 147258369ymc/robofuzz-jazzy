"""Bucketed semantic feedback novelty for MoveIt fuzzing.

High-watermark feedback saturates during long MoveIt runs. Bucket novelty keeps
behaviorally different, still-valid seeds alive without changing the generic
Feedback class or affecting non-MoveIt targets.
"""

import math


# metric -> (bucket_width, min_value_to_track, max_value_to_track)
MOVEIT_BUCKETED_FEEDBACK = {
    "desired_vel_max_ratio": (0.1, 0.2, 2.0),
    "desired_acc_max_ratio": (0.1, 0.2, 2.0),
    "desired_jerk_max_ratio": (0.1, 0.2, 3.0),
    "smoothness_violation_ratio": (0.1, 0.2, 3.0),
    "velocity_roughness": (5.0, 5.0, 100.0),
    "joint_motion_range": (0.5, 0.5, 8.0),
    "planning_duration": (10.0, 10.0, 120.0),
}


def feedback_bucket_key(feedback):
    config = MOVEIT_BUCKETED_FEEDBACK.get(getattr(feedback, "name", None))
    if config is None:
        return None

    value = getattr(feedback, "value", None)
    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value):
        return None

    width, min_value, max_value = config
    if value < min_value:
        return None
    capped = min(value, max_value)
    bucket = int(capped / width)
    return (feedback.name, bucket)


def collect_new_feedback_buckets(feedback_list, seen_buckets):
    """Update ``seen_buckets`` and return newly observed MoveIt bucket keys."""
    new_keys = []
    for feedback in feedback_list:
        key = feedback_bucket_key(feedback)
        if key is None or key in seen_buckets:
            continue
        seen_buckets.add(key)
        new_keys.append(key)
    return new_keys
