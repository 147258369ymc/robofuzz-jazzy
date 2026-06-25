"""MoveIt MotionPlanRequest planning parameter side-channel helpers.

The fuzzer keeps MoveIt inputs as pure ``geometry_msgs/Pose`` objects for
queueing and serialization compatibility. These helpers carry per-sequence
planning knobs separately and publish them as JSON for oracle correlation.
"""

import json


PLAN_PARAMS_TOPIC = "/robofuzz/moveit_plan_params"

DEFAULT_MOVEIT_PLAN_PARAMS = {
    "velocity_scaling": 0.1,
    "acceleration_scaling": 0.1,
    "planning_time": 5.0,
    "position_tolerance": 0.01,
    "orientation_tolerance": 0.5,
}

PLAN_PARAM_RANGES = {
    "velocity_scaling": (0.01, 1.0),
    "acceleration_scaling": (0.01, 1.0),
    "planning_time": (0.1, 30.0),
    "position_tolerance": (0.0005, 0.10),
    "orientation_tolerance": (0.01, 1.57),
}


def _clamp(value, low, high):
    return max(low, min(high, value))


def _float_or_default(value, default):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def normalize_plan_params(params=None):
    """Fill defaults and clamp all supported MoveIt planning parameters."""
    normalized = dict(DEFAULT_MOVEIT_PLAN_PARAMS)
    if params:
        for key in DEFAULT_MOVEIT_PLAN_PARAMS:
            if key in params:
                normalized[key] = params[key]

    for key, default in DEFAULT_MOVEIT_PLAN_PARAMS.items():
        low, high = PLAN_PARAM_RANGES[key]
        normalized[key] = _clamp(
            _float_or_default(normalized.get(key), default),
            low,
            high,
        )
    return normalized


def plan_params_to_json(params=None):
    """Serialize normalized plan parameters with stable key ordering."""
    return json.dumps(normalize_plan_params(params), sort_keys=True)


def plan_params_from_json(text):
    """Deserialize JSON into a normalized parameter dict."""
    raw = json.loads(text)
    if not isinstance(raw, dict):
        raise ValueError("MoveIt plan params JSON must encode an object")
    return normalize_plan_params(raw)


def latest_plan_params_from_state(state_dict):
    """Return the latest recorded plan params from a parsed rosbag state dict.

    Missing or malformed diagnostic data is not an oracle error; older bags and
    legacy runs simply return ``None`` and skip scaling consistency checks.
    """
    samples = state_dict.get(PLAN_PARAMS_TOPIC) or []
    if not samples:
        return None
    try:
        msg = samples[-1][1]
        return plan_params_from_json(getattr(msg, "data", ""))
    except (AttributeError, TypeError, ValueError, json.JSONDecodeError):
        return None
