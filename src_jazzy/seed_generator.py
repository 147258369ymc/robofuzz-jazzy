"""
Seed generators for robotic fuzzing campaigns.

Each robot platform provides seed sequences that exercise boundary conditions
and physically meaningful state transitions. Seeds are designed to be
extensible: add a new function for each platform.
"""

from copy import deepcopy
import math
import os
import random


TB4_SAFE_MAX_LINEAR = 0.15
TB4_SAFE_MAX_ANGULAR = 0.8
SLAM_DEFAULT_MAX_LINEAR = 0.35
SLAM_DEFAULT_MAX_ANGULAR = 1.5
TB4_PHYSICAL_MAX_LINEAR = 0.46
TB4_PHYSICAL_MAX_ANGULAR = 1.9
SLAM_ORACLE_MODE = "slam_toolbox_jazzy"


def _set_velocity(msg, lin_x, ang_z):
    target = msg.twist if hasattr(msg, "twist") else msg
    target.linear.x = lin_x
    target.angular.z = ang_z
    return msg


def _clamp(value, lower, upper):
    return min(max(value, lower), upper)


def clamp_velocity_message(msg, max_linear=0.15, max_angular=0.8):
    """Clamp a diff-drive velocity command after generic mutation.

    RoboFuzz's generic float mutator can inject values such as +/-5.0 into
    Twist/TwistStamped fields. TurtleBot4 phase-1 fuzzing should explore within
    a conservative drivable envelope, while deeper velocity-limit oracles can
    later widen this intentionally.
    """
    target = msg.twist if hasattr(msg, "twist") else msg
    target.linear.x = _clamp(target.linear.x, -max_linear, max_linear)
    target.linear.y = 0.0
    target.linear.z = 0.0
    target.angular.x = 0.0
    target.angular.y = 0.0
    target.angular.z = _clamp(target.angular.z, -max_angular, max_angular)
    return msg


def clamp_velocity_sequence(msg_list, max_linear=0.15, max_angular=0.8):
    for msg in msg_list:
        clamp_velocity_message(msg, max_linear, max_angular)
    return msg_list


def slam_velocity_limits():
    """Velocity clamp limits for the slam_toolbox target.

    Higher than the TurtleBot4 physical-safety clamp (0.15/0.8) because SLAM
    must be stressed with aggressive motion (fast moves, sharp turns) to
    provoke scan-match / loop-closure failures. Defaults 0.35/1.5; override
    via SLAM_MAX_LIN / SLAM_MAX_ANG to push toward the TB4 envelope
    (0.46/1.9) without code changes.
    """
    max_lin = _validated_velocity_limit(
        "SLAM_MAX_LIN",
        SLAM_DEFAULT_MAX_LINEAR,
        TB4_PHYSICAL_MAX_LINEAR,
    )
    max_ang = _validated_velocity_limit(
        "SLAM_MAX_ANG",
        SLAM_DEFAULT_MAX_ANGULAR,
        TB4_PHYSICAL_MAX_ANGULAR,
    )
    return max_lin, max_ang


def _validated_velocity_limit(env_name, default_value, physical_max):
    raw_value = os.environ.get(env_name, str(default_value))
    try:
        value = float(raw_value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"{env_name} must be finite, > 0, and <= {physical_max}; "
            f"got {raw_value!r}"
        ) from exc

    if not math.isfinite(value) or value <= 0 or value > physical_max:
        raise ValueError(
            f"{env_name} must be finite, > 0, and <= {physical_max}; "
            f"got {raw_value!r}"
        )
    return value


def velocity_limits_for_oracle(oracle_mode):
    """Select velocity limits without leaking SLAM settings into TB4."""
    if oracle_mode == SLAM_ORACLE_MODE:
        return slam_velocity_limits()
    return TB4_SAFE_MAX_LINEAR, TB4_SAFE_MAX_ANGULAR


def generate_velocity_sequence_seeds(
    msg_class,
    seqlen=10,
    max_linear=0.22,
    max_angular=2.84,
):
    """
    Generate step-sequence seeds for TurtleBot3 that trigger acceleration
    and kinematic consistency oracles.

    Returns a list of message sequences (each sequence is a list of Twist msgs).
    """
    MAX_LIN = max_linear
    MAX_ANG = max_angular

    seeds = []

    # 1. Step response: zero -> max forward (triggers linear acceleration)
    seq = []
    for i in range(seqlen):
        msg = msg_class()
        if i < seqlen // 2:
            lin_x = 0.0
        else:
            lin_x = MAX_LIN
        _set_velocity(msg, lin_x, 0.0)
        seq.append(msg)
    seeds.append(seq)

    # 2. Step response: zero -> max angular (triggers angular acceleration)
    seq = []
    for i in range(seqlen):
        msg = msg_class()
        if i < seqlen // 2:
            ang_z = 0.0
        else:
            ang_z = MAX_ANG
        _set_velocity(msg, 0.0, ang_z)
        seq.append(msg)
    seeds.append(seq)

    # PLACEHOLDER_SEEDS

    # 3. Alternating: max forward <-> max reverse (stress test)
    seq = []
    for i in range(seqlen):
        msg = msg_class()
        _set_velocity(msg, MAX_LIN if i % 2 == 0 else -MAX_LIN, 0.0)
        seq.append(msg)
    seeds.append(seq)

    # 4. Alternating: max left <-> max right turn
    seq = []
    for i in range(seqlen):
        msg = msg_class()
        _set_velocity(msg, 0.0, MAX_ANG if i % 2 == 0 else -MAX_ANG)
        seq.append(msg)
    seeds.append(seq)

    # 5. Combined step: zero -> max forward + max turn
    seq = []
    for i in range(seqlen):
        msg = msg_class()
        if i < seqlen // 2:
            lin_x = 0.0
            ang_z = 0.0
        else:
            lin_x = MAX_LIN
            ang_z = MAX_ANG
        _set_velocity(msg, lin_x, ang_z)
        seq.append(msg)
    seeds.append(seq)

    # 6. Ramp up: gradually increase velocity (smooth acceleration)
    seq = []
    for i in range(seqlen):
        msg = msg_class()
        _set_velocity(msg, MAX_LIN * (i / max(seqlen - 1, 1)), 0.0)
        seq.append(msg)
    seeds.append(seq)

    # 7. Emergency stop: max forward -> zero (deceleration)
    seq = []
    for i in range(seqlen):
        msg = msg_class()
        if i < seqlen // 2:
            lin_x = MAX_LIN
        else:
            lin_x = 0.0
        _set_velocity(msg, lin_x, 0.0)
        seq.append(msg)
    seeds.append(seq)

    return seeds


def generate_tb3_sequence_seeds(msg_class, seqlen=10):
    return generate_velocity_sequence_seeds(
        msg_class,
        seqlen=seqlen,
        max_linear=0.22,
        max_angular=2.84,
    )


def generate_tb4_sequence_seeds(msg_class, seqlen=10):
    # Conservative Phase-1 smoke envelope for TurtleBot4 Standard / Create3.
    # These are safe seed bounds (NOT oracle thresholds): deep velocity /
    # hazard thresholds are derived later from runtime parameters,
    # TurtleBot4/Create3 docs, or measured Jazzy baselines.
    seeds = generate_velocity_sequence_seeds(
        msg_class,
        seqlen=seqlen,
        max_linear=0.15,
        max_angular=0.8,
    )

    # 8. Near-zero sign-threshold probes around the smoke oracle's direction
    # agreement threshold. These help separate deadband/rounding behavior from
    # true cmd/odom sign conflicts.
    seq = []
    near_values = [0.0, 0.03, 0.05, -0.03, -0.05]
    for i in range(seqlen):
        msg = msg_class()
        _set_velocity(msg, near_values[i % len(near_values)], 0.0)
        seq.append(msg)
    seeds.append(seq)

    # 9. Coupled arc: linear and angular commands together. This stresses
    # wheel/odom consistency more effectively than pure translation or spin.
    seq = []
    for i in range(seqlen):
        msg = msg_class()
        phase = i / max(seqlen - 1, 1)
        lin_x = 0.05 + 0.10 * phase
        ang_z = 0.25 + 0.55 * phase
        if i >= seqlen // 2:
            ang_z = -ang_z
        _set_velocity(msg, lin_x, ang_z)
        seq.append(msg)
    seeds.append(seq)

    # 10. Timeout-tail probe: leave the last command nonzero so that after the
    # publisher stops, the cmd_vel timeout oracle can observe whether motion
    # decays rather than being kept alive by a final zero command.
    seq = []
    for i in range(seqlen):
        msg = msg_class()
        lin_x = 0.0 if i < seqlen // 3 else 0.12
        _set_velocity(msg, lin_x, 0.0)
        seq.append(msg)
    seeds.append(seq)

    # 11. Angular timeout-tail probe.
    seq = []
    for i in range(seqlen):
        msg = msg_class()
        ang_z = 0.0 if i < seqlen // 3 else 0.65
        _set_velocity(msg, 0.0, ang_z)
        seq.append(msg)
    seeds.append(seq)

    # 12. Curved reversal: both velocity components flip sign. This targets
    # acceleration-ratio and wheel/odom direction consistency feedback.
    seq = []
    for i in range(seqlen):
        msg = msg_class()
        if i < seqlen // 2:
            _set_velocity(msg, 0.12, 0.65)
        else:
            _set_velocity(msg, -0.12, -0.65)
        seq.append(msg)
    seeds.append(seq)

    return seeds


def generate_slam_sequence_seeds(msg_class, seqlen=10):
    """Generate sustained SLAM motion seeds inside the validated TB4 limits.

    Unlike the ordinary TurtleBot4 smoke seeds, these sequences intentionally
    leave enough translational or angular headroom to cross slam_toolbox's
    scan-acceptance motion thresholds even when individual velocity commands
    are shortened by the controller timeout.
    """
    max_linear, max_angular = slam_velocity_limits()
    seeds = []

    def constant_sequence(lin_x, ang_z):
        sequence = []
        for _ in range(seqlen):
            sequence.append(_set_velocity(msg_class(), lin_x, ang_z))
        return sequence

    seeds.append(constant_sequence(max_linear, 0.0))
    seeds.append(constant_sequence(-max_linear, 0.0))
    seeds.append(constant_sequence(0.0, max_angular))
    seeds.append(constant_sequence(0.0, -max_angular))
    seeds.append(constant_sequence(0.7 * max_linear, 0.8 * max_angular))
    seeds.append(constant_sequence(0.7 * max_linear, -0.8 * max_angular))

    half = seqlen // 2
    sequence = []
    for index in range(seqlen):
        lin_x = max_linear if index < half else -max_linear
        sequence.append(_set_velocity(msg_class(), lin_x, 0.0))
    seeds.append(sequence)

    sequence = []
    for index in range(seqlen):
        ang_z = max_angular if index < half else -max_angular
        sequence.append(_set_velocity(msg_class(), 0.0, ang_z))
    seeds.append(sequence)

    start = seqlen // 3
    sequence = []
    for index in range(seqlen):
        lin_x = 0.0 if index < start else max_linear
        sequence.append(_set_velocity(msg_class(), lin_x, 0.0))
    seeds.append(sequence)

    sequence = []
    for index in range(seqlen):
        ang_z = 0.0 if index < start else max_angular
        sequence.append(_set_velocity(msg_class(), 0.0, ang_z))
    seeds.append(sequence)

    return seeds


def apply_slam_motion_strategy(
    msg_list,
    strategy,
    max_linear,
    max_angular,
    recent_feedback=None,
):
    """Apply a coherent SLAM motion shape to an existing message sequence."""
    if not msg_list:
        return msg_list

    angular_feedback = recent_feedback and (
        "yaw" in recent_feedback or "angular" in recent_feedback
    )
    preferred_axis = "angular" if angular_feedback else "linear"

    if strategy == "single_block":
        start = len(msg_list) // 3
        sign = random.choice([-1.0, 1.0])
        for index, msg in enumerate(msg_list):
            if index < start:
                _set_velocity(msg, 0.0, 0.0)
            elif preferred_axis == "angular":
                _set_velocity(msg, 0.0, sign * 0.85 * max_angular)
            else:
                _set_velocity(msg, sign * 0.85 * max_linear, 0.0)
        return msg_list

    if strategy == "reversal":
        half = max(1, len(msg_list) // 2)
        initial_sign = random.choice([-1.0, 1.0])
        for index, msg in enumerate(msg_list):
            sign = initial_sign if index < half else -initial_sign
            if preferred_axis == "angular":
                _set_velocity(
                    msg,
                    sign * 0.35 * max_linear,
                    sign * 0.85 * max_angular,
                )
            else:
                _set_velocity(
                    msg,
                    sign * 0.85 * max_linear,
                    sign * 0.40 * max_angular,
                )
        return msg_list

    if strategy == "boundary_push":
        sign = random.choice([-1.0, 1.0])
        for index, msg in enumerate(msg_list):
            phase = index / max(len(msg_list) - 1, 1)
            magnitude = 0.65 + 0.35 * phase
            if preferred_axis == "angular":
                _set_velocity(
                    msg,
                    0.35 * max_linear,
                    sign * magnitude * max_angular,
                )
            else:
                _set_velocity(
                    msg,
                    sign * magnitude * max_linear,
                    sign * 0.50 * max_angular,
                )
        return msg_list

    if strategy != "random":
        raise ValueError(f"unknown SLAM motion strategy: {strategy}")

    block_len = max(3, len(msg_list) // 5)
    index = 0
    while index < len(msg_list):
        end = min(len(msg_list), index + block_len)
        if 0 < len(msg_list) - end < 3:
            end = len(msg_list)
        linear = (
            random.choice([-1.0, 1.0])
            * random.uniform(0.55, 0.95)
            * max_linear
        )
        angular = (
            random.choice([-1.0, 1.0])
            * random.uniform(0.55, 0.95)
            * max_angular
        )
        for msg in msg_list[index:end]:
            _set_velocity(msg, linear, angular)
        index = end

    return msg_list


def generate_sequence_seeds(platform, msg_class, seqlen=10):
    """
    Dispatch to platform-specific seed generator.
    Extensible: add new platforms here.
    """
    generators = {
        "tb3": generate_tb3_sequence_seeds,
        "tb4": generate_tb4_sequence_seeds,
        "turtlebot4": generate_tb4_sequence_seeds,
        "slam": generate_slam_sequence_seeds,
    }
    gen_func = generators.get(platform)
    if gen_func is None:
        return []
    return gen_func(msg_class, seqlen)
