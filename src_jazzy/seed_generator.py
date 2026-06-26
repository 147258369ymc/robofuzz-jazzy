"""
Seed generators for robotic fuzzing campaigns.

Each robot platform provides seed sequences that exercise boundary conditions
and physically meaningful state transitions. Seeds are designed to be
extensible: add a new function for each platform.
"""

from copy import deepcopy


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
    return generate_velocity_sequence_seeds(
        msg_class,
        seqlen=seqlen,
        max_linear=0.15,
        max_angular=0.8,
    )


def generate_sequence_seeds(platform, msg_class, seqlen=10):
    """
    Dispatch to platform-specific seed generator.
    Extensible: add new platforms here.
    """
    generators = {
        "tb3": generate_tb3_sequence_seeds,
        "tb4": generate_tb4_sequence_seeds,
        "turtlebot4": generate_tb4_sequence_seeds,
    }
    gen_func = generators.get(platform)
    if gen_func is None:
        return []
    return gen_func(msg_class, seqlen)
