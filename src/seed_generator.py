"""
Seed generators for robotic fuzzing campaigns.

Each robot platform provides seed sequences that exercise boundary conditions
and physically meaningful state transitions. Seeds are designed to be
extensible: add a new function for each platform.
"""

from copy import deepcopy


def generate_tb3_sequence_seeds(msg_class, seqlen=10):
    """
    Generate step-sequence seeds for TurtleBot3 that trigger acceleration
    and kinematic consistency oracles.

    Returns a list of message sequences (each sequence is a list of Twist msgs).
    """
    MAX_LIN = 0.22   # Burger max linear velocity
    MAX_ANG = 2.84   # Burger max angular velocity

    seeds = []

    # 1. Step response: zero -> max forward (triggers linear acceleration)
    seq = []
    for i in range(seqlen):
        msg = msg_class()
        if i < seqlen // 2:
            msg.linear.x = 0.0
        else:
            msg.linear.x = MAX_LIN
        msg.angular.z = 0.0
        seq.append(msg)
    seeds.append(seq)

    # 2. Step response: zero -> max angular (triggers angular acceleration)
    seq = []
    for i in range(seqlen):
        msg = msg_class()
        msg.linear.x = 0.0
        if i < seqlen // 2:
            msg.angular.z = 0.0
        else:
            msg.angular.z = MAX_ANG
        seq.append(msg)
    seeds.append(seq)

    # PLACEHOLDER_SEEDS

    # 3. Alternating: max forward <-> max reverse (stress test)
    seq = []
    for i in range(seqlen):
        msg = msg_class()
        msg.linear.x = MAX_LIN if i % 2 == 0 else -MAX_LIN
        msg.angular.z = 0.0
        seq.append(msg)
    seeds.append(seq)

    # 4. Alternating: max left <-> max right turn
    seq = []
    for i in range(seqlen):
        msg = msg_class()
        msg.linear.x = 0.0
        msg.angular.z = MAX_ANG if i % 2 == 0 else -MAX_ANG
        seq.append(msg)
    seeds.append(seq)

    # 5. Combined step: zero -> max forward + max turn
    seq = []
    for i in range(seqlen):
        msg = msg_class()
        if i < seqlen // 2:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            msg.linear.x = MAX_LIN
            msg.angular.z = MAX_ANG
        seq.append(msg)
    seeds.append(seq)

    # 6. Ramp up: gradually increase velocity (smooth acceleration)
    seq = []
    for i in range(seqlen):
        msg = msg_class()
        msg.linear.x = MAX_LIN * (i / max(seqlen - 1, 1))
        msg.angular.z = 0.0
        seq.append(msg)
    seeds.append(seq)

    # 7. Emergency stop: max forward -> zero (deceleration)
    seq = []
    for i in range(seqlen):
        msg = msg_class()
        if i < seqlen // 2:
            msg.linear.x = MAX_LIN
        else:
            msg.linear.x = 0.0
        msg.angular.z = 0.0
        seq.append(msg)
    seeds.append(seq)

    return seeds


def generate_sequence_seeds(platform, msg_class, seqlen=10):
    """
    Dispatch to platform-specific seed generator.
    Extensible: add new platforms here.
    """
    generators = {
        "tb3": generate_tb3_sequence_seeds,
    }
    gen_func = generators.get(platform)
    if gen_func is None:
        return []
    return gen_func(msg_class, seqlen)
