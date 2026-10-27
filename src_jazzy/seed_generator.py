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
# The Jazzy diffdrive controller stops 0.5 s after each sparse command.  The
# external `ros2 topic pub --once` path takes longer than that between messages,
# so every sequence element contributes one isolated motion pulse.  Symmetric
# acceleration/deceleration has total velocity-time area `command * timeout`,
# and the controller's preserve_turning_radius setting keeps the pulse at the
# requested curvature while applying those limits.
SLAM_COMMAND_EFFECTIVE_DURATION_SEC = 0.5
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


def _slam_loop_plan(sequence_length, max_linear, max_angular):
    """Return a feasible continuous-circle plan, or ``None``.

    The external publisher spaces commands beyond the controller's 0.5-second
    timeout, so the loop is a series of equal constant-curvature motion pulses.
    A useful seed integrates those pulses to one full turn, travels at least
    one metre, and leaves two final commands for post-closure observation.
    """
    minimum_sequence_length = 10
    idle_steps = 2
    circle_steps = sequence_length - idle_steps
    move_speed = 0.55 * max_linear
    turn_speed_limit = 0.85 * max_angular
    if (
        sequence_length < minimum_sequence_length
        or move_speed <= 0.0
        or turn_speed_limit <= 0.0
    ):
        return None

    motion_time = circle_steps * SLAM_COMMAND_EFFECTIVE_DURATION_SEC
    turn_rate = 2.0 * math.pi / motion_time
    if turn_rate > turn_speed_limit or motion_time * move_speed < 1.0:
        return None

    return {
        "circle_steps": circle_steps,
        "idle_steps": idle_steps,
        "move_speed": move_speed,
        "turn_rate": turn_rate,
    }


def _apply_slam_arc(msg_list, max_linear, max_angular, direction):
    """Apply an alternating-curvature arc with the requested first turn."""
    half = max(1, len(msg_list) // 2)
    for index, msg in enumerate(msg_list):
        curvature = direction if index < half else -direction
        _set_velocity(
            msg,
            0.70 * max_linear,
            curvature * 0.65 * max_angular,
        )
    return msg_list


def _apply_slam_loop(msg_list, plan, direction):
    """Apply a previously validated continuous-circle loop plan."""

    for index, msg in enumerate(msg_list):
        if index < plan["circle_steps"]:
            _set_velocity(
                msg,
                plan["move_speed"],
                direction * plan["turn_rate"],
            )
        else:
            _set_velocity(msg, 0.0, 0.0)
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

    # Only advertise a loop-closure seed when the sequence can complete a
    # continuous full circle plus the post-closure observation dwell.  Shorter
    # sequences still receive the alternating-curvature seed below.
    loop_plan = _slam_loop_plan(seqlen, max_linear, max_angular)
    if loop_plan is not None:
        sequence = [msg_class() for _ in range(seqlen)]
        seeds.append(_apply_slam_loop(sequence, loop_plan, direction=1.0))

    # Alternating-curvature arc provides sustained translation and rotation
    # as a non-closing curved-motion alternative.
    half = max(1, seqlen // 2)
    sequence = []
    for index in range(seqlen):
        direction = 1.0 if index < half else -1.0
        sequence.append(_set_velocity(
            msg_class(),
            0.70 * max_linear,
            direction * 0.65 * max_angular,
        ))
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

    if strategy == "trajectory_arc":
        direction = random.choice([-1.0, 1.0])
        return _apply_slam_arc(
            msg_list, max_linear, max_angular, direction
        )

    if strategy == "loop_closure":
        direction = random.choice([-1.0, 1.0])
        loop_plan = _slam_loop_plan(
            len(msg_list), max_linear, max_angular
        )
        if loop_plan is None:
            return _apply_slam_arc(
                msg_list, max_linear, max_angular, direction
            )
        return _apply_slam_loop(msg_list, loop_plan, direction)

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


def generate_nav2_goal_seeds(map_ctx, start_xy=(0.0, 0.0), count=16,
                             min_dist=0.75, max_dist=2.5):
    """Sample reachable free-space goal poses (x, y, yaw) from an occupancy map.

    Deterministic: walks free cells on a stride so the same map yields the same
    seeds (reproducible campaigns). Each returned goal is BFS-reachable from
    start_xy, so nav2's ABORTED-on-reachable 'false failure' verdict stays sound.
    Returns a list of (x, y, yaw) world-coordinate tuples. yaw cycles a few
    headings so orientation is exercised too.

    Goals are constrained to ``min_dist <= |goal - start| <= max_dist`` (metres,
    straight-line). The max bound matters because the gz Harmonic full-sensor
    TB4 sim is CPU-bound and runs well below real time: a goal near the far
    corner of slam_arena (~6-7 m) does not finish within the action-timeout
    budget even while navigating correctly. Capping distance keeps each round
    completable so the arrival/collision oracles get a real verdict. Override
    max_dist=None to sample the whole free space (long campaigns with a large
    NAV2_ACTION_TIMEOUT_SEC). min_dist keeps goals off the spawn point so the
    robot actually has to move.
    """
    import math as _math
    import numpy as np
    grid = map_ctx.grid
    free = np.argwhere((grid >= 0) & (grid < 50))  # (row, col) free cells
    if len(free) == 0:
        return []
    yaws = [0.0, 1.5708, 3.14159, -1.5708]

    # First collect ALL free cells inside the [min_dist, max_dist] annulus that
    # are BFS-reachable, THEN sub-sample count of them evenly. The earlier
    # approach strode the whole-map free list by a fixed step and only kept
    # cells that happened to land in the annulus -- for a small annulus that
    # yielded far fewer than `count` seeds (observed: 4 of 16). Filtering first
    # then evenly sampling guarantees up to `count` diverse seeds regardless of
    # how small the annulus is relative to the whole map.
    candidates = []
    for row, col in free:
        x, y = map_ctx.cell_to_world(int(col), int(row))
        dist = _math.hypot(x - start_xy[0], y - start_xy[1])
        if dist < min_dist:
            continue
        if max_dist is not None and dist > max_dist:
            continue
        if not map_ctx.reachable(start_xy, (x, y)):
            continue
        # Reject goals inside nav2's costmap inflation layer. A goal within
        # inflation(0.45)+robot_radius(0.22)=0.67m of an obstacle is BFS-reachable
        # but NOT costmap-plannable -> nav2 legitimately ABORTs -> spurious
        # FALSE_FAILURE. Filtering at the source stops wall-hugging goals from
        # ever being sent (root-cause fix for 13/13 false positives in the
        # 1000-round campaign, all at x=+-6.48 map-edge goals, clearance 0.40-0.45m).
        if map_ctx.clearance(x, y, max_r=0.7) < 0.67:
            continue
        candidates.append((x, y))

    if not candidates:
        return []

    # Evenly sub-sample `count` candidates across the reachable annulus so the
    # seeds spread over the whole ring rather than clustering in one arc.
    step = max(1, len(candidates) // count)
    seeds = []
    for i, idx in enumerate(range(0, len(candidates), step)):
        if len(seeds) >= count:
            break
        x, y = candidates[idx]
        seeds.append((x, y, yaws[i % len(yaws)]))
    return seeds


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
