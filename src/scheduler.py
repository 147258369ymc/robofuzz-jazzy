import os
import time
import random
from functools import reduce
from copy import deepcopy
from enum import Enum, auto

import numpy as np

import constants as c
import mutator
import harness
import ros_utils
from ros2_fuzzer import ros_commons
try:
    import px4_utils
    from px4_prep.blacklist import blacklist as param_blacklist
    from px4_prep.blacklist import tested as param_tested
except ImportError:
    px4_utils = None
    param_blacklist = set()
    param_tested = set()
from mutation_profile import (
    MutationProfile, STRATEGY_MULTI_AXIS, STRATEGY_FLIP, STRATEGY_SINGLE_BLOCK,
    STRATEGY_RANDOM, STRATEGY_BOUNDARY_PUSH, STRATEGY_REVERSAL,
    STRATEGY_TRAJECTORY_ARC, STRATEGY_SINGLE_EXTREME
)


class Campaign(Enum):
    RND_SINGLE = auto()
    RND_SEQUENCE = auto()
    RND_REPEATED = auto()
    INTERCEPTION = auto()
    SROS_AUTH = auto()
    DDS_CROSSCHK = auto()
    IDL_CHECK = auto()


class Scheduler:
    def __init__(self, fuzzer, campaign, target, fast_float_determ=False):
        self.fuzzer = fuzzer
        self.campaign = campaign
        self.topic_name = target[0]
        self.msg_type_class = target[1]
        self.subscriber_node = target[2]
        self.default_msg = self.msg_type_class()
        self.fast_float_determ = fast_float_determ
        # Queue consumption interval: check queue every N iterations
        self.queue_check_interval = 50
        self.iter_since_queue_check = 0

        msg_type_dict = ros_commons.map_ros_types(self.msg_type_class)
        self.msg_field_list = list(
            ros_utils.flatten_nested_dict(msg_type_dict)
        )

    def filter_field_list(self, whitelist, blacklist):
        if whitelist is not None:
            self.msg_field_list = whitelist
        elif blacklist is not None:
            for field in blacklist:
                self.msg_field_list.remove(field)

    def init_schedule(self):
        if (
            self.campaign == Campaign.RND_SINGLE
            or self.campaign == Campaign.RND_REPEATED
        ):
            self.num_fields = len(self.msg_field_list)
            self.cur_fm_field = 0
            self.fm_field_stages = [0] * self.num_fields
            self.fm_determ_stages = [0] * self.num_fields

            # determine havoc stages
            self.fm_rand_stages = []
            for i in range(self.num_fields):
                self.fm_rand_stages.append(random.randint(1, 2))  # XXX: WHY?

            self.fm_odata = []

            self.num_msg_mutation = 0

            # Rounds
            # A. Field-by-field mutation
            #   0: pop from queue / generate if empty
            #   1: deterministic stages
            #   2: random mutation (havoc)
            # B. All fields mutation
            #   0: select random field & mut_op

            self.bit_pos = 0
            self.arith_val = -35
            self.interesting_idx = 0

            # pre-select fields to mutate for all-fields mutation
            self.fields_to_mutate = random.sample(
                self.msg_field_list, random.randint(1, self.num_fields)
            )

            self.cycle_cnt = 0
            self.is_new_cycle = True
            self.round_cnt = 0
            self.from_queue = False
            self.exec_cnt = 0

        elif self.campaign == Campaign.RND_SEQUENCE:
            seqlen = self.fuzzer.config.seqlen
            self.num_msgs = seqlen
            # self.num_msgs = random.randint(2, seqlen)
            self.num_fields = len(self.msg_field_list)
            self.cycle_cnt = 0
            self.is_new_cycle = True
            self.round_cnt = 0
            self.from_queue = False

        elif self.campaign == Campaign.IDL_CHECK:
            self.cycle_cnt = 0
            self.is_new_cycle = True
            self.round_cnt = 0
            self.from_queue = False
            self.init_builtin_type_name = None
            self.full_type_name = None
            self.current_type = None
            self.default_val = None

    def random_stage(self):
        print("STAGE: RANDOM")

        error = None
        expecting = False
        if "Array" in self.full_type_name:
            try:
                idx = random.randint(0, len(self.cur_msg.data) - 1)
                self.cur_msg.data[idx] = mutator.get_rand_val(
                        self.full_type_name)
            except Exception as e:
                error = str(e)

        else:
            try:
                self.cur_msg.data = mutator.get_rand_val(
                        self.full_type_name)
            except Exception as e:
                error = str(e)

        return (error, expecting)

    def mutate_moveit_joint(self, config):
        frame = str(time.time())

        meta_file = os.path.join(config.meta_dir, "meta-{}".format(frame))
        with open(meta_file, "w") as fp:
            fp.write(
                self.subscriber_node
                + "\t"
                + self.topic_name
                + "\t"
                + str(self.msg_type_class)
                + "\t"
                + str(self.cycle_cnt)
                + "\t"
                + str(self.round_cnt)
            )

        print(
            "\n\x1b[92mCYCLE:",
            self.cycle_cnt,
            "ROUND:",
            self.round_cnt,
            "\x1b[0m",
            frame,
        )

        if self.is_new_cycle:
            try:
                # list contains seven joint constraints
                self.msg_list = self.fuzzer.queue.popleft()
            except IndexError:
                self.msg_list = harness.get_init_joint_constraints()

            self.is_new_cycle = False

        else:
            joint_idx = random.randint(0, len(self.msg_list) - 1)
            field_name = "position" # only fuzz goal position for the time being
            dtype = np.dtype("float64")

            self.msg_list[joint_idx].position = mutator.gen_rand_data(
                dtype, False)

        return (self.msg_list, frame)

    def mutate_moveit_goal(self, config):
        frame = str(time.time())

        meta_file = os.path.join(config.meta_dir, "meta-{}".format(frame))
        with open(meta_file, "w") as fp:
            fp.write(
                self.subscriber_node
                + "\t"
                + self.topic_name
                + "\t"
                + str(self.msg_type_class)
                + "\t"
                + str(self.cycle_cnt)
                + "\t"
                + str(self.round_cnt)
            )

        print(
            "\n\x1b[92mCYCLE:",
            self.cycle_cnt,
            "ROUND:",
            self.round_cnt,
            "\x1b[0m",
            frame,
        )

        if self.is_new_cycle:
            try:
                self.msg = self.fuzzer.queue.popleft()
            except IndexError:
                self.msg = harness.get_init_moveit_pose()

            self.is_new_cycle = False

        else:
            # It really doesn't make sense to use the entire double range
            # when the manipulator workspace is limited to a small region:
            # x: -855 ~ 855, y: -855 ~ 855, z: -360 ~ 1190
            #
            # We don't have to stick to the exact constraints that precisely
            # determine the workspace boundary - let the fuzzer generate the
            # end effector goals that sometimes exceed the workspace boundary
            # with a small margin.
            #
            # In the meantime, special values (INF, NaN) should be tested.

            # dtype = np.dtype("float64")
            # self.msg.position.x = mutator.gen_rand_data(dtype, False)
            # self.msg.position.y = mutator.gen_rand_data(dtype, False)
            # self.msg.position.z = mutator.gen_rand_data(dtype, False)
            # self.msg.orientation.w = mutator.gen_rand_data(dtype, False)

            self.msg = deepcopy(self.last_msg)
            prob_special = 5
            sel = np.random.randint(3)

            if sel == 0:
                if np.random.randint(0, 100) >= prob_special:
                    x_mm = mutator.gen_float_in_range(-900, 900, 4)
                else:
                    x_mm = mutator.gen_special_floats()

                self.msg.position.x = x_mm / 1000.0 # (unit: m)

            elif sel == 1:
                if np.random.randint(0, 100) >= prob_special:
                    y_mm = mutator.gen_float_in_range(-900, 900, 4)
                else:
                    y_mm = mutator.gen_special_floats()

                self.msg.position.y = y_mm / 1000.0

            elif sel == 2:
                if np.random.randint(0, 100) >= prob_special:
                    z_mm = mutator.gen_float_in_range(-400, 1300, 4)
                else:
                    z_mm = mutator.gen_special_floats()

                self.msg.position.z = z_mm / 1000.0

        self.round_cnt += 1

        # print(self.msg)
        self.last_msg = deepcopy(self.msg)
        return (self.msg, frame)

    # ===================================================================
    # MoveIt2 Sequence Mutation (multi-goal, dual-phase, feedback-adaptive)
    # ===================================================================

    # Cycle length & phase constants
    CYCLE_MIN = 20
    CYCLE_MAX = 30
    EXPLOIT_PHASE_END = 12
    EXTEND_WINDOW = 5

    def mutate_sequence_moveit(self, config, fbk_list=None):
        """Multi-goal, dual-phase mutation for MoveIt2.

        Phase 1 (rounds 0-12): exploitation via seed neighborhood mutation.
        Phase 2 (rounds 13+): strategy-driven exploration using seed centroid.
        """
        if not hasattr(self, '_moveit_profile'):
            self._moveit_profile = MutationProfile.moveit_panda()
        profile = self._moveit_profile

        frame = str(time.time())

        meta_file = os.path.join(config.meta_dir, "meta-{}".format(frame))
        with open(meta_file, "w") as fp:
            fp.write(
                self.subscriber_node + "\t"
                + self.topic_name + "\t"
                + str(self.msg_type_class) + "\t"
                + str(self.cycle_cnt) + "\t"
                + str(self.round_cnt)
            )

        print(
            "\n\x1b[92mCYCLE:", self.cycle_cnt,
            "ROUND:", self.round_cnt, "\x1b[0m", frame,
        )
        print("QUEUE LEN:", len(self.fuzzer.queue))

        if self.is_new_cycle:
            self._moveit_init_cycle(profile)
        elif self.round_cnt <= self.EXPLOIT_PHASE_END:
            self._moveit_exploit_round(profile)
        else:
            self._moveit_explore_round(profile, fbk_list)

        self.round_cnt += 1
        return (self.msg_list, frame)

    def _moveit_init_cycle(self, profile):
        """Initialize goal_list for a new MoveIt cycle, save seed_base."""
        if len(self.fuzzer.queue) > 0:
            try:
                queued = self.fuzzer.queue.popleft()
            except IndexError:
                queued = None
            if queued is not None:
                if isinstance(queued, list):
                    self.msg_list = queued
                else:
                    self.msg_list = [deepcopy(queued)]
                self.from_queue = True
                self.num_msgs = len(self.msg_list)
                print("seed from queue")
                self._save_seed_base()
                self.is_new_cycle = False
                return

        # Queue empty: generate fresh random goals
        min_goals, max_goals = profile.block_len_range
        num_goals = random.randint(min_goals, max_goals)
        self.msg_list = []
        for _ in range(num_goals):
            msg = harness.get_init_moveit_pose()
            msg.position.x = profile.get_range("x").sample()
            msg.position.y = profile.get_range("y").sample()
            msg.position.z = profile.get_range("z").sample()
            self.msg_list.append(msg)
        self.num_msgs = num_goals
        self.from_queue = False
        self._save_seed_base()
        self.is_new_cycle = False
        print(f"generate fresh {num_goals} goals")

    def _save_seed_base(self):
        """Save immutable seed reference and compute centroid."""
        self._seed_base = deepcopy(self.msg_list)
        # Compute centroid for Phase 2 exploration
        n = len(self._seed_base)
        cx = sum(m.position.x for m in self._seed_base) / n
        cy = sum(m.position.y for m in self._seed_base) / n
        cz = sum(m.position.z for m in self._seed_base) / n
        self._seed_center = (cx, cy, cz)

    def _moveit_exploit_round(self, profile):
        """Phase 1: neighborhood mutation around seed_base."""
        import math as _math
        # Weighted mutation operations
        ops = ['perturb', 'swap', 'negate', 'replace', 'resize', 'orient', 'oscillate']
        weights = [20, 12, 12, 12, 10, 18, 16]
        op = random.choices(ops, weights=weights, k=1)[0]

        new_list = deepcopy(self.msg_list)
        min_goals, max_goals = profile.block_len_range

        if op == 'perturb':
            # Perturb 1-2 goals with Gaussian noise
            n_perturb = min(random.randint(1, 2), len(new_list))
            idxs = random.sample(range(len(new_list)), n_perturb)
            for idx in idxs:
                new_list[idx].position.x += random.gauss(0, 0.05)
                new_list[idx].position.y += random.gauss(0, 0.05)
                new_list[idx].position.z += random.gauss(0, 0.03)
                new_list[idx].position.x = profile.get_range("x").clamp(
                    new_list[idx].position.x)
                new_list[idx].position.y = profile.get_range("y").clamp(
                    new_list[idx].position.y)
                new_list[idx].position.z = profile.get_range("z").clamp(
                    new_list[idx].position.z)
        elif op == 'swap' and len(new_list) >= 2:
            i, j = random.sample(range(len(new_list)), 2)
            new_list[i], new_list[j] = new_list[j], new_list[i]
        elif op == 'negate':
            idx = random.randint(0, len(new_list) - 1)
            axis = random.choice(['x', 'y'])
            if axis == 'x':
                new_list[idx].position.x = -new_list[idx].position.x
            else:
                new_list[idx].position.y = -new_list[idx].position.y
        elif op == 'replace':
            idx = random.randint(0, len(new_list) - 1)
            msg = harness.get_init_moveit_pose()
            msg.position.x = profile.get_range("x").sample()
            msg.position.y = profile.get_range("y").sample()
            msg.position.z = profile.get_range("z").sample()
            new_list[idx] = msg
        elif op == 'resize':
            if len(new_list) < max_goals and random.random() < 0.5:
                # Insert a new goal at random position
                msg = harness.get_init_moveit_pose()
                msg.position.x = profile.get_range("x").sample()
                msg.position.y = profile.get_range("y").sample()
                msg.position.z = profile.get_range("z").sample()
                pos = random.randint(0, len(new_list))
                new_list.insert(pos, msg)
            elif len(new_list) > min_goals:
                idx = random.randint(0, len(new_list) - 1)
                new_list.pop(idx)
        elif op == 'orient':
            # Perturb orientation with small quaternion rotation
            idx = random.randint(0, len(new_list) - 1)
            # Generate small rotation quaternion
            angle = random.gauss(0, 0.3)  # ~17 degrees std
            axis = random.choice(['x', 'y', 'z'])
            half = angle / 2.0
            sin_h = _math.sin(half)
            cos_h = _math.cos(half)
            if axis == 'x':
                new_list[idx].orientation.x = sin_h
                new_list[idx].orientation.w = cos_h
            elif axis == 'y':
                new_list[idx].orientation.y = sin_h
                new_list[idx].orientation.w = cos_h
            else:
                new_list[idx].orientation.z = sin_h
                new_list[idx].orientation.w = cos_h
        elif op == 'oscillate' and len(new_list) >= 2:
            # Create A→B→A pattern (rapid back-and-forth)
            idx = random.randint(0, len(new_list) - 2)
            goal_a = deepcopy(new_list[idx])
            goal_b = deepcopy(new_list[idx + 1])
            # Insert A after B to create A→B→A
            if len(new_list) < max_goals:
                new_list.insert(idx + 2, deepcopy(goal_a))

        self.msg_list = new_list
        self.num_msgs = len(self.msg_list)
        print(f"[moveit] phase1 exploit op={op}")

    def _moveit_explore_round(self, profile, fbk_list):
        """Phase 2: strategy-driven exploration anchored to seed centroid."""
        center = getattr(self, '_seed_center', (0.3, 0.0, 0.5))

        recent_feedback = None
        if fbk_list:
            for fbk in fbk_list:
                if fbk.value is not None and fbk.interesting_value is not None:
                    if fbk.value >= fbk.interesting_value:
                        recent_feedback = fbk.name
                        break

        # Stagnation detection
        if not hasattr(self, '_no_interesting_rounds'):
            self._no_interesting_rounds = 0
        self._no_interesting_rounds += 1

        if self._no_interesting_rounds >= 10:
            strategy = STRATEGY_RANDOM
        else:
            strategy = profile.select_strategy(recent_feedback)

        min_goals, max_goals = profile.block_len_range
        num_goals = random.randint(min_goals, max_goals)

        print(f"[moveit] phase2 explore strategy={strategy} "
              f"center=({center[0]:.2f},{center[1]:.2f},{center[2]:.2f})")

        if strategy == STRATEGY_BOUNDARY_PUSH:
            self.msg_list = self._moveit_boundary_push(
                profile, num_goals, center)
        elif strategy == STRATEGY_REVERSAL:
            self.msg_list = self._moveit_reversal(
                profile, num_goals, center)
        elif strategy == STRATEGY_TRAJECTORY_ARC:
            self.msg_list = self._moveit_trajectory_arc(
                profile, num_goals, center)
        elif strategy == STRATEGY_SINGLE_EXTREME:
            self.msg_list = self._moveit_single_extreme(profile, num_goals)
        else:  # STRATEGY_RANDOM
            self.msg_list = self._moveit_random_fresh(profile, num_goals)

        self.num_msgs = len(self.msg_list)

    def _moveit_boundary_push(self, profile, num_goals, center=None):
        """Generate goals near the workspace boundary, anchored to center."""
        import math as _math
        cx, cy, cz = center if center else (0.0, 0.0, 0.5)
        goals = []
        for _ in range(num_goals):
            # Sample in spherical coords: radius near boundary
            r = 0.855 + random.uniform(-0.1, 0.1)
            theta = random.uniform(0, 2 * _math.pi)  # azimuth
            phi = random.uniform(0.2, _math.pi - 0.2)  # polar (avoid poles)
            x = cx + r * _math.sin(phi) * _math.cos(theta)
            y = cy + r * _math.sin(phi) * _math.sin(theta)
            z = cz + r * _math.cos(phi) * 0.5  # dampened z offset
            # Clamp to domain
            x = profile.get_range("x").clamp(x)
            y = profile.get_range("y").clamp(y)
            z = profile.get_range("z").clamp(z)
            msg = harness.get_init_moveit_pose()
            msg.position.x = x
            msg.position.y = y
            msg.position.z = z
            if random.random() < 0.2:
                self._apply_random_orientation(msg)
            goals.append(msg)
        return goals

    def _moveit_reversal(self, profile, num_goals, center=None):
        """Generate goals that reverse direction mid-sequence."""
        cx, cy, cz = center if center else (0.0, 0.0, 0.5)
        goals = []
        # First half: positive offset from center, second: negative
        half = num_goals // 2
        offset_x = random.uniform(0.2, 0.5)
        offset_y = random.uniform(0.2, 0.5)
        offset_z = random.uniform(0.1, 0.4)
        for i in range(num_goals):
            if i < half:
                x = cx + offset_x + random.uniform(-0.05, 0.05)
                y = cy + offset_y + random.uniform(-0.05, 0.05)
                z = cz + offset_z + random.uniform(-0.05, 0.05)
            else:
                x = cx - offset_x + random.uniform(-0.05, 0.05)
                y = cy - offset_y + random.uniform(-0.05, 0.05)
                z = cz - offset_z + random.uniform(-0.05, 0.05)
            x = profile.get_range("x").clamp(x)
            y = profile.get_range("y").clamp(y)
            z = profile.get_range("z").clamp(z)
            msg = harness.get_init_moveit_pose()
            msg.position.x = x
            msg.position.y = y
            msg.position.z = z
            goals.append(msg)
        return goals

    def _moveit_trajectory_arc(self, profile, num_goals, center=None):
        """Generate goals along an arc anchored to seed centroid."""
        import math as _math
        cx, cy, cz = center if center else (0.0, 0.0, 0.5)
        goals = []
        r = random.uniform(0.3, 0.6)
        theta_start = random.uniform(0, 2 * _math.pi)
        arc_span = random.uniform(_math.pi * 0.5, _math.pi * 1.5)
        for i in range(num_goals):
            t = theta_start + arc_span * (i / max(1, num_goals - 1))
            x = cx + r * _math.cos(t)
            y = cy + r * _math.sin(t)
            z = cz + 0.1 * _math.sin(t * 2)
            x = profile.get_range("x").clamp(x)
            y = profile.get_range("y").clamp(y)
            z = profile.get_range("z").clamp(z)
            msg = harness.get_init_moveit_pose()
            msg.position.x = x
            msg.position.y = y
            msg.position.z = z
            goals.append(msg)
        return goals

    def _moveit_single_extreme(self, profile, num_goals):
        """Sweep one axis from min to max while holding others constant."""
        goals = []
        # Pick the axis to sweep
        axis = random.choice(["x", "y", "z"])
        fr = profile.get_range(axis)
        # Fixed values for other axes
        fixed = {}
        for a in ["x", "y", "z"]:
            if a != axis:
                fixed[a] = profile.get_range(a).sample()

        for i in range(num_goals):
            t = i / max(1, num_goals - 1)
            sweep_val = fr.low + t * (fr.high - fr.low)
            msg = harness.get_init_moveit_pose()
            msg.position.x = sweep_val if axis == "x" else fixed["x"]
            msg.position.y = sweep_val if axis == "y" else fixed["y"]
            msg.position.z = sweep_val if axis == "z" else fixed["z"]
            goals.append(msg)
        return goals

    def _moveit_random_fresh(self, profile, num_goals):
        """Generate completely random goals. Anti-stagnation fallback."""
        import math as _math
        goals = []
        prob_special = 5
        for _ in range(num_goals):
            msg = harness.get_init_moveit_pose()
            if np.random.randint(0, 100) < prob_special:
                msg.position.x = mutator.gen_special_floats() / 1000.0
            else:
                msg.position.x = profile.get_range("x").sample()
            if np.random.randint(0, 100) < prob_special:
                msg.position.y = mutator.gen_special_floats() / 1000.0
            else:
                msg.position.y = profile.get_range("y").sample()
            if np.random.randint(0, 100) < prob_special:
                msg.position.z = mutator.gen_special_floats() / 1000.0
            else:
                msg.position.z = profile.get_range("z").sample()
            # 30% chance of non-identity orientation
            if random.random() < 0.3:
                self._apply_random_orientation(msg)
            goals.append(msg)
        return goals

    def _apply_random_orientation(self, msg):
        """Apply a random orientation to a Pose goal."""
        import math as _math
        # Random rotation: angle up to 90 degrees around random axis
        angle = random.uniform(-_math.pi / 2, _math.pi / 2)
        # Random axis (unit vector)
        ax = random.gauss(0, 1)
        ay = random.gauss(0, 1)
        az = random.gauss(0, 1)
        norm = _math.sqrt(ax*ax + ay*ay + az*az)
        if norm < 0.001:
            return
        ax, ay, az = ax/norm, ay/norm, az/norm
        half = angle / 2.0
        sin_h = _math.sin(half)
        msg.orientation.x = ax * sin_h
        msg.orientation.y = ay * sin_h
        msg.orientation.z = az * sin_h
        msg.orientation.w = _math.cos(half)

    def mutate_px4_param(self, config):
        frame = str(time.time())

        meta_file = os.path.join(config.meta_dir, "meta-{}".format(frame))
        with open(meta_file, "w") as fp:
            fp.write(
                self.subscriber_node
                + "\t"
                + self.topic_name
                + "\t"
                + str(self.msg_type_class)
                + "\t"
                + str(self.cycle_cnt)
                + "\t"
                + str(self.round_cnt)
            )

        print(
            "\n\x1b[92mCYCLE:",
            self.cycle_cnt,
            "ROUND:",
            self.round_cnt,
            "\x1b[0m",
            frame,
        )

        if self.is_new_cycle:
            try:
                self.msg = self.fuzzer.queue.popleft()
            except IndexError:
                self.msg = px4_utils.get_init_parameter_msg()

            # Test one parameter per cycle

            while True:
                # randomly select the parameter with an open range
                sel = np.random.randint(len(px4_utils.param_dict))

                param_name = list(px4_utils.param_dict.keys())[sel]
                if param_name in param_blacklist:
                    continue
                elif param_name in param_tested:
                    continue

                param_meta = px4_utils.param_dict[param_name]
                range_min = param_meta["min"]
                range_max = param_meta["max"]

                if range_min is None or range_max is None:
                    break

            self.msg.param_name = param_name
            self.msg.param_type = param_meta["type"]
            self.msg.value = param_meta["default"]
            self.msg.min = param_meta["min"]
            self.msg.max = param_meta["max"]

            with open("params.log", "a") as f:
                f.write(param_name + "\n")

            print(f"[scheduler] {param_name} ({range_min} ~ {range_max})")

            self.is_new_cycle = False

        else:
            # TODO
            # probabilistically generate out-of-valid-range data -> (X)
            # https://github.com/PX4/PX4-Autopilot/issues/16122#issuecomment-723567775
            #
            # Instead, focus on the parameters if the min or max is not
            # specified on the documentation.

            self.msg = deepcopy(self.last_msg)

            if self.msg.param_type == "INT32":
                if self.msg.min is not None:
                    low = self.msg.min
                else:
                    low = -1000

                if self.msg.max is not None:
                    high = self.msg.max
                else:
                    high = 1000

                self.msg.value = mutator.gen_int_in_range(low, high)

            elif self.msg.param_type == "FLOAT":
                if self.msg.min is not None:
                    low = self.msg.min
                else:
                    low = -1000

                if self.msg.max is not None:
                    high = self.msg.max
                else:
                    high = 1000

                self.msg.value = mutator.gen_float_in_range(low, high, 2)

        self.round_cnt += 1

        print(self.msg)
        self.last_msg = deepcopy(self.msg)
        return (self.msg, frame)

    def mutate_typemsg(self, config):
        frame = str(time.time())

        print(
            "\n\x1b[92mCYCLE:",
            self.cycle_cnt,
            "ROUND:",
            self.round_cnt,
            "\x1b[0m",
            frame,
        )

        error = None
        expecting = None

        if self.round_cnt == 0:
            print("STAGE: GEN")
            assert (self.is_new_cycle == True)
            (ros_type, ext, num_elem) = mutator.mutate_type()
            self.is_new_cycle = False
            self.init_builtin_type_name = ros_type.name

            self.default_val = mutator.get_default_val(ros_type.name)

            type_name = ros_type.name.lower().capitalize()
            if type_name.startswith("Uint") or type_name.startswith("Wstr"):
                type_name = type_name[0] + type_name[1:].capitalize()

            if ext == c.TypeExtension.BUILTIN:
                type_name = type_name
            elif ext == c.TypeExtension.FARRAY:
                type_name += "FixedArray"
            elif ext == c.TypeExtension.BARRAY:
                type_name += "BoundedDynArray"
            elif ext == c.TypeExtension.UBARRAY:
                type_name += "UnboundedDynArray"
            else:
                print("[-] unknown type")
                exit(-1)

            self.full_type_name = type_name
            msg_type_class = ros_utils.get_msg_class_from_name(
                    "idltest_msgs", type_name)

            assert (msg_type_class is not None)

            self.current_type = msg_type_class

            msg = self.current_type()

            # built-in types and fixed arrays are auto-initialized
            # dynamic arrays need to be initialized
            if "DynArray" in type_name:
                dyn_data = []

                for i in range(num_elem):
                    dyn_data.append(self.default_val)

                # print("init:")
                # print(dyn_data)

                msg.data = dyn_data

            self.cur_msg = msg
            # print(self.cur_msg)

        elif self.round_cnt == 1:
            print("STAGE: DETERM - incorrect type")
            print("  [+]", self.full_type_name)
            (ros_type, ext, num_elem) = mutator.random_builtin_type_except(
                    self.init_builtin_type_name)
            bad_data = mutator.get_default_val(ros_type.name)

            # should fail at any time
            expecting = f"Type mismatch - type: {self.full_type_name}, given data: {bad_data} ({ros_type.name})"
            if "Array" in self.full_type_name:
                original_data = self.cur_msg.data[:]
                try:
                    idx = random.randint(0, len(self.cur_msg.data) - 1)
                    self.cur_msg.data[idx] = bad_data
                except Exception as e:
                    # print("ERROR:", e)
                    error = str(e)
                finally:
                    # revert as we expect an error
                    self.cur_msg.data = original_data
            else:
                original_data = self.cur_msg.data
                try:
                    self.cur_msg.data = bad_data
                except Exception as e:
                    # print("ERROR:", e)
                    error = str(e)
                finally:
                    # revert as we expect an error
                    self.cur_msg.data = original_data

        elif self.round_cnt == 2:
            print("STAGE: DETERM - = lower bound")

            expecting = False
            if "Array" in self.full_type_name:
                try:
                    idx = random.randint(0, len(self.cur_msg.data) - 1)
                    self.cur_msg.data[idx] = mutator.get_bounds(
                            self.full_type_name, 0) # lower
                except Exception as e:
                    error = str(e)

            else:
                try:
                    self.cur_msg.data = mutator.get_bounds(
                            self.full_type_name, 0) # lower
                except Exception as e:
                    error = str(e)

        elif self.round_cnt == 3:
            print("STAGE: DETERM - = upper bound")

            expecting = False
            if "Array" in self.full_type_name:
                try:
                    idx = random.randint(0, len(self.cur_msg.data) - 1)
                    self.cur_msg.data[idx] = mutator.get_bounds(
                            self.full_type_name, 1) # upper
                except Exception as e:
                    error = str(e)

            else:
                try:
                    self.cur_msg.data = mutator.get_bounds(
                            self.full_type_name, 1) # upper
                except Exception as e:
                    error = str(e)

        # off-bound checks are only meaningful for numeric types
        # (except double; cannot get smaller than min)
        elif self.round_cnt == 4:
            if "Int" in self.full_type_name or "Float32" in self.full_type_name:
                print("STAGE: DETERM - < lower bound")

                bad_data = mutator.get_bounds(self.full_type_name, 0) - 1

                expecting = f"Bound error - {bad_data} is too small for {self.full_type_name}"
                if "Array" in self.full_type_name:
                    original_data = self.cur_msg.data[:]
                    try:
                        idx = random.randint(0, len(self.cur_msg.data) - 1)
                        self.cur_msg.data[idx] = bad_data
                    except Exception as e:
                        error = str(e)
                    finally:
                        # revert as we expect an error
                        self.cur_msg.data = original_data

                else:
                    original_data = self.cur_msg.data
                    try:
                        self.cur_msg.data = bad_data
                    except Exception as e:
                        error = str(e)
                    finally:
                        # revert as we expect an error
                        self.cur_msg.data = original_data

            else:
                (error, expecting) = self.random_stage()

        # off-bound checks are only meaningful for numeric types
        # (except double; cannot get bigger than max)
        elif self.round_cnt == 5:
            if "Int" in self.full_type_name or "Float32" in self.full_type_name:
                print("STAGE: DETERM - > upper bound")

                bad_data = mutator.get_bounds(self.full_type_name, 1) + 1

                expecting = f"Bound error - {bad_data} is too big for {self.full_type_name}"
                if "Array" in self.full_type_name:
                    original_data = self.cur_msg.data[:]
                    try:
                        idx = random.randint(0, len(self.cur_msg.data) - 1)
                        self.cur_msg.data[idx] = bad_data
                    except Exception as e:
                        error = str(e)
                    finally:
                        # revert as we expect an error
                        self.cur_msg.data = original_data

                else:
                    original_data = self.cur_msg.data
                    try:
                        self.cur_msg.data = bad_data
                    except Exception as e:
                        error = str(e)
                    finally:
                        # revert as we expect an error
                        self.cur_msg.data = original_data

            else:
                (error, expecting) = self.random_stage()

        elif self.round_cnt == 6:
            if "FixedArray" in self.full_type_name:
                print("STAGE: DETERM - fixed array # elements ++")
                expecting = f"Array size of {self.full_type_name} can't be increased"
                original_data = self.cur_msg.data[:]

                arr = list(self.cur_msg.data).append(self.default_val)

                try:
                    self.cur_msg.data = arr
                except Exception as e:
                    error = str(e)
                finally:
                    # revert as we expect an error
                    self.cur_msg.data = original_data

            elif "BoundedDynArray" in self.full_type_name:
                print("STAGE: DETERM - bounded array # elements ++")
                expecting = f"Array size of {self.full_type_name} can't be increased"
                original_data = self.cur_msg.data[:]

                arr = [self.default_val] * (c.BARRAY_BOUND_MAX + 1)

                try:
                    self.cur_msg.data = arr
                except Exception as e:
                    error = str(e)
                finally:
                    # revert as we expect an error
                    self.cur_msg.data = original_data

            else:
                (error, expecting) = self.random_stage()

        elif self.round_cnt == 7:
            if "FixedArray" in self.full_type_name:
                print("STAGE: DETERM - fixed array # elements --")

                expecting = f"Array size of {self.full_type_name} can't be decreased"
                original_data = self.cur_msg.data[:]

                arr = list(self.cur_msg.data)[:-1]
                try:
                    self.cur_msg.data = arr
                except Exception as e:
                    error = str(e)
                finally:
                    # revert as we expect an error
                    self.cur_msg.data = original_data

            else:
                (error, expecting) = self.random_stage()

        else:
            (error, expecting) = self.random_stage()


        # [Schedule]
        # 0. check default msg -> expecting no error
        # 1. out of [lower, upper] bounds value -> expecting an error
        # 2. (arrays) excessive number of elements -> expecting an error
        # 3. special values (if applicable) -> expecting no error ?
        # 4. random values within a valid range

        meta_file = os.path.join(config.meta_dir, "meta-{}".format(frame))
        with open(meta_file, "w") as fp:
            fp.write(
                self.subscriber_node
                + "\t"
                + self.topic_name
                + "\t"
                + str(self.current_type)
                + "\t"
                + str(self.cycle_cnt)
                + "\t"
                + str(self.round_cnt)
            )

        self.round_cnt += 1
        if self.round_cnt == 32:
            self.cycle_cnt += 1
            self.round_cnt = 0
            self.is_new_cycle = True

        return (self.cur_msg, frame, error, expecting)

    def mutate_sequence_mav(self, config):
        frame = str(time.time())

        meta_file = os.path.join(config.meta_dir, "meta-{}".format(frame))
        with open(meta_file, "w") as fp:
            fp.write(
                self.subscriber_node
                + "\t"
                + self.topic_name
                + "\t"
                + str(self.msg_type_class)
                + "\t"
                + str(self.cycle_cnt)
                + "\t"
                + str(self.round_cnt)
            )

        print(
            "\n\x1b[92mCYCLE:",
            self.cycle_cnt,
            "ROUND:",
            self.round_cnt,
            "\x1b[0m",
            frame,
        )
        print("QUEUE LEN:", len(self.fuzzer.queue))

        if self.is_new_cycle:
            # Pick a seed from queue (random selection, not FIFO)
            if len(self.fuzzer.queue) > 0:
                self.msg_list = self.fuzzer.queue.popleft()
                self.from_queue = True
                self.num_msgs = len(self.msg_list)
            else:
                self.msg_list = []

                for i in range(self.num_msgs):
                    msg = self.msg_type_class()
                    self.msg_list.append(msg)

                self.from_queue = False

            self.is_new_cycle = False

            if self.from_queue:
                print("skip GEN (from queue)")

            else:
                print("Generate initial msg list")

                for msg_idx in range(self.num_msgs):
                    msg = self.msg_list[msg_idx]

                    for field_idx in range(self.num_fields):
                        field = self.msg_field_list[field_idx]
                        dtype = field[-1]
                        attr_list = field[:-1]
                        attr_leaf = attr_list[-1]

                        if field[0] == "z":
                            data_val = mutator.gen_int_in_range(
                                0, 1000
                            ) / 1000
                        else:
                            data_val = mutator.gen_int_in_range(
                                -1000, 1000
                            ) / 1000
                        obj = reduce(getattr, attr_list[:-1], msg)
                        setattr(obj, attr_leaf, data_val)

        else:
            # --- Block mutation for PX4 POSCTL ---
            # In POSCTL mode, single-message mutations are ineffective because
            # the position controller brakes immediately when stick returns to
            # center. Mutate a BLOCK of consecutive messages with the same value
            # to simulate sustained stick input.
            block_len = random.randint(5, min(30, self.num_msgs // 3))
            start_idx = random.randint(0, self.num_msgs - block_len)

            roll = random.random()

            if roll < 0.30:
                # --- Multi-axis combined mutation (方案A) ---
                # Mutate 2-3 fields simultaneously on the same block to
                # produce combined effects (e.g., tilt > 45° needs x+y).
                num_fields = random.randint(2, min(3, len(self.msg_field_list)))
                fields = random.sample(self.msg_field_list, num_fields)
                field_vals = []
                for f in fields:
                    if f[0] == "z":
                        val = mutator.gen_int_in_range(0, 1000) / 1000
                    else:
                        val = mutator.gen_int_in_range(-1000, 1000) / 1000
                    field_vals.append((f, val))

                desc = " + ".join(f"{f[0]}={v}" for f, v in field_vals)
                print(f"multi-axis mutate msgs[{start_idx}:{start_idx+block_len}] {desc}")

                for idx in range(start_idx, start_idx + block_len):
                    msg_mutated = deepcopy(self.msg_list[idx])
                    for f, val in field_vals:
                        attr_list = f[:-1]
                        attr_leaf = attr_list[-1]
                        obj = reduce(getattr, attr_list[:-1], msg_mutated)
                        setattr(obj, attr_leaf, val)
                    self.msg_list[idx] = msg_mutated

            elif roll < 0.50:
                # --- Direction flip mutation (方案B) ---
                # Insert a block followed by its opposite to produce high
                # angular rate and jerk from rapid direction changes.
                half_len = block_len // 2
                field = random.choice(self.msg_field_list)
                attr_list = field[:-1]
                attr_leaf = attr_list[-1]

                if field[0] == "z":
                    val1 = mutator.gen_int_in_range(500, 1000) / 1000
                    val2 = mutator.gen_int_in_range(0, 500) / 1000
                else:
                    val1 = mutator.gen_int_in_range(500, 1000) / 1000
                    val2 = -val1

                print(f"flip mutate {field} msgs[{start_idx}:{start_idx+block_len}] {val1} -> {val2}")

                for idx in range(start_idx, start_idx + half_len):
                    msg_mutated = deepcopy(self.msg_list[idx])
                    obj = reduce(getattr, attr_list[:-1], msg_mutated)
                    setattr(obj, attr_leaf, val1)
                    self.msg_list[idx] = msg_mutated
                for idx in range(start_idx + half_len, start_idx + block_len):
                    msg_mutated = deepcopy(self.msg_list[idx])
                    obj = reduce(getattr, attr_list[:-1], msg_mutated)
                    setattr(obj, attr_leaf, val2)
                    self.msg_list[idx] = msg_mutated

            else:
                # --- Standard single-axis block mutation ---
                field = random.choice(self.msg_field_list)
                dtype = field[-1]
                attr_list = field[:-1]
                attr_leaf = attr_list[-1]

                if field[0] == "z":
                    data_val = mutator.gen_int_in_range(0, 1000) / 1000
                else:
                    data_val = mutator.gen_int_in_range(-1000, 1000) / 1000

                print(f"block mutate {field} msgs[{start_idx}:{start_idx+block_len}] = {data_val}")

                for idx in range(start_idx, start_idx + block_len):
                    msg_mutated = deepcopy(self.msg_list[idx])
                    obj = reduce(getattr, attr_list[:-1], msg_mutated)
                    setattr(obj, attr_leaf, data_val)
                    self.msg_list[idx] = msg_mutated

        self.round_cnt += 1
        return (self.msg_list, frame)

    def mutate_sequence_ros(self, config, fbk_list=None, profile=None):
        """
        Block-level, domain-constrained, feedback-adaptive mutation for
        PX4 OFFBOARD velocity control mode (ROS path).

        Args:
            config: RuntimeConfig instance.
            fbk_list: List of Feedback instances for adaptive strategy selection.
            profile: Optional MutationProfile override. Defaults to px4_ros_velocity.
        """
        if profile is None:
            if not hasattr(self, '_ros_profile'):
                self._ros_profile = MutationProfile.px4_ros_velocity()
            profile = self._ros_profile

        frame = str(time.time())

        meta_file = os.path.join(config.meta_dir, "meta-{}".format(frame))
        with open(meta_file, "w") as fp:
            fp.write(
                self.subscriber_node + "\t"
                + self.topic_name + "\t"
                + str(self.msg_type_class) + "\t"
                + str(self.cycle_cnt) + "\t"
                + str(self.round_cnt)
            )

        print(
            "\n\x1b[92mCYCLE:", self.cycle_cnt,
            "ROUND:", self.round_cnt, "\x1b[0m", frame,
        )
        print("QUEUE LEN:", len(self.fuzzer.queue))

        if self.is_new_cycle:
            self._ros_init_cycle(profile)
        else:
            self._ros_mutate_round(profile, fbk_list)

        self.round_cnt += 1
        return (self.msg_list, frame)

    # --- ROS mutation internals ---

    def _ros_init_cycle(self, profile):
        """Initialize or reseed msg_list for a new ROS mutation cycle."""
        if len(self.fuzzer.queue) > 0:
            try:
                queued = self.fuzzer.queue.popleft()
            except IndexError:
                queued = None
            if queued is not None:
                if isinstance(queued, list):
                    self.msg_list = queued
                else:
                    self.msg_list = [deepcopy(queued)
                                     for _ in range(self.num_msgs)]
                self.from_queue = True
                self.num_msgs = len(self.msg_list)
                print("seed from queue")
                self.is_new_cycle = False
                return

        # Queue empty or popleft failed: generate fresh random seed
        self.msg_list = []
        for i in range(self.num_msgs):
            msg = self.msg_type_class()
            self.msg_list.append(msg)
        self.from_queue = False
        print("generate fresh random seed (queue exhausted)")

        for msg_idx in range(self.num_msgs):
            msg = self.msg_list[msg_idx]
            for field in self.msg_field_list:
                attr_list = field[:-1]
                attr_leaf = attr_list[-1]
                fr = profile.get_range(attr_leaf)
                data_val = fr.sample()
                obj = reduce(getattr, attr_list[:-1], msg)
                setattr(obj, attr_leaf, data_val)

        self.is_new_cycle = False

    def _ros_mutate_round(self, profile, fbk_list):
        """Apply one round of block mutation using the selected strategy."""
        # Determine which feedback triggered most recently for adaptation
        recent_feedback = None
        if fbk_list:
            for fbk in fbk_list:
                if fbk.value is not None and fbk.interesting_value is not None:
                    if fbk.value >= fbk.interesting_value:
                        recent_feedback = fbk.name
                        break

        # Stagnation detection: force RANDOM when stuck
        if not hasattr(self, '_no_interesting_rounds'):
            self._no_interesting_rounds = 0
        self._no_interesting_rounds += 1

        if self._no_interesting_rounds >= 10:
            strategy = STRATEGY_RANDOM
        else:
            strategy = profile.select_strategy(recent_feedback)

        min_block, max_block = profile.block_len_range
        max_block = min(max_block, self.num_msgs // 3)
        if max_block < min_block:
            max_block = min_block
        block_len = random.randint(min_block, max_block)
        start_idx = random.randint(0, self.num_msgs - block_len)

        if strategy == STRATEGY_MULTI_AXIS:
            self._ros_multi_axis(profile, start_idx, block_len)
        elif strategy == STRATEGY_FLIP:
            self._ros_direction_flip(profile, start_idx, block_len)
        elif strategy == STRATEGY_RANDOM:
            self._ros_random_fresh(profile, start_idx, block_len)
        else:
            self._ros_single_block(profile, start_idx, block_len)

    def _ros_multi_axis(self, profile, start_idx, block_len):
        """Mutate 2-3 fields simultaneously on a block of messages."""
        num_fields = random.randint(2, min(3, len(self.msg_field_list)))
        fields = random.sample(self.msg_field_list, num_fields)
        field_vals = []
        for f in fields:
            attr_leaf = f[-2] if len(f) > 2 else f[0]
            fr = profile.get_range(attr_leaf)
            val = fr.sample_high_magnitude()
            field_vals.append((f, val))

        desc = " + ".join(f"{f[0]}={v:.2f}" for f, v in field_vals)
        print(f"[ros] multi-axis msgs[{start_idx}:{start_idx+block_len}] {desc}")

        for idx in range(start_idx, start_idx + block_len):
            msg_mutated = deepcopy(self.msg_list[idx])
            for f, val in field_vals:
                attr_list = f[:-1]
                attr_leaf = attr_list[-1]
                obj = reduce(getattr, attr_list[:-1], msg_mutated)
                setattr(obj, attr_leaf, val)
            self.msg_list[idx] = msg_mutated

    def _ros_direction_flip(self, profile, start_idx, block_len):
        """Split block in half: first half gets +val, second half gets -val."""
        half_len = block_len // 2
        field = random.choice(self.msg_field_list)
        attr_list = field[:-1]
        attr_leaf = attr_list[-1]
        fr = profile.get_range(attr_leaf)

        # Generate a high-magnitude value for the flip
        val1 = fr.sample_high_magnitude()
        # For symmetric ranges, flip sign; for asymmetric (like vz), sample opposite end
        if fr.low < 0:
            val2 = -val1
            val2 = fr.clamp(val2)
        else:
            # Asymmetric range: sample from opposite end
            val2 = fr.low + (fr.high - val1)

        print(f"[ros] flip {attr_leaf} msgs[{start_idx}:{start_idx+block_len}]"
              f" {val1:.2f} -> {val2:.2f}")

        for idx in range(start_idx, start_idx + half_len):
            msg_mutated = deepcopy(self.msg_list[idx])
            obj = reduce(getattr, attr_list[:-1], msg_mutated)
            setattr(obj, attr_leaf, val1)
            self.msg_list[idx] = msg_mutated
        for idx in range(start_idx + half_len, start_idx + block_len):
            msg_mutated = deepcopy(self.msg_list[idx])
            obj = reduce(getattr, attr_list[:-1], msg_mutated)
            setattr(obj, attr_leaf, val2)
            self.msg_list[idx] = msg_mutated

    def _ros_single_block(self, profile, start_idx, block_len):
        """Set one field to a constant value across the entire block."""
        field = random.choice(self.msg_field_list)
        attr_list = field[:-1]
        attr_leaf = attr_list[-1]
        fr = profile.get_range(attr_leaf)
        data_val = fr.sample()

        print(f"[ros] block {attr_leaf} msgs[{start_idx}:{start_idx+block_len}]"
              f" = {data_val:.2f}")

        for idx in range(start_idx, start_idx + block_len):
            msg_mutated = deepcopy(self.msg_list[idx])
            obj = reduce(getattr, attr_list[:-1], msg_mutated)
            setattr(obj, attr_leaf, data_val)
            self.msg_list[idx] = msg_mutated

    def _ros_random_fresh(self, profile, start_idx, block_len):
        """Generate completely fresh random values for all fields in the block."""
        print(f"[ros] random msgs[{start_idx}:{start_idx+block_len}]")
        for idx in range(start_idx, start_idx + block_len):
            msg_mutated = deepcopy(self.msg_list[idx])
            for field in self.msg_field_list:
                attr_list = field[:-1]
                attr_leaf = attr_list[-1]
                fr = profile.get_range(attr_leaf)
                data_val = fr.sample()
                obj = reduce(getattr, attr_list[:-1], msg_mutated)
                setattr(obj, attr_leaf, data_val)
            self.msg_list[idx] = msg_mutated

    def mutate_sequence(self, config):
        frame = str(time.time())

        meta_file = os.path.join(config.meta_dir, "meta-{}".format(frame))
        with open(meta_file, "w") as fp:
            fp.write(
                self.subscriber_node
                + "\t"
                + self.topic_name
                + "\t"
                + str(self.msg_type_class)
                + "\t"
                + str(self.cycle_cnt)
                + "\t"
                + str(self.round_cnt)
            )

        print(
            "\n\x1b[92mCYCLE:",
            self.cycle_cnt,
            "ROUND:",
            self.round_cnt,
            "\x1b[0m",
            frame,
        )

        if self.is_new_cycle:
            try:
                queued = self.fuzzer.queue.popleft()
                # Ensure msg_list is always a list of messages
                if isinstance(queued, list):
                    self.msg_list = queued
                else:
                    # Single message seed: wrap into a sequence
                    self.msg_list = [deepcopy(queued)
                                     for _ in range(self.num_msgs)]
                self.from_queue = True
                self.num_msgs = len(self.msg_list)
            except IndexError:
                self.msg_list = []

                for i in range(self.num_msgs):
                    msg = self.msg_type_class()
                    self.msg_list.append(msg)

                self.from_queue = False

            self.is_new_cycle = False

            if self.from_queue:
                print("skip GEN (from queue)")

            else:
                print("Generate initial msg list")

                for msg_idx in range(self.num_msgs):
                    msg = self.msg_list[msg_idx]

                    for field_idx in range(self.num_fields):
                        field = self.msg_field_list[field_idx]
                        dtype = field[-1]
                        attr_list = field[:-1]
                        attr_leaf = attr_list[-1]

                        data_val = mutator.gen_rand_data(dtype, False)
                        obj = reduce(getattr, attr_list[:-1], msg)
                        setattr(obj, attr_leaf, data_val)

                # for msg in self.msg_list:
                    # print(msg)

        else:
            # Decide how many messages to mutate this round:
            # 50% chance: mutate 1 msg (original behavior)
            # 35% chance: mutate 2~3 msgs (multi_mutate)
            # 15% chance: mutate all msgs with same delta (coordinated)
            roll = random.random()
            if roll < 0.50:
                num_to_mutate = 1
            elif roll < 0.85:
                num_to_mutate = random.randint(2, min(3, self.num_msgs))
            else:
                num_to_mutate = self.num_msgs

            indices = random.sample(range(self.num_msgs), num_to_mutate)
            print(f"mutate msgs {indices} from the sequence")

            for msg_idx in indices:
                msg_to_mutate = self.msg_list[msg_idx]

                field = random.choice(self.msg_field_list)
                dtype = field[-1]
                attr_list = field[:-1]
                attr_leaf = attr_list[-1]

                data_val = reduce(
                    getattr, attr_list, msg_to_mutate
                )

                if self.fast_float_determ and dtype.name in mutator.FLOAT_FAST_STAGES:
                    stages = mutator.FLOAT_FAST_STAGES[dtype.name]
                else:
                    stages = mutator.APPLICABLE_STAGES[dtype.name]

                rand_mutation_stage = random.choice(stages)

                if (rand_mutation_stage >= mutator.STAGE_ARITH8
                        and rand_mutation_stage <= mutator.STAGE_ARITH32):
                    arith_val = random.randint(1, 35)
                else:
                    arith_val = 0

                if (rand_mutation_stage >= mutator.STAGE_INTEREST8
                        and rand_mutation_stage <= mutator.STAGE_INTEREST32):
                    interesting_idx = random.randint(
                        0, len(mutator.INTERESTING_MAP[rand_mutation_stage]) - 1)
                elif rand_mutation_stage == mutator.STAGE_INTEREST_FLOAT:
                    interesting_idx = random.randint(
                        0, len(mutator.INTERESTING_FLOAT) - 1)
                else:
                    interesting_idx = -1

                bit_size = dtype.itemsize * 8
                if dtype.itemsize == 0:
                    if dtype.type is np.str_:
                        bit_size = c.STRLEN_MAX * 8

                data_val = mutator.mutate_one(
                    dtype, data_val, rand_mutation_stage,
                    random.randint(0, bit_size - 1),
                    arith_val, interesting_idx,
                )

                if data_val is not None:
                    msg_mutated = deepcopy(msg_to_mutate)
                    obj = reduce(getattr, attr_list[:-1], msg_mutated)
                    setattr(obj, attr_leaf, data_val)
                    self.msg_list[msg_idx] = msg_mutated

        self.round_cnt += 1
        return (self.msg_list, frame)

    def mutate_generic(self, config):
        # Handle one Cycle
        frame = str(time.time())

        meta_file = os.path.join(config.meta_dir, "meta-{}".format(frame))
        with open(meta_file, "w") as fp:
            fp.write(
                self.subscriber_node
                + "\t"
                + self.topic_name
                + "\t"
                + str(self.msg_type_class)
                + "\t"
                + str(self.cycle_cnt)
                + "\t"
                + str(self.round_cnt)
            )

        print(
            "\n\x1b[92mCYCLE:",
            self.cycle_cnt,
            "ROUND:",
            self.round_cnt,
            "EXEC:",
            self.exec_cnt,
            "\x1b[0m",
            frame,
        )
        self.exec_cnt += 1

        # Periodic queue consumption: interrupt deterministic stage to use
        # interesting seeds from feedback, preventing queue starvation.
        self.iter_since_queue_check += 1
        if (not self.is_new_cycle
                and self.iter_since_queue_check >= self.queue_check_interval
                and len(self.fuzzer.queue) > 0):
            print("QUEUE INTERRUPT: consuming seed from queue "
                  f"({len(self.fuzzer.queue)} pending)")
            self.iter_since_queue_check = 0
            # Force a mini-cycle: reset state and consume from queue
            self.cycle_cnt += 1
            self.is_new_cycle = True
            self.round_cnt = 0
            self.cur_fm_field = 0
            self.fm_field_stages = [0] * self.num_fields
            self.fm_determ_stages = [0] * self.num_fields
            self.fm_odata = []
            self.num_msg_mutation = 0
            self.bit_pos = 0
            self.arith_val = -35
            self.interesting_idx = 0

        if self.is_new_cycle:
            print("NEW CYCLE")
            self.iter_since_queue_check = 0
            try:
                msg = self.fuzzer.queue.popleft()
                # Skip incompatible seeds (e.g., sequence lists in single mode)
                while isinstance(msg, list):
                    print("SKIP: queue entry is list, not single msg")
                    msg = self.fuzzer.queue.popleft()
                self.from_queue = True
            except IndexError:
                msg = self.default_msg
                self.from_queue = False
            self.is_new_cycle = False
        else:
            msg = self.init_msg

        if self.cur_fm_field < self.num_fields:
            # A. FIELD MUTATION
            field = self.msg_field_list[self.cur_fm_field]
            cur_round = self.fm_field_stages[self.cur_fm_field]
            print("Current field:", field[0], "({})".format(field[1]))
            # print("cur_round:", cur_round)

            dtype = field[-1]
            attr_list = field[:-1]
            attr_leaf = attr_list[-1]

            if cur_round == 0:
                if self.from_queue:
                    print("SKIP GEN (from queue)")
                    # set odata == msg
                    data_val = reduce(getattr, attr_list, msg)
                    self.fm_odata.append(data_val)

                else:
                    print("STAGE: GEN")
                    # GENERATE RANDOM DATA FOR SELECTED FIELD
                    data_val = mutator.gen_rand_data(dtype, False)
                    obj = reduce(getattr, attr_list[:-1], msg)
                    setattr(obj, attr_leaf, data_val)
                    self.fm_odata.append(data_val)

                    # set rest of the fields (rfield) to default
                    for idx, rfield in enumerate(self.msg_field_list):
                        if idx == self.cur_fm_field:
                            continue
                        dtype = rfield[-1]
                        attr_list = rfield[:-1]
                        attr_leaf = attr_list[-1]
                        data_val = mutator.gen_rand_data(dtype, True)
                        obj = reduce(getattr, attr_list[:-1], msg)
                        setattr(obj, attr_leaf, data_val)

                    # fuzzer.fuzz_and_check(self.fuzzer, msg, frame)

                self.init_msg = msg
                self.round_cnt += 1
                self.fm_field_stages[self.cur_fm_field] += 1
                return (msg, frame)

            if cur_round == 1:
                # GO THROUGH DETERMINISTIC MUTATION STAGES
                cur_determ_stage_id = self.fm_determ_stages[self.cur_fm_field]

                # Use fast stages for float types if enabled
                if self.fast_float_determ and dtype.name in mutator.FLOAT_FAST_STAGES:
                    applicable_stages = mutator.FLOAT_FAST_STAGES[dtype.name]
                else:
                    applicable_stages = mutator.APPLICABLE_STAGES[dtype.name]

                # Check if all deterministic stages are done for this field
                if cur_determ_stage_id >= len(applicable_stages):
                    self.round_cnt += 1
                    self.fm_field_stages[self.cur_fm_field] += 1
                    return (None, None)

                determ_stage = applicable_stages[cur_determ_stage_id]
                print(
                    "STAGE: DETERM {}".format(
                        mutator.STAGE_NAMES[determ_stage]
                    )
                )

                bit_size = dtype.itemsize * 8
                if dtype.itemsize == 0:
                    if dtype.type is np.str_:
                        bit_size = c.STRLEN_MAX * 8

                if determ_stage < 3:
                    # bit flip stages
                    skip = 1
                else:
                    # byte flips and rest of the stages
                    skip = 8

                # obj = reduce(getattr, attr_list[:-1], msg)
                # data_val = getattr(obj, attr_leaf)
                odata = self.fm_odata[self.cur_fm_field]
                print("data before determ mutation:", odata)

                # for bit_pos in range(0, bit_size, skip):
                nmsg = None
                if self.bit_pos < bit_size:
                    print("bit_pos", self.bit_pos, "(", bit_size, ")")
                    if (
                        determ_stage >= mutator.STAGE_ARITH8
                        and determ_stage <= mutator.STAGE_ARITH32
                    ):
                        # for arith_val in range(-35, 36):
                        if self.arith_val < 36:
                            data_val = mutator.mutate_one(
                                dtype,
                                odata,
                                determ_stage,
                                self.bit_pos,
                                self.arith_val,
                            )
                            print("data after determ mutation:", data_val)

                            if data_val is not None:
                                nmsg = deepcopy(msg)
                                obj = reduce(getattr, attr_list[:-1], nmsg)
                                setattr(obj, attr_leaf, data_val)
                                # fuzzer.fuzz_and_check(self.fuzzer, nmsg, frame)

                            self.arith_val += 1
                        else:
                            # move on to the text bits after trying all arithmetic values for the current bits
                            self.bit_pos += skip
                            self.arith_val = -35

                    elif (
                        determ_stage >= mutator.STAGE_INTEREST8
                        and determ_stage <= mutator.STAGE_INTEREST32
                    ):
                        # for interesting_idx in range(len(mutator.INTERESTING_MAP[determ_stage])):
                        if self.interesting_idx < len(
                            mutator.INTERESTING_MAP[determ_stage]
                        ):
                            data_val = mutator.mutate_one(
                                dtype,
                                odata,
                                determ_stage,
                                self.bit_pos,
                                arith_val=0,
                                interesting_idx=self.interesting_idx,
                            )
                            if data_val is not None:
                                nmsg = deepcopy(msg)
                                obj = reduce(getattr, attr_list[:-1], nmsg)
                                setattr(obj, attr_leaf, data_val)
                                # fuzzer.fuzz_and_check(self.fuzzer, nmsg, frame)
                            # print("data after determ mutation:", data_val)

                            self.interesting_idx += 1
                        else:
                            # move on to the text bits after trying all intersting values for the current bits
                            self.bit_pos += skip
                            self.interesting_idx = 0

                    elif determ_stage == mutator.STAGE_INTEREST_FLOAT:
                        # Physics-aware float substitution: iterate through
                        # all interesting float values (no bit_pos needed)
                        if self.interesting_idx < len(
                            mutator.INTERESTING_FLOAT
                        ):
                            data_val = mutator.mutate_one(
                                dtype,
                                odata,
                                determ_stage,
                                0,
                                arith_val=0,
                                interesting_idx=self.interesting_idx,
                            )
                            if data_val is not None:
                                nmsg = deepcopy(msg)
                                obj = reduce(getattr, attr_list[:-1], nmsg)
                                setattr(obj, attr_leaf, data_val)

                            self.interesting_idx += 1
                        else:
                            # All interesting floats exhausted, advance bit_pos
                            # to end the stage
                            self.bit_pos = bit_size
                            self.interesting_idx = 0

                    else:
                        data_val = mutator.mutate_one(
                            dtype, odata, determ_stage, self.bit_pos
                        )
                        # print("data after determ mutation:", data_val)

                        if data_val is not None:
                            nmsg = deepcopy(msg)
                            obj = reduce(getattr, attr_list[:-1], nmsg)
                            setattr(obj, attr_leaf, data_val)
                            # fuzzer.fuzz_and_check(self.fuzzer, nmsg, frame)

                        # flip then move on
                        self.bit_pos += skip

                    return (nmsg, frame)

                print("end of one determ stage of an operation")
                # reset internal states
                self.bit_pos = 0
                self.interesting_idx = 0

                self.fm_determ_stages[self.cur_fm_field] += 1
                if self.fm_determ_stages[self.cur_fm_field] == len(
                    applicable_stages
                ):
                    self.round_cnt += 1
                    self.fm_field_stages[self.cur_fm_field] += 1

                return (
                    None,
                    None,
                )  # temporarily return to avoid nonetype unpacking error

            if cur_round == 2:
                print("STAGE: HAVOC")
                print(self.fm_rand_stages[self.cur_fm_field], "remaining")
                # GO THROUGH RANDOM MUTATION STAGES
                rand_mutation_stage = random.choice(
                    mutator.APPLICABLE_STAGES[dtype.name]
                )

                if (
                    rand_mutation_stage >= mutator.STAGE_ARITH8
                    and rand_mutation_stage <= mutator.STAGE_ARITH32
                ):
                    arith_val = random.randint(1, 35)
                else:
                    arith_val = 0

                if (
                    rand_mutation_stage >= mutator.STAGE_INTEREST8
                    and rand_mutation_stage <= mutator.STAGE_INTEREST32
                ):
                    interesting_idx = random.randint(
                        0,
                        len(mutator.INTERESTING_MAP[rand_mutation_stage]) - 1,
                    )
                elif rand_mutation_stage == mutator.STAGE_INTEREST_FLOAT:
                    interesting_idx = random.randint(
                        0,
                        len(mutator.INTERESTING_FLOAT) - 1,
                    )
                else:
                    interesting_idx = -1

                bit_size = dtype.itemsize * 8
                if dtype.itemsize == 0:
                    if dtype.type is np.str_:
                        bit_size = c.STRLEN_MAX * 8

                data_val = self.fm_odata[self.cur_fm_field]
                # print("data before rand mutation:", data_val)
                data_val = mutator.mutate_one(
                    dtype,
                    data_val,
                    rand_mutation_stage,
                    random.randint(0, bit_size - 1),
                    arith_val,
                    interesting_idx,
                )
                # print("data after rand mutation:", data_val)

                nmsg = None
                if data_val is not None:
                    nmsg = deepcopy(msg)
                    obj = reduce(getattr, attr_list[:-1], nmsg)
                    setattr(obj, attr_leaf, data_val)
                    # fuzzer.fuzz_and_check(self.fuzzer, nmsg, frame)

                self.fm_rand_stages[self.cur_fm_field] -= 1
                if self.fm_rand_stages[self.cur_fm_field] == 0:
                    self.round_cnt += 1
                    self.fm_field_stages[self.cur_fm_field] += 1

                    print("next field")
                    self.cur_fm_field += 1

                return (nmsg, frame)

            # END OF FIELD MUTATION

        else:
            print("STAGE: MSG-ALL")
            # B. MESSAGE MUTATION
            # for field in fields_to_mutate:
            field = random.choice(self.fields_to_mutate)
            dtype = field[-1]
            attr_list = field[:-1]
            attr_leaf = attr_list[-1]

            determ_stage = random.choice(mutator.APPLICABLE_STAGES[dtype.name])
            if (
                determ_stage >= mutator.STAGE_ARITH8
                and determ_stage <= mutator.STAGE_ARITH32
            ):
                arith_val = random.randint(1, 35)
            else:
                arith_val = 0

            if (
                determ_stage >= mutator.STAGE_INTEREST8
                and determ_stage <= mutator.STAGE_INTEREST32
            ):
                interesting_idx = random.randint(
                    0, len(mutator.INTERESTING_MAP[determ_stage]) - 1
                )
            elif determ_stage == mutator.STAGE_INTEREST_FLOAT:
                interesting_idx = random.randint(
                    0, len(mutator.INTERESTING_FLOAT) - 1
                )
            else:
                interesting_idx = -1

            bit_size = dtype.itemsize * 8
            if dtype.itemsize == 0:
                if dtype.type is np.str_:
                    bit_size = c.STRLEN_MAX * 8

            obj = reduce(getattr, attr_list[:-1], msg)
            data_val = getattr(obj, attr_leaf)
            # print("data before msg mutation:", data_val)
            data_val = mutator.mutate_one(
                dtype,
                data_val,
                determ_stage,
                random.randint(0, bit_size - 1),
                arith_val,
                interesting_idx,
            )
            # print("data after msg mutation:", data_val)
            if data_val is not None:
                obj = reduce(getattr, attr_list[:-1], msg)
                setattr(obj, attr_leaf, data_val)

            # fuzzer.fuzz_and_check(self.fuzzer, msg, frame)
            self.num_msg_mutation += 1

            if self.num_msg_mutation == 20:
                print("end of cycle")
                self.cycle_cnt += 1
                self.is_new_cycle = True
                self.round_cnt = 0

                # reset counters
                self.cur_fm_field = 0
                self.fm_field_stages = [0] * self.num_fields
                self.fm_determ_stages = [0] * self.num_fields
                fm_rand_stages = []
                for i in range(self.num_fields):
                    fm_rand_stages.append(random.randint(1, 2))
                self.fm_odata = []
                self.num_msg_mutation = 0
                self.bit_pos = 0
                self.arith_val = -35
                self.interesting_idx = 0

                return (None, frame)

                # finish a cycle and move on to the next item in
                # queue
                # break
            else:
                return (msg, frame)

            # B. END OF MESSAGE MUTATION
