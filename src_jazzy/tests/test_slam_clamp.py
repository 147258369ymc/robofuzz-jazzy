import ast
import math
import os
import random
import sys
import unittest
from pathlib import Path
from unittest import mock

SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

import seed_generator  # noqa: E402
from mutation_profile import MutationProfile  # noqa: E402


class _Vector3:

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class _Twist:

    def __init__(self, linear=None, angular=None):
        self.linear = linear or _Vector3()
        self.angular = angular or _Vector3()


class _TwistStamped:

    def __init__(self, twist=None):
        self.twist = twist or _Twist()


def _aggressive_message(message_class, linear_x, angular_z):
    twist = _Twist(
        linear=_Vector3(linear_x, 4.0, -4.0),
        angular=_Vector3(3.0, -3.0, angular_z),
    )
    if message_class is _TwistStamped:
        return _TwistStamped(twist)
    return twist


def _twist(msg):
    return msg.twist if hasattr(msg, "twist") else msg


def _integrate_diff_drive(sequence, interval=1.0):
    x = 0.0
    y = 0.0
    yaw = 0.0
    path = 0.0
    for msg in sequence:
        twist = _twist(msg)
        linear = float(twist.linear.x)
        angular = float(twist.angular.z)
        x += linear * math.cos(yaw) * interval
        y += linear * math.sin(yaw) * interval
        yaw += angular * interval
        path += abs(linear) * interval
    wrapped_yaw = (yaw + math.pi) % (2.0 * math.pi) - math.pi
    return math.hypot(x, y), path, abs(wrapped_yaw)


class TestSlamVelocityLimits(unittest.TestCase):

    def setUp(self):
        for k in ("SLAM_MAX_LIN", "SLAM_MAX_ANG"):
            os.environ.pop(k, None)

    def tearDown(self):
        for k in ("SLAM_MAX_LIN", "SLAM_MAX_ANG"):
            os.environ.pop(k, None)

    def test_defaults(self):
        self.assertEqual(seed_generator.slam_velocity_limits(), (0.35, 1.5))
        self.assertEqual(
            getattr(seed_generator, "SLAM_DEFAULT_MAX_LINEAR", None), 0.35,
        )
        self.assertEqual(
            getattr(seed_generator, "SLAM_DEFAULT_MAX_ANGULAR", None), 1.5,
        )

    def test_exact_physical_upper_bounds_are_accepted(self):
        os.environ["SLAM_MAX_LIN"] = "0.46"
        os.environ["SLAM_MAX_ANG"] = "1.9"
        self.assertEqual(seed_generator.slam_velocity_limits(), (0.46, 1.9))
        self.assertEqual(
            getattr(seed_generator, "TB4_PHYSICAL_MAX_LINEAR", None), 0.46,
        )
        self.assertEqual(
            getattr(seed_generator, "TB4_PHYSICAL_MAX_ANGULAR", None), 1.9,
        )

    def test_slam_limits_exceed_tb4_clamp(self):
        lin, ang = seed_generator.slam_velocity_limits()
        safe_linear = getattr(seed_generator, "TB4_SAFE_MAX_LINEAR", None)
        safe_angular = getattr(seed_generator, "TB4_SAFE_MAX_ANGULAR", None)
        self.assertEqual(safe_linear, 0.15)
        self.assertEqual(safe_angular, 0.8)
        self.assertGreater(lin, safe_linear)
        self.assertGreater(ang, safe_angular)

    def test_slam_sequence_seeds_cover_sustained_aggressive_motions(self):
        generator = getattr(
            seed_generator, "generate_slam_sequence_seeds", None,
        )
        self.assertIsNotNone(generator)

        seeds = generator(_TwistStamped, seqlen=20)

        self.assertEqual(len(seeds), 12)
        self.assertTrue(all(len(sequence) == 20 for sequence in seeds))
        self.assertTrue(all(_twist(msg).linear.x == 0.35 for msg in seeds[0]))
        self.assertTrue(all(_twist(msg).linear.x == -0.35 for msg in seeds[1]))
        self.assertTrue(all(_twist(msg).angular.z == 1.5 for msg in seeds[2]))
        self.assertTrue(all(_twist(msg).angular.z == -1.5 for msg in seeds[3]))
        self.assertTrue(
            all(
                abs(_twist(msg).linear.x) > 0.15
                and abs(_twist(msg).angular.z) > 0.8
                for msg in seeds[4] + seeds[5]
            )
        )

        half = 10
        self.assertTrue(
            all(_twist(msg).linear.x == 0.35 for msg in seeds[6][:half])
        )
        self.assertTrue(
            all(_twist(msg).linear.x == -0.35 for msg in seeds[6][half:])
        )
        self.assertTrue(
            all(_twist(msg).angular.z == 1.5 for msg in seeds[7][:half])
        )
        self.assertTrue(
            all(_twist(msg).angular.z == -1.5 for msg in seeds[7][half:])
        )

        start = 20 // 3
        self.assertTrue(
            all(_twist(msg).linear.x == 0.0 for msg in seeds[8][:start])
        )
        self.assertTrue(
            all(_twist(msg).linear.x == 0.35 for msg in seeds[8][start:])
        )
        self.assertTrue(
            all(_twist(msg).angular.z == 0.0 for msg in seeds[9][:start])
        )
        self.assertTrue(
            all(_twist(msg).angular.z == 1.5 for msg in seeds[9][start:])
        )

        endpoint, path, yaw_error = _integrate_diff_drive(
            seeds[10], interval=0.5
        )
        self.assertLessEqual(endpoint, 0.25)
        self.assertGreaterEqual(path, 1.0)
        self.assertLessEqual(yaw_error, 0.05)

        arc_values = [
            (_twist(msg).linear.x, _twist(msg).angular.z)
            for msg in seeds[11]
        ]
        half = len(arc_values) // 2
        self.assertTrue(all(lin_x > 0.0 and ang_z > 0.0
                            for lin_x, ang_z in arc_values[:half]))
        self.assertTrue(all(lin_x > 0.0 and ang_z < 0.0
                            for lin_x, ang_z in arc_values[half:]))

    def test_slam_sequence_seeds_honor_validated_environment_limits(self):
        generator = getattr(
            seed_generator, "generate_slam_sequence_seeds", None,
        )
        self.assertIsNotNone(generator)
        os.environ["SLAM_MAX_LIN"] = "0.4"
        os.environ["SLAM_MAX_ANG"] = "1.8"

        seeds = generator(_TwistStamped, seqlen=8)

        self.assertTrue(all(_twist(msg).linear.x == 0.4 for msg in seeds[0]))
        self.assertTrue(all(_twist(msg).angular.z == 1.8 for msg in seeds[2]))

    def test_ten_message_sequence_omits_timeout_shortened_false_loop(self):
        seeds = seed_generator.generate_slam_sequence_seeds(
            _TwistStamped, seqlen=10
        )

        self.assertEqual(len(seeds), 11)

    def test_eleven_message_sequence_omits_timeout_shortened_false_loop(self):
        seeds = seed_generator.generate_slam_sequence_seeds(
            _TwistStamped, seqlen=11
        )

        self.assertEqual(len(seeds), 11)

    def test_too_short_sequence_omits_false_loop_seed(self):
        seeds = seed_generator.generate_slam_sequence_seeds(
            _TwistStamped, seqlen=8
        )

        self.assertEqual(len(seeds), 11)

    def test_low_angular_limit_rejects_twenty_message_loop(self):
        os.environ["SLAM_MAX_ANG"] = "0.5"

        short_seeds = seed_generator.generate_slam_sequence_seeds(
            _TwistStamped, seqlen=10
        )
        long_seeds = seed_generator.generate_slam_sequence_seeds(
            _TwistStamped, seqlen=20
        )

        self.assertEqual(len(short_seeds), 11)
        self.assertEqual(len(long_seeds), 11)


class TestSlamMutationProfile(unittest.TestCase):

    def setUp(self):
        for key in ("SLAM_MAX_LIN", "SLAM_MAX_ANG"):
            os.environ.pop(key, None)

    def tearDown(self):
        for key in ("SLAM_MAX_LIN", "SLAM_MAX_ANG"):
            os.environ.pop(key, None)

    def test_slam_profile_uses_supplied_aggressive_limits(self):
        factory = getattr(MutationProfile, "slam_toolbox_velocity", None)
        self.assertIsNotNone(factory)

        profile = factory(0.35, 1.5)

        self.assertEqual(profile.get_range("linear.x").low, -0.35)
        self.assertEqual(profile.get_range("linear.x").high, 0.35)
        self.assertEqual(profile.get_range("angular.z").low, -1.5)
        self.assertEqual(profile.get_range("angular.z").high, 1.5)

    def test_deep_feedback_maps_to_semantic_motion_strategies(self):
        profile = MutationProfile.slam_toolbox_velocity(0.35, 1.5)
        mapping = profile.feedback_strategy_map

        self.assertEqual(
            mapping["slam_loop_closure_residual"], "loop_closure"
        )
        self.assertEqual(mapping["slam_revisit_count"], "loop_closure")
        self.assertEqual(mapping["slam_rpe_rotation_max"], "trajectory_arc")
        self.assertEqual(mapping["slam_rotation_excitation"], "trajectory_arc")
        self.assertEqual(mapping["slam_pose_odom_excess_rpe"], "reversal")
        self.assertEqual(mapping["slam_map_growth"], "trajectory_arc")
        self.assertEqual(
            mapping["slam_scan_gate_opportunities"], "single_block"
        )
        self.assertNotIn("slam_odom_rpe_drift", mapping)

    def test_ordinary_tb4_profile_keeps_safe_limits(self):
        profile = MutationProfile.turtlebot4_velocity()

        self.assertEqual(profile.get_range("linear.x").low, -0.15)
        self.assertEqual(profile.get_range("linear.x").high, 0.15)
        self.assertEqual(profile.get_range("angular.z").low, -0.8)
        self.assertEqual(profile.get_range("angular.z").high, 0.8)
        self.assertEqual(
            profile.strategy_weights,
            {
                "boundary_push": 0.30,
                "reversal": 0.30,
                "single_block": 0.25,
                "random": 0.15,
            },
        )

    def test_only_slam_profile_adds_arc_and_loop_strategies(self):
        slam_profile = MutationProfile.slam_toolbox_velocity(0.35, 1.5)
        tb4_profile = MutationProfile.turtlebot4_velocity()

        self.assertIn("trajectory_arc", slam_profile.strategy_weights)
        self.assertIn("loop_closure", slam_profile.strategy_weights)
        self.assertNotIn("trajectory_arc", tb4_profile.strategy_weights)
        self.assertNotIn("loop_closure", tb4_profile.strategy_weights)
        self.assertAlmostEqual(
            sum(slam_profile.strategy_weights.values()), 1.0
        )

    def test_each_invalid_linear_override_is_rejected(self):
        invalid_values = (
            "not-a-number", "nan", "inf", "-inf", "0", "-0.1", "0.460001",
        )
        for raw_value in invalid_values:
            with self.subTest(raw_value=raw_value):
                os.environ["SLAM_MAX_LIN"] = raw_value
                with self.assertRaises(ValueError) as ctx:
                    seed_generator.slam_velocity_limits()
                message = str(ctx.exception)
                self.assertIn("SLAM_MAX_LIN", message)
                self.assertIn("finite", message)
                self.assertIn("> 0", message)
                self.assertIn("<= 0.46", message)

    def test_each_invalid_angular_override_is_rejected(self):
        invalid_values = (
            "not-a-number", "nan", "inf", "-inf", "0", "-0.1", "1.900001",
        )
        for raw_value in invalid_values:
            with self.subTest(raw_value=raw_value):
                os.environ["SLAM_MAX_ANG"] = raw_value
                with self.assertRaises(ValueError) as ctx:
                    seed_generator.slam_velocity_limits()
                message = str(ctx.exception)
                self.assertIn("SLAM_MAX_ANG", message)
                self.assertIn("finite", message)
                self.assertIn("> 0", message)
                self.assertIn("<= 1.9", message)


class TestSlamMotionStrategies(unittest.TestCase):

    def _messages(self, count=12):
        return [_TwistStamped() for _ in range(count)]

    def _apply(self, messages, strategy, recent_feedback=None):
        apply_strategy = getattr(
            seed_generator, "apply_slam_motion_strategy", None,
        )
        self.assertIsNotNone(apply_strategy)
        return apply_strategy(
            messages,
            strategy=strategy,
            max_linear=0.35,
            max_angular=1.5,
            recent_feedback=recent_feedback,
        )

    def test_single_block_uses_one_stable_sign_for_the_nonzero_tail(self):
        messages = self._messages()
        with mock.patch.object(
            random, "choice", return_value=1.0,
        ):
            returned = self._apply(messages, "single_block")

        self.assertIs(returned, messages)
        start = len(messages) // 3
        self.assertTrue(
            all(_twist(msg).linear.x == 0.0 for msg in messages[:start])
        )
        tail = [_twist(msg).linear.x for msg in messages[start:]]
        self.assertTrue(all(value == tail[0] for value in tail))
        self.assertGreater(tail[0], 0.15)

    def test_reversal_uses_two_contiguous_opposite_motion_blocks(self):
        messages = self._messages()
        with mock.patch.object(
            random, "choice", return_value=1.0,
        ):
            self._apply(messages, "reversal")

        half = len(messages) // 2
        first = [_twist(msg).linear.x for msg in messages[:half]]
        second = [_twist(msg).linear.x for msg in messages[half:]]
        self.assertTrue(all(value == first[0] for value in first))
        self.assertTrue(all(value == second[0] for value in second))
        self.assertAlmostEqual(first[0], -second[0])
        self.assertGreater(abs(first[0]), 0.15)

    def test_random_strategy_changes_values_only_at_contiguous_block_edges(self):
        messages = self._messages(count=15)
        with mock.patch.object(
            random, "choice", return_value=1.0,
        ), mock.patch.object(
            random, "uniform", side_effect=[
                0.60, 0.60,
                0.70, 0.70,
                0.80, 0.80,
                0.90, 0.90,
                0.65, 0.65,
            ],
        ):
            self._apply(messages, "random")

        values = [
            (_twist(msg).linear.x, _twist(msg).angular.z)
            for msg in messages
        ]
        runs = []
        for value in values:
            if not runs or runs[-1][0] != value:
                runs.append([value, 1])
            else:
                runs[-1][1] += 1
        self.assertTrue(all(length >= 3 for _, length in runs))
        self.assertTrue(
            all(abs(lin_x) <= 0.35 and abs(ang_z) <= 1.5
                for lin_x, ang_z in values)
        )

    def test_loop_closure_strategy_builds_continuous_circle_then_dwell(self):
        messages = self._messages(count=20)
        with mock.patch.object(random, "choice", return_value=1.0):
            self._apply(
                messages,
                "loop_closure",
                recent_feedback="slam_loop_closure_residual",
            )

        values = [
            (_twist(msg).linear.x, _twist(msg).angular.z)
            for msg in messages
        ]
        self.assertTrue(all(
            lin_x > 0.0 and ang_z > 0.0
            for lin_x, ang_z in values[:-2]
        ))
        self.assertEqual(values[-2:], [(0.0, 0.0), (0.0, 0.0)])

        endpoint, path, yaw_error = _integrate_diff_drive(
            messages, interval=0.5
        )
        self.assertLessEqual(endpoint, 0.25)
        self.assertGreaterEqual(path, 1.0)
        self.assertLessEqual(yaw_error, 0.05)

    def test_short_loop_strategy_falls_back_to_alternating_arc(self):
        messages = self._messages(count=8)
        with mock.patch.object(random, "choice", return_value=1.0):
            self._apply(
                messages,
                "loop_closure",
                recent_feedback="slam_loop_closure_residual",
            )

        values = [
            (_twist(msg).linear.x, _twist(msg).angular.z)
            for msg in messages
        ]
        half = len(values) // 2
        self.assertTrue(all(lin_x > 0.0 and ang_z > 0.0
                            for lin_x, ang_z in values[:half]))
        self.assertTrue(all(lin_x > 0.0 and ang_z < 0.0
                            for lin_x, ang_z in values[half:]))

    def test_trajectory_arc_strategy_alternates_curvature(self):
        messages = self._messages(count=20)
        with mock.patch.object(random, "choice", return_value=1.0):
            self._apply(
                messages,
                "trajectory_arc",
                recent_feedback="slam_rpe_rotation_max",
            )

        values = [
            (_twist(msg).linear.x, _twist(msg).angular.z)
            for msg in messages
        ]
        half = len(values) // 2
        self.assertTrue(all(lin_x > 0.0 and ang_z > 0.0
                            for lin_x, ang_z in values[:half]))
        self.assertTrue(all(lin_x > 0.0 and ang_z < 0.0
                            for lin_x, ang_z in values[half:]))


class TestVelocityLimitSelection(unittest.TestCase):

    def setUp(self):
        for key in ("SLAM_MAX_LIN", "SLAM_MAX_ANG"):
            os.environ.pop(key, None)

    def tearDown(self):
        for key in ("SLAM_MAX_LIN", "SLAM_MAX_ANG"):
            os.environ.pop(key, None)

    def test_slam_mode_selects_validated_slam_limits(self):
        os.environ["SLAM_MAX_LIN"] = "0.4"
        os.environ["SLAM_MAX_ANG"] = "1.8"
        selector = getattr(seed_generator, "velocity_limits_for_oracle", None)
        self.assertIsNotNone(selector)
        self.assertEqual(
            selector("slam_toolbox_jazzy"),
            (0.4, 1.8),
        )

    def test_every_non_slam_mode_selects_safe_tb4_limits(self):
        # Invalid SLAM-only settings must not contaminate ordinary TB4 modes.
        os.environ["SLAM_MAX_LIN"] = "nan"
        os.environ["SLAM_MAX_ANG"] = "-99"
        selector = getattr(seed_generator, "velocity_limits_for_oracle", None)
        self.assertIsNotNone(selector)
        for oracle_mode in ("turtlebot4_jazzy", "", None, "unknown"):
            with self.subTest(oracle_mode=oracle_mode):
                self.assertEqual(
                    selector(oracle_mode),
                    (0.15, 0.8),
                )


class TestVelocityClampBehavior(unittest.TestCase):

    def setUp(self):
        for key in ("SLAM_MAX_LIN", "SLAM_MAX_ANG"):
            os.environ.pop(key, None)

    def tearDown(self):
        for key in ("SLAM_MAX_LIN", "SLAM_MAX_ANG"):
            os.environ.pop(key, None)

    def _assert_clamped(self, oracle_mode, expected_linear, expected_angular):
        selector = getattr(seed_generator, "velocity_limits_for_oracle", None)
        self.assertIsNotNone(selector)
        for message_class in (_Twist, _TwistStamped):
            with self.subTest(message_class=message_class.__name__):
                messages = [
                    _aggressive_message(message_class, 5.0, -5.0),
                    _aggressive_message(message_class, -5.0, 5.0),
                ]
                max_linear, max_angular = selector(oracle_mode)
                returned = seed_generator.clamp_velocity_sequence(
                    messages,
                    max_linear=max_linear,
                    max_angular=max_angular,
                )
                self.assertIs(returned, messages)

                first = messages[0].twist if hasattr(messages[0], "twist") else messages[0]
                second = messages[1].twist if hasattr(messages[1], "twist") else messages[1]
                self.assertEqual(first.linear.x, expected_linear)
                self.assertEqual(first.angular.z, -expected_angular)
                self.assertEqual(second.linear.x, -expected_linear)
                self.assertEqual(second.angular.z, expected_angular)

                for target in (first, second):
                    self.assertEqual(target.linear.y, 0.0)
                    self.assertEqual(target.linear.z, 0.0)
                    self.assertEqual(target.angular.x, 0.0)
                    self.assertEqual(target.angular.y, 0.0)

    def test_slam_defaults_clamp_both_signs_and_zero_other_components(self):
        self._assert_clamped("slam_toolbox_jazzy", 0.35, 1.5)

    def test_ordinary_tb4_clamps_both_signs_and_zeroes_other_components(self):
        self._assert_clamped("turtlebot4_jazzy", 0.15, 0.8)

    def test_valid_slam_overrides_drive_actual_clamp(self):
        os.environ["SLAM_MAX_LIN"] = "0.4"
        os.environ["SLAM_MAX_ANG"] = "1.8"
        self._assert_clamped("slam_toolbox_jazzy", 0.4, 1.8)


class TestFuzzerVelocityClampIntegration(unittest.TestCase):

    def test_fuzzer_queue_selects_slam_specific_seeds(self):
        fuzzer_path = Path(SRC_DIR) / "fuzzer.py"
        text = fuzzer_path.read_text(encoding="utf-8")
        init_queue = text.split("    def init_queue(self):", 1)[1].split(
            "    def init_px4_bridge(self):", 1,
        )[0]

        self.assertIn('== "slam_toolbox_jazzy"', init_queue)
        self.assertIn('platform = "slam"', init_queue)
        self.assertIn("seed_generator.slam_velocity_limits()", init_queue)

    def test_slam_sequence_queue_skips_redundant_single_message_seeds(self):
        fuzzer_path = Path(SRC_DIR) / "fuzzer.py"
        tree = ast.parse(fuzzer_path.read_text(encoding="utf-8"))
        helpers = [
            node for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "_should_add_tb_velocity_singletons"
        ]
        self.assertEqual(len(helpers), 1)

        class _Campaign:
            RND_SEQUENCE = object()
            RND_REPEATED = object()

        namespace = {"Campaign": _Campaign}
        exec(
            compile(
                ast.fix_missing_locations(
                    ast.Module(body=helpers, type_ignores=[])
                ),
                str(fuzzer_path),
                "exec",
            ),
            namespace,
        )
        should_add = namespace["_should_add_tb_velocity_singletons"]
        self.assertFalse(
            should_add("slam_toolbox_jazzy", _Campaign.RND_SEQUENCE)
        )
        self.assertTrue(
            should_add("slam_toolbox_jazzy", _Campaign.RND_REPEATED)
        )
        self.assertTrue(
            should_add("turtlebot4_jazzy", _Campaign.RND_SEQUENCE)
        )
        self.assertTrue(
            should_add("turtlebot4_jazzy", _Campaign.RND_REPEATED)
        )

        fuzzer_class = next(
            node for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == "Fuzzer"
        )
        init_queue = next(
            node for node in fuzzer_class.body
            if isinstance(node, ast.FunctionDef) and node.name == "init_queue"
        )
        singleton_loop = next(
            node for node in ast.walk(init_queue)
            if isinstance(node, ast.For)
            and ast.unparse(node.target) == "(lin_x, ang_z)"
            and ast.unparse(node.iter) == "velocity_seeds"
        )
        matching_guards = [
            node for node in ast.walk(init_queue)
            if isinstance(node, ast.If)
            and singleton_loop in node.body
            and isinstance(node.test, ast.Call)
            and isinstance(node.test.func, ast.Name)
            and node.test.func.id == "_should_add_tb_velocity_singletons"
        ]

        self.assertEqual(len(matching_guards), 1)
        self.assertEqual(
            [ast.unparse(arg) for arg in matching_guards[0].test.args],
            [
                "getattr(self.config, 'oracle_mode', '')",
                "self.config.schedule",
            ],
        )

    def test_tb4_scheduler_selects_and_applies_slam_motion_profile(self):
        scheduler_path = Path(SRC_DIR) / "scheduler.py"
        text = scheduler_path.read_text(encoding="utf-8")
        mutate_sequence = text.split(
            "    def mutate_sequence_tb4(self, config, fbk_list=None):", 1,
        )[1].split("    def _tb4_init_cycle", 1)[0]
        mutate_round = text.split(
            "    def _tb4_mutate_round(self, profile, fbk_list):", 1,
        )[1].split("    @staticmethod\n    def _tb4_twist", 1)[0]

        self.assertIn("seed_generator.slam_velocity_limits()", mutate_sequence)
        self.assertIn(
            "MutationProfile.slam_toolbox_velocity", mutate_sequence,
        )
        self.assertIn(
            "seed_generator.apply_slam_motion_strategy", mutate_round,
        )
        self.assertIn("MutationProfile.turtlebot4_velocity()", mutate_sequence)

    def test_tb4_execution_path_selects_once_then_clamps_once(self):
        fuzzer_path = Path(SRC_DIR) / "fuzzer.py"
        tree = ast.parse(fuzzer_path.read_text(encoding="utf-8"))
        fuzz_msg = next(
            node for node in tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "fuzz_msg"
        )

        selector_calls = []
        clamp_calls = []
        for node in ast.walk(fuzz_msg):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
                continue
            if not isinstance(node.func.value, ast.Name):
                continue
            if node.func.value.id != "seed_generator":
                continue
            if node.func.attr == "velocity_limits_for_oracle":
                selector_calls.append(node)
            elif node.func.attr == "clamp_velocity_sequence":
                clamp_calls.append(node)

        self.assertEqual(len(selector_calls), 1)
        self.assertEqual(len(clamp_calls), 1)

        selector = selector_calls[0]
        self.assertEqual(len(selector.args), 1)
        self.assertEqual(
            ast.unparse(selector.args[0]),
            "getattr(fuzzer.config, 'oracle_mode', '')",
        )

        clamp = clamp_calls[0]
        self.assertEqual(ast.unparse(clamp.args[0]), "msg_list")
        keywords = {keyword.arg: ast.unparse(keyword.value) for keyword in clamp.keywords}
        self.assertEqual(
            keywords,
            {"max_linear": "max_linear", "max_angular": "max_angular"},
        )
