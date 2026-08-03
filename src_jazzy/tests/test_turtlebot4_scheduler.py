import importlib
from copy import deepcopy
import os
import random
import sys
import tempfile
import types
import unittest
from unittest import mock


SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)


class FakeTwist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class FakeTwistStamped:
    def __init__(self):
        self.twist = FakeTwist()


class FeedbackProbe:
    def __init__(self, name, value=1.0, interesting_value=0.0,
                 feed_type=None, prev_interesting_value=None):
        self.name = name
        self.value = value
        self.interesting_value = interesting_value
        self.feed_type = feed_type
        self.prev_interesting_value = prev_interesting_value


def import_scheduler_with_fakes():
    constants = types.ModuleType("constants")
    mutator = types.ModuleType("mutator")
    mutator.gen_special_floats = lambda: 0.0

    numpy = types.ModuleType("numpy")
    numpy.random = types.SimpleNamespace(randint=lambda *args: 50)

    harness = types.ModuleType("harness")
    ros_utils = types.ModuleType("ros_utils")
    ros_utils.flatten_nested_dict = lambda _msg_type_dict: []

    ros2_fuzzer = types.ModuleType("ros2_fuzzer")
    ros_commons = types.ModuleType("ros2_fuzzer.ros_commons")
    ros_commons.map_ros_types = lambda _msg_type_class: {}
    ros2_fuzzer.ros_commons = ros_commons

    with mock.patch.dict(
        sys.modules,
        {
            "constants": constants,
            "mutator": mutator,
            "numpy": numpy,
            "harness": harness,
            "ros_utils": ros_utils,
            "ros2_fuzzer": ros2_fuzzer,
            "ros2_fuzzer.ros_commons": ros_commons,
        },
    ):
        sys.modules.pop("scheduler", None)
        return importlib.import_module("scheduler")


def load_seedqueue_class():
    fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
    with open(fuzzer_path, encoding="utf-8") as fp:
        text = fp.read()

    start = text.index("class SeedQueue:")
    end = text.index("\n\nclass Fuzzer:", start)
    source = text[start:end]
    ns = {"deepcopy": deepcopy, "random": random}
    exec(compile(source, fuzzer_path, "exec"), ns)
    return ns["SeedQueue"]


def load_fuzzer_function(function_name):
    fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
    with open(fuzzer_path, encoding="utf-8") as fp:
        text = fp.read()

    start = text.index(f"def {function_name}(")
    end = text.index("\n\nclass SeedQueue:", start)
    source = text[start:end]
    ns = {}
    exec(compile(source, fuzzer_path, "exec"), ns)
    return ns[function_name]


def fuzzer_source_block(start_marker, end_marker):
    fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
    with open(fuzzer_path, encoding="utf-8") as fp:
        text = fp.read()

    start = text.index(start_marker)
    end = text.index(end_marker, start)
    return text[start:end]


class TurtleBot4SchedulerTests(unittest.TestCase):
    def test_slam_seed_keeps_trigger_across_cycle_selection(self):
        scheduler = import_scheduler_with_fakes()
        sched = object.__new__(scheduler.Scheduler)
        original = [FakeTwistStamped() for _ in range(8)]
        for index, msg in enumerate(original):
            msg.twist.linear.x = 0.20 if index < 4 else -0.20
            msg.twist.angular.z = 0.50

        sched.remember_slam_seed_triggers(
            original, ("slam_max_rpe_drift",)
        )
        sched.record_slam_feedback_triggers(("slam_pose_sample_count",))

        selected = deepcopy(original)
        selected[0].twist.linear.x += 0.01
        sched._activate_slam_seed_triggers(selected)

        self.assertEqual(
            sched._consume_slam_feedback_trigger(),
            "slam_max_rpe_drift",
        )
        self.assertIsNone(sched._consume_slam_feedback_trigger())

    def test_current_round_trigger_overrides_seed_origin_trigger(self):
        scheduler = import_scheduler_with_fakes()
        sched = object.__new__(scheduler.Scheduler)
        seed = [FakeTwistStamped() for _ in range(8)]

        sched.remember_slam_seed_triggers(
            seed, ("slam_loop_closure_residual",)
        )
        sched._activate_slam_seed_triggers(seed)
        sched.record_slam_feedback_triggers(("slam_max_rpe_drift",))

        self.assertEqual(
            sched._consume_slam_feedback_trigger(),
            "slam_max_rpe_drift",
        )
        self.assertIsNone(sched._consume_slam_feedback_trigger())

    def test_collision_discards_seed_origin_trigger(self):
        scheduler = import_scheduler_with_fakes()
        sched = object.__new__(scheduler.Scheduler)
        seed = [FakeTwistStamped() for _ in range(8)]

        sched.remember_slam_seed_triggers(
            seed, ("slam_loop_closure_residual",)
        )
        sched._activate_slam_seed_triggers(seed)
        sched.record_slam_feedback_triggers(
            (), discard_seed_origin=True, current_seed=seed
        )

        self.assertIsNone(sched._consume_slam_feedback_trigger())
        sched._activate_slam_seed_triggers(deepcopy(seed))
        self.assertIsNone(sched._consume_slam_feedback_trigger(seed))

    def test_slam_feedback_trigger_is_consumed_exactly_once(self):
        scheduler = import_scheduler_with_fakes()
        sched = object.__new__(scheduler.Scheduler)

        sched.record_slam_feedback_triggers(
            ("slam_max_rpe_drift", "slam_pose_sample_count")
        )

        self.assertEqual(
            sched._consume_slam_feedback_trigger(),
            "slam_max_rpe_drift",
        )
        self.assertIsNone(sched._consume_slam_feedback_trigger())

    def test_consuming_current_event_invalidates_same_seed_origin(self):
        scheduler = import_scheduler_with_fakes()
        sched = object.__new__(scheduler.Scheduler)
        seed = [FakeTwistStamped() for _ in range(8)]

        sched.remember_slam_seed_triggers(
            seed, ("slam_loop_closure_residual",)
        )
        sched.record_slam_feedback_triggers(
            ("slam_loop_closure_residual",)
        )

        self.assertEqual(
            sched._consume_slam_feedback_trigger(seed),
            "slam_loop_closure_residual",
        )
        sched._activate_slam_seed_triggers(deepcopy(seed))
        self.assertIsNone(sched._consume_slam_feedback_trigger(seed))

    def test_slam_mutation_uses_explicit_trigger_not_list_order(self):
        scheduler = import_scheduler_with_fakes()
        sched = object.__new__(scheduler.Scheduler)
        sched._tb4_is_slam = True
        sched._tb4_max_linear = 0.35
        sched._tb4_max_angular = 1.5
        sched._no_interesting_rounds = 0
        sched.msg_list = [FakeTwistStamped() for _ in range(8)]
        profile = scheduler.MutationProfile.slam_toolbox_velocity(0.35, 1.5)
        sched.record_slam_feedback_triggers(
            ("slam_loop_closure_residual", "slam_pose_sample_count")
        )
        feedback_list = [
            FeedbackProbe(
                "slam_pose_sample_count",
                value=20.0,
                interesting_value=20.0,
                prev_interesting_value=10.0,
            ),
        ]

        with mock.patch.object(
            scheduler.seed_generator,
            "apply_slam_motion_strategy",
        ) as apply_strategy:
            sched._tb4_mutate_round(profile, feedback_list)

        self.assertEqual(
            apply_strategy.call_args.kwargs["recent_feedback"],
            "slam_loop_closure_residual",
        )
        self.assertEqual(
            apply_strategy.call_args.kwargs["strategy"],
            "loop_closure",
        )

    def test_seedqueue_compares_twiststamped_velocity_fields(self):
        SeedQueue = load_seedqueue_class()

        a = FakeTwistStamped()
        b = FakeTwistStamped()
        c = FakeTwistStamped()
        a.twist.linear.x = 0.15
        a.twist.angular.z = 0.8
        b.twist.linear.x = -0.15
        b.twist.angular.z = 0.8
        c.twist.linear.x = 0.16
        c.twist.angular.z = 0.79

        self.assertFalse(SeedQueue._msg_similar(a, b, threshold=0.05))
        self.assertTrue(SeedQueue._msg_similar(a, c, threshold=0.05))

    def test_seedqueue_keeps_sequence_with_hidden_middle_difference(self):
        SeedQueue = load_seedqueue_class()
        original = [FakeTwistStamped() for _ in range(8)]
        changed = deepcopy(original)
        changed[2].twist.linear.x = 0.20
        queue = SeedQueue()

        queue.append(original)
        queue.append(changed, is_readd=True)

        self.assertEqual(len(queue), 2)

    def test_seedqueue_collapses_full_sequence_with_only_small_differences(self):
        SeedQueue = load_seedqueue_class()
        original = [FakeTwistStamped() for _ in range(8)]
        changed = deepcopy(original)
        for msg in changed:
            msg.twist.linear.x = 0.01
            msg.twist.angular.z = -0.01
        queue = SeedQueue()

        queue.append(original)
        queue.append(changed, is_readd=True)

        self.assertEqual(len(queue), 1)

    def test_seedqueue_selection_returns_independent_sequence(self):
        SeedQueue = load_seedqueue_class()
        original = [FakeTwistStamped() for _ in range(8)]
        queue = SeedQueue()
        queue.append(original)

        selected = queue.popleft()
        selected[2].twist.linear.x = 0.20

        self.assertIsNot(selected, queue._items[0])
        self.assertEqual(queue._items[0][2].twist.linear.x, 0.0)
        queue.append(selected, is_readd=True)
        self.assertEqual(len(queue), 2)

    def test_tb4_semantic_reversal_uses_feedback_strategy(self):
        scheduler = import_scheduler_with_fakes()
        sched = object.__new__(scheduler.Scheduler)
        sched.fuzzer = types.SimpleNamespace(queue=[])
        sched.subscriber_node = "turtlebot4_jazzy"
        sched.topic_name = "/cmd_vel"
        sched.msg_type_class = FakeTwistStamped
        sched.cycle_cnt = 0
        sched.round_cnt = 1
        sched.is_new_cycle = False
        sched.from_queue = False
        sched.num_msgs = 8
        sched.msg_list = [FakeTwistStamped() for _ in range(8)]
        sched.msg_field_list = []

        config = types.SimpleNamespace(meta_dir=tempfile.gettempdir(),
                                       seqlen=8)
        fbk_list = [
            FeedbackProbe(
                "tb4_linear_accel_ratio",
                value=1.0,
                interesting_value=1.0,
                prev_interesting_value=-1.0,
            )
        ]

        msg_list, _frame = sched.mutate_sequence_tb4(config, fbk_list)

        lin_values = [msg.twist.linear.x for msg in msg_list]
        self.assertTrue(any(value > 0.0 for value in lin_values), lin_values)
        self.assertTrue(any(value < 0.0 for value in lin_values), lin_values)
        self.assertEqual(8, len(msg_list))

    def test_tb4_reversal_rotates_variants_to_avoid_one_signature(self):
        scheduler = import_scheduler_with_fakes()
        sched = object.__new__(scheduler.Scheduler)
        sched.msg_type_class = FakeTwistStamped
        sched.msg_list = [FakeTwistStamped() for _ in range(8)]

        sched._tb4_reversal(None, "tb4_linear_accel_ratio")
        first = [
            (msg.twist.linear.x, msg.twist.angular.z)
            for msg in sched.msg_list
        ]
        sched._tb4_reversal(None, "tb4_linear_accel_ratio")
        second = [
            (msg.twist.linear.x, msg.twist.angular.z)
            for msg in sched.msg_list
        ]

        self.assertNotEqual(first, second)
        self.assertTrue(
            any(abs(ang_z) < 1e-9 for _lin_x, ang_z in second)
            or any(abs(ang_z) != 0.45 for _lin_x, ang_z in second),
            second,
        )

    def test_tb4_recent_feedback_respects_decreasing_scan_metric(self):
        scheduler = import_scheduler_with_fakes()
        sched = object.__new__(scheduler.Scheduler)

        feedback = FeedbackProbe(
            "scan_min_range",
            value=0.4,
            interesting_value=0.4,
            prev_interesting_value=1.5,
            feed_type=types.SimpleNamespace(name="DEC"),
        )

        self.assertEqual(
            "scan_min_range",
            sched._tb4_recent_feedback([feedback]),
        )

    def test_tb4_recent_feedback_ignores_initial_dec_baseline(self):
        scheduler = import_scheduler_with_fakes()
        sched = object.__new__(scheduler.Scheduler)

        feedback = FeedbackProbe(
            "scan_min_range",
            value=0.0,
            interesting_value=0.0,
            prev_interesting_value=0.0,
            feed_type=types.SimpleNamespace(name="DEC"),
        )

        self.assertIsNone(sched._tb4_recent_feedback([feedback]))

    def test_tb4_error_signature_buckets_duplicate_accel_errors(self):
        tb4_error_signature = load_fuzzer_function("tb4_error_signature")

        sigs = tb4_error_signature([
            "TB4 linear acceleration envelope violated: "
            "accel=2.000 m/s^2 ratio=2.21 limit=0.900",
            "TB4 angular acceleration envelope violated: "
            "accel=12.0 rad/s^2 ratio=1.55 limit=[-7.725, 7.725]",
        ])

        self.assertIn(("tb4_linear_accel", "", 4), sigs)
        self.assertIn(("tb4_angular_accel", "", 3), sigs)

    def test_tb4_feedback_is_not_reset_after_error(self):
        block = fuzzer_source_block(
            "# Do not reset feedback on error",
            "if fuzzer.config.px4_sitl and errs:",
        )

        self.assertIn("fuzzer.config.tb4_sitl", block)


if __name__ == "__main__":
    unittest.main()
