import importlib
import math
import os
import sys
import types
import unittest
from unittest import mock


SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)


class Pose:
    def __init__(self):
        self.position = types.SimpleNamespace(x=0.3, y=0.0, z=0.5)
        self.orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


def import_scheduler_with_fakes():
    constants = types.ModuleType("constants")
    mutator = types.ModuleType("mutator")
    mutator.gen_special_floats = lambda: 0.0

    numpy = types.ModuleType("numpy")
    numpy.random = types.SimpleNamespace(randint=lambda *args: 50)

    harness = types.ModuleType("harness")
    harness.get_init_moveit_pose = Pose

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
        if "scheduler" in sys.modules:
            return sys.modules["scheduler"]
        return importlib.import_module("scheduler")


def radius(goal):
    return math.sqrt(
        goal.position.x * goal.position.x
        + goal.position.y * goal.position.y
        + goal.position.z * goal.position.z
    )


class MoveItSchedulerTests(unittest.TestCase):
    def test_initial_moveit_seed_pool_avoids_known_unreachable_literals(self):
        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, encoding="utf-8") as fp:
            text = fp.read()

        self.assertNotIn("(0.85, 0.0, 0.5)", text)
        self.assertNotIn("(0.3, 0.0, 1.0)", text)
        self.assertNotIn("(0.3, 0.0, -0.1)", text)

    def test_boundary_push_prefers_reliably_reachable_goals(self):
        scheduler = import_scheduler_with_fakes()
        sched = object.__new__(scheduler.Scheduler)
        profile = scheduler.MutationProfile.moveit_panda()

        goals = sched._moveit_boundary_push(profile, 80, center=(0.0, 0.0, 0.5))

        self.assertTrue(goals)
        self.assertTrue(
            all(radius(goal) <= scheduler.MOVEIT_RELIABLE_REACH_M for goal in goals),
            [radius(goal) for goal in goals],
        )

    def test_reversal_and_arc_stay_reliably_reachable(self):
        scheduler = import_scheduler_with_fakes()
        sched = object.__new__(scheduler.Scheduler)
        profile = scheduler.MutationProfile.moveit_panda()

        goals = []
        goals.extend(sched._moveit_reversal(profile, 20, center=(0.0, 0.0, 0.5)))
        goals.extend(sched._moveit_trajectory_arc(profile, 20, center=(0.0, 0.0, 0.5)))

        self.assertTrue(goals)
        self.assertTrue(
            all(radius(goal) <= scheduler.MOVEIT_RELIABLE_REACH_M for goal in goals),
            [radius(goal) for goal in goals],
        )

    def test_moveit_profile_exposes_plan_param_ranges(self):
        scheduler = import_scheduler_with_fakes()
        profile = scheduler.MutationProfile.moveit_panda()

        params = scheduler._sample_moveit_plan_params(profile)

        self.assertGreaterEqual(params["velocity_scaling"], 0.01)
        self.assertLessEqual(params["velocity_scaling"], 1.0)
        self.assertGreaterEqual(params["acceleration_scaling"], 0.01)
        self.assertLessEqual(params["acceleration_scaling"], 1.0)
        self.assertGreaterEqual(params["planning_time"], 0.1)
        self.assertLessEqual(params["planning_time"], 30.0)
        self.assertGreaterEqual(params["position_tolerance"], 0.0005)
        self.assertLessEqual(params["position_tolerance"], 0.10)
        self.assertGreaterEqual(params["orientation_tolerance"], 0.01)
        self.assertLessEqual(params["orientation_tolerance"], 1.57)

    def test_moveit_mutation_records_plan_params_side_channel(self):
        scheduler = import_scheduler_with_fakes()
        sched = object.__new__(scheduler.Scheduler)
        sched.fuzzer = types.SimpleNamespace(queue=[])
        sched.subscriber_node = "/move_group"
        sched.topic_name = "/motion_plan_request"
        sched.msg_type_class = Pose
        sched.cycle_cnt = 0
        sched.round_cnt = 0
        sched.is_new_cycle = True
        sched.from_queue = False
        config = types.SimpleNamespace(meta_dir=os.devnull)

        with mock.patch("builtins.open", mock.mock_open()):
            sched.mutate_sequence_moveit(config)

        self.assertIsInstance(sched._plan_params, dict)
        self.assertIn("velocity_scaling", sched._plan_params)

    def test_execution_failure_path_advances_moveit_cycle_and_round(self):
        # mutate_sequence_moveit increments round_cnt before execution; the
        # RuntimeError handler must advance the cycle + global round so a
        # persistently failing target cannot loop forever on one seed
        # (the cycle-16 / 523-round runaway).
        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, encoding="utf-8") as fp:
            text = fp.read()

        except_idx = text.index("except RuntimeError as e:")
        continue_idx = text.index("continue", except_idx)
        handler = text[except_idx:continue_idx]

        self.assertIn("_advance_moveit_cycle(scheduler, fbk_list, fuzzer)",
                      handler)
        self.assertIn("fuzzer.rounds += 1", handler)
        self.assertIn("fuzzer.config.maxloop", handler)

    def test_advance_moveit_cycle_helper_is_single_source_of_truth(self):
        # The adaptive cycle bookkeeping must live in one helper that both the
        # normal path and the failure path call, so they cannot diverge.
        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("def _advance_moveit_cycle(scheduler, fbk_list, fuzzer):",
                      text)
        # The cycle-finished bookkeeping should not be inlined twice anymore;
        # the reset + cycle increment must only live inside the helper.
        self.assertEqual(
            1, text.count('scheduler.cycle_cnt += 1\n'
                          '        scheduler.is_new_cycle = True\n'
                          '        scheduler._recent_interesting_rounds = []'),
            "MoveIt cycle reset should only live in _advance_moveit_cycle",
        )

    def test_failure_path_records_harness_failure_marker(self):
        # A no-goal-handle / readiness-timeout round must be explicitly marked
        # as a harness failure so the bookkeeping-only round is self-describing
        # rather than inferred from "metadata without rosbag".
        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn(
            "def _record_moveit_harness_failure(fuzzer, scheduler, frame, "
            "reason):",
            text,
        )
        except_idx = text.index("except RuntimeError as e:")
        continue_idx = text.index("continue", except_idx)
        handler = text[except_idx:continue_idx]
        self.assertIn(
            "_record_moveit_harness_failure(", handler,
            "failure branch must record a harness-failure marker",
        )

    def test_record_harness_failure_writes_marker_without_touching_rosbags(self):
        # Behavioral test of the helper in isolation: fuzzer.py is not
        # importable here (ROS deps), so exec just the helper's source in a
        # namespace that only provides os + json.
        import json as _json
        import re
        import tempfile

        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, encoding="utf-8") as fp:
            text = fp.read()

        start = text.index("def _record_moveit_harness_failure")
        end = text.index("\ndef fuzz_msg", start)
        source = text[start:end]

        ns = {"os": os, "json": _json}
        exec(compile(source, fuzzer_path, "exec"), ns)
        record = ns["_record_moveit_harness_failure"]

        with tempfile.TemporaryDirectory() as meta_dir:
            fuzzer = types.SimpleNamespace(
                config=types.SimpleNamespace(meta_dir=meta_dir))
            sched = types.SimpleNamespace(cycle_cnt=16, round_cnt=7)
            frame = "1782327650.2862499"

            record(fuzzer, sched, frame,
                   RuntimeError("MoveIt goal request returned no goal handle"))

            marker = os.path.join(meta_dir, f"harness_fail-{frame}")
            self.assertTrue(os.path.exists(marker))
            with open(marker) as fp:
                data = _json.load(fp)
            self.assertEqual(data["cycle"], 16)
            self.assertEqual(data["round"], 7)
            self.assertEqual(data["frame"], frame)
            self.assertIn("no goal handle", data["reason"])
            # Only the marker file is written; nothing resembling rosbag/error
            # artifacts is created.
            self.assertEqual(os.listdir(meta_dir), [f"harness_fail-{frame}"])

    def test_record_harness_failure_is_safe_without_meta_dir(self):
        import json as _json

        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, encoding="utf-8") as fp:
            text = fp.read()
        start = text.index("def _record_moveit_harness_failure")
        end = text.index("\ndef fuzz_msg", start)
        source = text[start:end]
        ns = {"os": os, "json": _json}
        exec(compile(source, fuzzer_path, "exec"), ns)
        record = ns["_record_moveit_harness_failure"]

        fuzzer = types.SimpleNamespace(
            config=types.SimpleNamespace(meta_dir=None))
        sched = types.SimpleNamespace(cycle_cnt=0, round_cnt=0)
        # Must not raise when meta_dir is unset.
        record(fuzzer, sched, "1.0", RuntimeError("x"))


if __name__ == "__main__":
    unittest.main()
