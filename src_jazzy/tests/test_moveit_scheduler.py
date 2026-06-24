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


if __name__ == "__main__":
    unittest.main()
