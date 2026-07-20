import ast
import os
import unittest
from types import SimpleNamespace as NS


SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CHECKER_PATH = os.path.join(SRC_DIR, "checker.py")


class FeedbackProbe:
    def __init__(self, name):
        self.name = name
        self.value = 0.0

    def update_value(self, value):
        self.value = value


def _config(**overrides):
    values = {
        "rospkg": "not_turtlesim",
        "oracle_mode": "",
        "topic_aliases": {},
        "tb4_sitl": False,
        "tb3_sitl": False,
        "tb3_hitl": False,
        "px4_sitl": False,
        "test_moveit": False,
        "test_rosidl": False,
        "test_cli": False,
    }
    values.update(overrides)
    return NS(**values)


def _extract_run_checks(slam_check, turtlebot_check=None):
    """Compile checker.run_checks with only its import-time dependencies stubbed.

    Importing checker.py directly requires ROS 2's rclpy in the host test
    environment.  Extracting the function keeps its real branch control flow
    while allowing the oracle calls and topic normalizer to be observed.
    """
    with open(CHECKER_PATH, encoding="utf-8") as stream:
        source = stream.read()
    tree = ast.parse(source, filename=CHECKER_PATH)
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "run_checks"
    )
    module = ast.Module(body=[function], type_ignores=[])
    ast.fix_missing_locations(module)

    def normalize_state_dict(state_dict, aliases):
        return state_dict

    if turtlebot_check is None:
        turtlebot_check = lambda *_args: []

    namespace = {
        "target_profiles": NS(normalize_state_dict=normalize_state_dict),
        "oracles": NS(
            slam=NS(check=slam_check),
            turtlebot=NS(check=turtlebot_check),
        ),
        "c": NS(RED="", END=""),
    }
    exec(compile(module, CHECKER_PATH, "exec"), namespace)
    return namespace["run_checks"]


class TestSlamCheckerDispatch(unittest.TestCase):

    def test_bumper_only_round_reaches_slam_oracle_and_marks_collision(self):
        state = {"/bumper_contact": [(123, NS(contacts=[NS()]))]}
        feedback = [FeedbackProbe("slam_collision_detected")]
        calls = []

        def slam_check(config, msg_list, state_dict, feedback_list):
            calls.append((config, msg_list, state_dict, feedback_list))
            if state_dict["/bumper_contact"][0][1].contacts:
                feedback_list[0].update_value(1.0)
            return []

        run_checks = _extract_run_checks(slam_check)
        config = _config(oracle_mode="slam_toolbox_jazzy")
        msg_list = [NS(command="round-input")]

        self.assertEqual(run_checks(config, msg_list, state, feedback), [])
        self.assertEqual(len(calls), 1)
        self.assertIs(calls[0][0], config)
        self.assertIs(calls[0][1], msg_list)
        self.assertIs(calls[0][2], state)
        self.assertIs(calls[0][3], feedback)
        self.assertEqual(feedback[0].value, 1.0)

    def test_empty_slam_state_still_reaches_oracle_for_clean_round_reset(self):
        state = {}
        feedback = [FeedbackProbe("slam_collision_detected")]
        feedback[0].value = 1.0
        calls = []

        def slam_check(config, msg_list, state_dict, feedback_list):
            calls.append((state_dict, feedback_list))
            feedback_list[0].update_value(0.0)
            return []

        run_checks = _extract_run_checks(slam_check)

        self.assertEqual(
            run_checks(
                _config(oracle_mode="slam_toolbox_jazzy"),
                [],
                state,
                feedback,
            ),
            [],
        )
        self.assertEqual(calls, [(state, feedback)])
        self.assertEqual(feedback[0].value, 0.0)

    def test_empty_turtlebot_state_keeps_existing_short_circuit(self):
        slam_calls = []
        turtlebot_calls = []
        run_checks = _extract_run_checks(
            lambda *_args: slam_calls.append(_args),
            lambda *_args: turtlebot_calls.append(_args),
        )

        self.assertEqual(run_checks(_config(tb4_sitl=True), [], {}, []), [])
        self.assertEqual(slam_calls, [])
        self.assertEqual(turtlebot_calls, [])


if __name__ == "__main__":
    unittest.main()
