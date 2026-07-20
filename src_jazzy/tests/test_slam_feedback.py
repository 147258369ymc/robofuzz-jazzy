import ast
from copy import deepcopy
import importlib
import os
import sys
import unittest
from types import SimpleNamespace as NS


SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from feedback import Feedback, FeedbackType  # noqa: E402


FUZZER_PATH = os.path.join(SRC_DIR, "fuzzer.py")
SLAM_FEEDBACK_PATH = os.path.join(SRC_DIR, "slam_feedback.py")


def _load_slam_feedback():
    try:
        return importlib.import_module("slam_feedback")
    except ModuleNotFoundError:
        raise AssertionError(
            "slam_feedback helper module must implement the SLAM quality gate"
        ) from None


def _reset_round_values(feedback_list):
    """Mirror fuzzer.py's per-round value reset without clearing history."""
    for fbk in feedback_list:
        fbk.value = fbk.default_value


def _raw_any_interesting(feedback_list):
    # Match fuzzer.py exactly: every metric is evaluated so its historical
    # interesting value is maintained even after an earlier metric fires.
    is_interesting = False
    for fbk in feedback_list:
        cur_interesting = fbk.is_interesting()
        is_interesting = is_interesting or cur_interesting
    return is_interesting


def _evaluate_slam_feedback_interest(slam_feedback, feedback_list):
    evaluator = getattr(
        slam_feedback, "evaluate_slam_feedback_interest", None
    )
    if evaluator is None:
        raise AssertionError(
            "slam_feedback must evaluate collision quality before mutating "
            "Feedback history"
        )
    return evaluator(feedback_list)


def _format_slam_feedback_status(feedback_list):
    slam_feedback = _load_slam_feedback()
    formatter = getattr(slam_feedback, "format_slam_feedback_status", None)
    if formatter is None:
        raise AssertionError(
            "slam_feedback must expose format_slam_feedback_status"
        )
    return formatter(feedback_list)


def _status_feedback(**values):
    return [NS(name=name, value=value) for name, value in values.items()]


class TestSlamFeedbackStatusFormatter(unittest.TestCase):

    def test_judged_status_formats_stable_metric_line(self):
        feedback_list = _status_feedback(
            slam_oracle_valid=1.0,
            slam_collision_detected=0.0,
            slam_pose_sample_count=12.0,
            slam_max_rpe_drift=0.1234,
            slam_ate_rmse=0.4567,
            slam_max_yaw_drift=0.7894,
        )

        self.assertEqual(
            _format_slam_feedback_status(feedback_list),
            "[slam] status=judged valid=1 collision=0 matched_pairs=12 "
            "rpe_m=0.123 ate_m=0.457 yaw_rad=0.789",
        )

    def test_collision_status_has_priority_over_validity(self):
        feedback_list = _status_feedback(
            slam_oracle_valid=1.0,
            slam_collision_detected=1.0,
            slam_pose_sample_count=3.0,
            slam_max_rpe_drift=None,
        )

        self.assertEqual(
            _format_slam_feedback_status(feedback_list),
            "[slam] status=collision valid=1 collision=1 matched_pairs=3 "
            "rpe_m=0.000 ate_m=0.000 yaw_rad=0.000",
        )

    def test_insufficient_status_treats_missing_and_invalid_values_as_zero(self):
        feedback_list = _status_feedback(
            slam_oracle_valid=None,
            slam_collision_detected="not-a-number",
            slam_pose_sample_count=float("nan"),
            slam_max_rpe_drift=float("inf"),
            slam_max_yaw_drift="invalid",
        )

        self.assertEqual(
            _format_slam_feedback_status(feedback_list),
            "[slam] status=insufficient valid=0 collision=0 "
            "matched_pairs=0 rpe_m=0.000 ate_m=0.000 yaw_rad=0.000",
        )


class TestSlamFeedbackQualityGate(unittest.TestCase):

    def test_insufficient_round_sample_growth_remains_interesting(self):
        slam_feedback = _load_slam_feedback()
        feedback_list = [
            Feedback(
                "slam_oracle_valid",
                FeedbackType.INC,
                default_value=0.0,
                min_threshold=1.0,
            ),
            Feedback(
                "slam_pose_sample_count",
                FeedbackType.INC,
                default_value=0.0,
                min_threshold=1.0,
            ),
            Feedback(
                "slam_collision_detected",
                FeedbackType.INC,
                default_value=0.0,
                min_threshold=1.0,
            ),
        ]
        feedback_by_name = {fbk.name: fbk for fbk in feedback_list}

        feedback_by_name["slam_oracle_valid"].update_value(0.0)
        feedback_by_name["slam_pose_sample_count"].update_value(10.0)
        feedback_by_name["slam_collision_detected"].update_value(0.0)

        self.assertFalse(
            _evaluate_slam_feedback_interest(
                slam_feedback, feedback_list
            )
        )
        self.assertEqual(
            feedback_by_name["slam_pose_sample_count"].interesting_value,
            10.0,
        )
        self.assertEqual(
            feedback_by_name["slam_oracle_valid"].interesting_value, 0.0
        )
        self.assertEqual(
            feedback_by_name["slam_collision_detected"].interesting_value,
            0.0,
        )

        # Reset only current-round values; historical interesting values must
        # survive exactly as they do in fuzzer.py between rounds.
        _reset_round_values(feedback_list)
        feedback_by_name["slam_oracle_valid"].update_value(0.0)
        feedback_by_name["slam_pose_sample_count"].update_value(12.0)
        feedback_by_name["slam_collision_detected"].update_value(0.0)

        self.assertTrue(
            _evaluate_slam_feedback_interest(
                slam_feedback, feedback_list
            )
        )
        self.assertEqual(
            feedback_by_name["slam_pose_sample_count"].interesting_value,
            12.0,
        )
        self.assertEqual(
            feedback_by_name["slam_oracle_valid"].interesting_value, 0.0
        )
        self.assertEqual(
            feedback_by_name["slam_collision_detected"].interesting_value,
            0.0,
        )

    def test_collision_round_sample_growth_is_not_requeued(self):
        slam_feedback = _load_slam_feedback()
        feedback_list = [
            Feedback(
                "slam_pose_sample_count",
                FeedbackType.INC,
                default_value=0.0,
                min_threshold=1.0,
            ),
            Feedback(
                "slam_collision_detected",
                FeedbackType.INC,
                default_value=0.0,
                min_threshold=1.0,
            ),
        ]

        # A clean round establishes the historical sample-count baseline.
        feedback_list[0].update_value(20.0)
        feedback_list[1].update_value(0.0)
        self.assertFalse(
            _evaluate_slam_feedback_interest(
                slam_feedback, feedback_list
            )
        )
        self.assertEqual(feedback_list[0].interesting_value, 20.0)
        self.assertEqual(feedback_list[1].interesting_value, 0.0)

        # The next round collides and happens to contain more pose samples.
        # A control copy proves raw Feedback semantics consider that larger
        # sample count interesting, without contaminating the real list.
        raw_control = deepcopy(feedback_list)
        _reset_round_values(raw_control)
        raw_control[0].update_value(30.0)
        raw_control[1].update_value(1.0)
        self.assertTrue(_raw_any_interesting(raw_control))

        _reset_round_values(feedback_list)
        feedback_list[0].update_value(30.0)
        feedback_list[1].update_value(1.0)
        self.assertFalse(
            _evaluate_slam_feedback_interest(
                slam_feedback, feedback_list
            )
        )
        self.assertEqual(feedback_list[0].interesting_value, 20.0)
        self.assertEqual(feedback_list[1].interesting_value, 0.0)

        # A later clean round can be interesting again: the gate reads the
        # current-round value, not collision feedback history.
        _reset_round_values(feedback_list)
        feedback_list[0].update_value(25.0)
        feedback_list[1].update_value(0.0)
        self.assertTrue(
            _evaluate_slam_feedback_interest(
                slam_feedback, feedback_list
            )
        )
        self.assertEqual(feedback_list[0].interesting_value, 25.0)

    def test_missing_collision_metric_leaves_non_slam_feedback_unchanged(self):
        slam_feedback = _load_slam_feedback()
        feedback_list = [
            Feedback(
                "some_other_metric",
                FeedbackType.INC,
                default_value=0.0,
            )
        ]

        self.assertTrue(
            slam_feedback.apply_slam_feedback_quality_gate(
                True, feedback_list
            )
        )


class TestFuzzerSlamFeedbackIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        with open(FUZZER_PATH, encoding="utf-8") as stream:
            cls.source = stream.read()
        cls.tree = ast.parse(cls.source, filename=FUZZER_PATH)

    def test_fuzzer_registers_collision_feedback_with_round_default(self):
        matching_calls = []
        for node in ast.walk(self.tree):
            if not isinstance(node, ast.Call):
                continue
            if not isinstance(node.func, ast.Name) or node.func.id != "Feedback":
                continue
            if not node.args:
                continue
            first_arg = node.args[0]
            if (
                isinstance(first_arg, ast.Constant)
                and first_arg.value == "slam_collision_detected"
            ):
                matching_calls.append(node)

        self.assertEqual(len(matching_calls), 1)
        call = matching_calls[0]
        keywords = {keyword.arg: keyword.value for keyword in call.keywords}
        self.assertEqual(ast.literal_eval(keywords["default_value"]), 0.0)
        self.assertEqual(ast.literal_eval(keywords["min_threshold"]), 1.0)
        self.assertGreaterEqual(len(call.args), 2)
        self.assertIsInstance(call.args[1], ast.Attribute)
        self.assertEqual(call.args[1].attr, "INC")

    def test_fuzzer_registers_validity_once_in_slam_branch(self):
        matching_calls = []
        for node in ast.walk(self.tree):
            if not isinstance(node, ast.Call):
                continue
            if not isinstance(node.func, ast.Name) or node.func.id != "Feedback":
                continue
            if not node.args:
                continue
            first_arg = node.args[0]
            if (
                isinstance(first_arg, ast.Constant)
                and first_arg.value == "slam_oracle_valid"
            ):
                matching_calls.append(node)

        self.assertEqual(len(matching_calls), 1)
        call = matching_calls[0]
        keywords = {keyword.arg: keyword.value for keyword in call.keywords}
        self.assertEqual(ast.literal_eval(keywords["default_value"]), 0.0)
        self.assertEqual(ast.literal_eval(keywords["min_threshold"]), 1.0)
        self.assertGreaterEqual(len(call.args), 2)
        self.assertIsInstance(call.args[1], ast.Attribute)
        self.assertEqual(call.args[1].attr, "INC")

        enclosing_slam_branches = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.If)
            and "slam_toolbox_jazzy" in ast.get_source_segment(
                self.source, node.test
            )
            and any(
                descendant is call
                for statement in node.body
                for descendant in ast.walk(statement)
            )
        ]
        self.assertTrue(
            enclosing_slam_branches,
            "validity feedback must be registered in the SLAM branch",
        )

    def test_fuzzer_imports_slam_status_formatter(self):
        imported_names = [
            alias.name
            for node in self.tree.body
            if isinstance(node, ast.ImportFrom)
            and node.module == "slam_feedback"
            for alias in node.names
        ]
        self.assertEqual(
            imported_names.count("format_slam_feedback_status"), 1
        )

    def test_fuzzer_prints_status_once_after_slam_checker(self):
        formatter_calls = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "format_slam_feedback_status"
        ]
        self.assertEqual(len(formatter_calls), 1)
        formatter_call = formatter_calls[0]
        self.assertEqual(len(formatter_call.args), 1)
        self.assertIsInstance(formatter_call.args[0], ast.Name)
        self.assertEqual(formatter_call.args[0].id, "fbk_list")

        wrapping_prints = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "print"
            and formatter_call in node.args
        ]
        self.assertEqual(len(wrapping_prints), 1)

        checker_calls = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "checker"
            and node.func.attr == "run_checks"
        ]
        self.assertEqual(len(checker_calls), 1)
        self.assertGreater(formatter_call.lineno, checker_calls[0].lineno)

        enclosing_slam_checks = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.If)
            and node.lineno <= formatter_call.lineno <= node.end_lineno
            and "slam_toolbox_jazzy" in ast.get_source_segment(
                self.source, node.test
            )
        ]
        self.assertTrue(
            enclosing_slam_checks,
            "status print must be restricted to slam_toolbox_jazzy",
        )

    def test_fuzzer_uses_slam_evaluator_before_requeue(self):
        helper_calls = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "evaluate_slam_feedback_interest"
        ]
        self.assertEqual(len(helper_calls), 1)
        helper_call = helper_calls[0]
        self.assertEqual(len(helper_call.args), 1)
        self.assertIsInstance(helper_call.args[0], ast.Name)
        self.assertEqual(helper_call.args[0].id, "fbk_list")

        helper_line = helper_call.lineno
        requeue_lines = [
            node.lineno
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "append"
            and node.lineno >= 2500
        ]
        self.assertTrue(any(line > helper_line for line in requeue_lines))

        enclosing_slam_checks = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.If)
            and node.lineno <= helper_line <= getattr(node, "end_lineno", -1)
            and "slam_toolbox_jazzy" in ast.get_source_segment(
                self.source, node.test
            )
        ]
        self.assertTrue(
            enclosing_slam_checks,
            "SLAM evaluator call must be restricted to slam_toolbox_jazzy",
        )
        slam_if = min(enclosing_slam_checks, key=lambda node: node.end_lineno)
        self.assertTrue(
            any(
                isinstance(node, ast.For)
                and any(
                    isinstance(call, ast.Call)
                    and isinstance(call.func, ast.Attribute)
                    and call.func.attr == "is_interesting"
                    for call in ast.walk(node)
                )
                for branch_node in slam_if.orelse
                for node in ast.walk(branch_node)
            ),
            "non-SLAM branch must retain the existing feedback traversal",
        )

    def test_slam_evaluator_checks_collision_before_feedback_traversal(self):
        with open(SLAM_FEEDBACK_PATH, encoding="utf-8") as stream:
            source = stream.read()
        tree = ast.parse(source, filename=SLAM_FEEDBACK_PATH)
        functions = [
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "evaluate_slam_feedback_interest"
        ]
        self.assertEqual(len(functions), 1)
        evaluator = functions[0]

        collision_guards = [
            node
            for node in evaluator.body
            if isinstance(node, ast.If)
            and any(
                isinstance(call, ast.Call)
                and isinstance(call.func, ast.Name)
                and call.func.id == "round_has_slam_collision"
                for call in ast.walk(node.test)
            )
        ]
        feedback_loops = [
            node
            for node in evaluator.body
            if isinstance(node, ast.For)
            and any(
                isinstance(call, ast.Call)
                and isinstance(call.func, ast.Attribute)
                and call.func.attr == "is_interesting"
                for call in ast.walk(node)
            )
        ]
        self.assertEqual(len(collision_guards), 1)
        self.assertEqual(len(feedback_loops), 1)
        self.assertLess(collision_guards[0].lineno, feedback_loops[0].lineno)
        self.assertFalse(
            any(
                isinstance(call, ast.Call)
                and isinstance(call.func, ast.Attribute)
                and call.func.attr == "is_interesting"
                for statement in collision_guards[0].body
                for call in ast.walk(statement)
            ),
            "collision branch must not mutate Feedback history",
        )


if __name__ == "__main__":
    unittest.main()
