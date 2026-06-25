import os
import sys
import unittest


SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)


class FeedbackProbe:
    def __init__(self, name, value):
        self.name = name
        self.value = value


class MoveItFeedbackBucketTests(unittest.TestCase):
    def test_collect_new_feedback_buckets_returns_only_unseen_buckets(self):
        import moveit_feedback_buckets

        seen = set()
        feedback = [
            FeedbackProbe("desired_vel_max_ratio", 0.84),
            FeedbackProbe("joint_motion_range", 1.1),
            FeedbackProbe("unbucketed_metric", 99.0),
        ]

        first = moveit_feedback_buckets.collect_new_feedback_buckets(
            feedback,
            seen,
        )
        second = moveit_feedback_buckets.collect_new_feedback_buckets(
            feedback,
            seen,
        )

        self.assertEqual(2, len(first))
        self.assertEqual([], second)
        self.assertTrue(all(key in seen for key in first))

    def test_feedback_bucket_key_ignores_defaults_and_unknown_metrics(self):
        import moveit_feedback_buckets

        self.assertIsNone(
            moveit_feedback_buckets.feedback_bucket_key(
                FeedbackProbe("desired_vel_max_ratio", 0.0),
            )
        )
        self.assertIsNone(
            moveit_feedback_buckets.feedback_bucket_key(
                FeedbackProbe("not_moveit_semantic", 1.0),
            )
        )

    def test_fuzzer_wires_bucket_novelty_only_for_moveit(self):
        with open(os.path.join(SRC_DIR, "fuzzer.py"), encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("collect_new_feedback_buckets", text)
        self.assertIn("_seen_feedback_buckets", text)
        self.assertIn("if fuzzer.config.test_moveit", text)

    def test_fuzzer_binds_moveit_plan_params_side_channel(self):
        with open(os.path.join(SRC_DIR, "fuzzer.py"), encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("plan_params = getattr(scheduler, \"_plan_params\", None)", text)
        self.assertIn("partial(", text)
        self.assertIn("plan_params=plan_params", text)

    def test_scaling_violation_has_dedup_signature_branch(self):
        with open(os.path.join(SRC_DIR, "fuzzer.py"), encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("scaling_violation", text)
        self.assertIn("desired_(?:vel|acc)_ratio", text)


if __name__ == "__main__":
    unittest.main()
