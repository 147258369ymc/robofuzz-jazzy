import json
import os
import sys
import types
import unittest


SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)


class MoveItPlanParamsTests(unittest.TestCase):
    def test_normalize_plan_params_clamps_and_fills_defaults(self):
        import moveit_plan_params

        params = moveit_plan_params.normalize_plan_params(
            {
                "velocity_scaling": 2.0,
                "acceleration_scaling": -1.0,
                "planning_time": 99.0,
                "position_tolerance": 0.0,
            }
        )

        self.assertEqual(1.0, params["velocity_scaling"])
        self.assertEqual(0.01, params["acceleration_scaling"])
        self.assertEqual(30.0, params["planning_time"])
        self.assertEqual(0.0005, params["position_tolerance"])
        self.assertEqual(
            moveit_plan_params.DEFAULT_MOVEIT_PLAN_PARAMS["orientation_tolerance"],
            params["orientation_tolerance"],
        )

    def test_plan_params_json_roundtrip_is_stable(self):
        import moveit_plan_params

        params = moveit_plan_params.normalize_plan_params(
            {
                "velocity_scaling": 0.8,
                "acceleration_scaling": 0.9,
                "planning_time": 3.5,
                "position_tolerance": 0.02,
                "orientation_tolerance": 0.25,
            }
        )

        text = moveit_plan_params.plan_params_to_json(params)
        self.assertEqual(params, moveit_plan_params.plan_params_from_json(text))
        self.assertEqual(sorted(params), sorted(json.loads(text)))

    def test_latest_plan_params_from_state_reads_string_topic(self):
        import moveit_plan_params

        first = moveit_plan_params.plan_params_to_json({"velocity_scaling": 0.4})
        second = moveit_plan_params.plan_params_to_json({"velocity_scaling": 0.7})
        state = {
            "/robofuzz/moveit_plan_params": [
                (1, types.SimpleNamespace(data=first)),
                (2, types.SimpleNamespace(data=second)),
            ]
        }

        params = moveit_plan_params.latest_plan_params_from_state(state)

        self.assertEqual(0.7, params["velocity_scaling"])

    def test_latest_plan_params_from_state_tolerates_missing_or_bad_data(self):
        import moveit_plan_params

        self.assertIsNone(moveit_plan_params.latest_plan_params_from_state({}))
        self.assertIsNone(
            moveit_plan_params.latest_plan_params_from_state(
                {"/robofuzz/moveit_plan_params": [(1, types.SimpleNamespace(data="{"))]}
            )
        )


if __name__ == "__main__":
    unittest.main()
