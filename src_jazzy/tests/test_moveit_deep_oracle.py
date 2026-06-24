import importlib
import os
import sys
import types
import unittest
from unittest import mock


SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
REPO_ROOT = os.path.dirname(SRC_DIR)
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


class FeedbackProbe:
    def __init__(self, name, default=0.0):
        self.name = name
        self.value = default

    def update_value(self, value):
        self.value = value


class FakeTransform:
    def __init__(self, pos):
        self.pos = pos


class FakeChain:
    def __init__(self, pos):
        self._pos = pos

    def forward_kinematics(self, _joint_angle_map):
        return {"panda_hand": FakeTransform(self._pos)}


def import_moveit_with_fk(final_pos, build_counter=None):
    fake_kinpy = types.ModuleType("kinpy")

    def build_chain(_urdf):
        if build_counter is not None:
            build_counter["calls"] = build_counter.get("calls", 0) + 1
        return FakeChain(final_pos)

    fake_kinpy.build_chain_from_urdf = build_chain
    with mock.patch.dict(sys.modules, {"kinpy": fake_kinpy}):
        sys.modules.pop("oracles.moveit", None)
        from oracles import moveit
    moveit.PANDA_URDF = __file__
    return moveit


PANDA_HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]


def point(positions=None, velocities=None, accelerations=None):
    return types.SimpleNamespace(
        positions=list(positions or PANDA_HOME),
        velocities=list(velocities or [0.0] * 7),
        accelerations=list(accelerations or [0.0] * 7),
    )


def controller_state(ts, reference=None, feedback=None, error=None):
    names = [f"panda_joint{i}" for i in range(1, 8)]
    return (
        ts,
        types.SimpleNamespace(
            joint_names=names,
            reference=reference or point(),
            feedback=feedback or point(),
            error=error or point(positions=[0.0] * 7),
        ),
    )


def joint_state(ts, positions=None, velocities=None):
    names = [f"panda_joint{i}" for i in range(1, 8)]
    return (
        ts,
        types.SimpleNamespace(
            name=names,
            position=list(positions or PANDA_HOME),
            velocity=list(velocities or [0.0] * 7),
        ),
    )


def status_msg(statuses):
    return types.SimpleNamespace(
        status_list=[types.SimpleNamespace(status=s) for s in statuses]
    )


def motion_plan_request(x=0.40, y=0.10, z=0.50):
    pos = types.SimpleNamespace(x=x, y=y, z=z)
    primitive_pose = types.SimpleNamespace(position=pos)
    region = types.SimpleNamespace(primitive_poses=[primitive_pose])
    pc = types.SimpleNamespace(constraint_region=region)
    constraints = types.SimpleNamespace(position_constraints=[pc])
    return types.SimpleNamespace(goal_constraints=[constraints])


def pose(x=0.40, y=0.10, z=0.50):
    return types.SimpleNamespace(
        position=types.SimpleNamespace(x=x, y=y, z=z),
        orientation=types.SimpleNamespace(w=1.0),
    )


def base_state(final_status=4):
    samples = [
        controller_state(i * 10_000_000)
        for i in range(8)
    ]
    return {
        "/joint_states": [joint_state(0)],
        "/panda_arm_controller/state": samples,
        "/move_action/_action/status": [
            (samples[-1][0], status_msg([final_status]))
        ],
        "/motion_plan_request": [
            (samples[-1][0] - 1, motion_plan_request())
        ],
    }


class MoveItDeepOracleTests(unittest.TestCase):
    def test_endpoint_baseline_deviation_updates_feedback_without_error(self):
        moveit = import_moveit_with_fk(final_pos=(0.409, 0.10, 0.50))
        feedback = [FeedbackProbe("end_point_deviation")]

        errs = moveit.check(
            types.SimpleNamespace(moveit_planning_only=False),
            [],
            base_state(),
            feedback,
        )

        self.assertEqual([], errs)
        self.assertGreater(feedback[0].value, 0.008)
        self.assertLess(feedback[0].value, 0.010)

    def test_endpoint_large_success_outlier_is_classified_error(self):
        moveit = import_moveit_with_fk(final_pos=(0.45, 0.10, 0.50))
        feedback = [FeedbackProbe("success_endpoint_outlier_score")]

        errs = moveit.check(
            types.SimpleNamespace(moveit_planning_only=False),
            [],
            base_state(),
            feedback,
        )

        self.assertTrue(
            any("success_but_endpoint_outlier" in err for err in errs),
            errs,
        )
        self.assertGreater(feedback[0].value, 0.04)

    def test_endpoint_fk_chain_is_cached_across_checks(self):
        build_counter = {"calls": 0}
        moveit = import_moveit_with_fk(
            final_pos=(0.409, 0.10, 0.50),
            build_counter=build_counter,
        )

        cfg = types.SimpleNamespace(moveit_planning_only=False)
        moveit.check(cfg, [], base_state(), [FeedbackProbe("end_point_deviation")])
        moveit.check(cfg, [], base_state(), [FeedbackProbe("end_point_deviation")])

        self.assertEqual(1, build_counter["calls"])

    def test_controller_state_acceleration_reports_direct_toppra_bound(self):
        moveit = import_moveit_with_fk(final_pos=(0.40, 0.10, 0.50))
        feedback = [FeedbackProbe("smoothness_violation_ratio")]
        bad_state = base_state()
        bad_state["/panda_arm_controller/state"] = [
            controller_state(
                i * 10_000_000,
                reference=point(accelerations=[4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            )
            for i in range(8)
        ]

        errs = moveit.check(
            types.SimpleNamespace(moveit_planning_only=False),
            [],
            bad_state,
            feedback,
        )

        self.assertTrue(
            any("TOPP-RA planned acceleration" in err for err in errs),
            errs,
        )

    def test_sustained_jerk_spike_is_candidate_error(self):
        moveit = import_moveit_with_fk(final_pos=(0.40, 0.10, 0.50))
        accelerations = ([0.0], [3.5], [0.0], [3.5], [0.0], [0.0], [0.0], [0.0])
        state = base_state()
        state["/panda_arm_controller/state"] = [
            controller_state(
                i * 10_000_000,
                reference=point(
                    accelerations=list(acc) + [0.0] * 6,
                ),
            )
            for i, acc in enumerate(accelerations)
        ]

        errs = moveit.check(
            types.SimpleNamespace(moveit_planning_only=False),
            [],
            state,
            [FeedbackProbe("smoothness_violation_ratio")],
        )

        self.assertTrue(
            any("trajectory_smoothness_violation_candidate" in err for err in errs),
            errs,
        )

    def test_moveit_oracle_does_not_runtime_load_oracleir_compiler(self):
        moveit_path = os.path.join(SRC_DIR, "oracles", "moveit.py")
        with open(moveit_path, encoding="utf-8") as fp:
            moveit_text = fp.read()

        self.assertNotIn("load_compiled_oracles", moveit_text)
        self.assertNotIn("_load_moveit_oracle_ir", moveit_text)
        self.assertNotIn("_run_oracle_ir", moveit_text)

    def test_controller_reference_velocity_violation_reports_direct_bound(self):
        moveit = import_moveit_with_fk(final_pos=(0.40, 0.10, 0.50))
        feedback = [
            FeedbackProbe("desired_vel_max_ratio"),
            FeedbackProbe("smoothness_violation_ratio"),
        ]
        bad_state = base_state()
        bad_state["/panda_arm_controller/state"] = [
            controller_state(
                i * 10_000_000,
                reference=point(velocities=[3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            )
            for i in range(8)
        ]

        errs = moveit.check(
            types.SimpleNamespace(moveit_planning_only=False),
            [],
            bad_state,
            feedback,
        )

        self.assertTrue(
            any("TOPP-RA planned velocity" in err for err in errs),
            errs,
        )
        self.assertGreater(feedback[1].value, 1.0)

    def test_success_without_controller_samples_is_execution_state_missing(self):
        moveit = import_moveit_with_fk(final_pos=(0.40, 0.10, 0.50))
        state = base_state()
        state["/panda_arm_controller/state"] = []

        errs = moveit.check(
            types.SimpleNamespace(moveit_planning_only=False),
            [],
            state,
            [FeedbackProbe("execution_sample_count", default=None)],
        )

        self.assertTrue(
            any("execution_state_missing" in err for err in errs),
            errs,
        )

    def test_reachable_failure_result_code_becomes_candidate_signal(self):
        moveit = import_moveit_with_fk(final_pos=(0.40, 0.10, 0.50))
        state = base_state(final_status=6)
        state["/robofuzz/moveit_result_code"] = [
            (state["/move_action/_action/status"][-1][0], types.SimpleNamespace(data=99999))
        ]
        feedback = [FeedbackProbe("reachable_rejection_score")]

        errs = moveit.check(
            types.SimpleNamespace(moveit_planning_only=False),
            [pose(0.35, 0.10, 0.45), pose(0.40, 0.05, 0.50)],
            state,
            feedback,
        )

        self.assertTrue(
            any("reachable_failure_candidate" in err for err in errs),
            errs,
        )
        self.assertGreaterEqual(feedback[0].value, 1.0)

    def test_result_status_inconsistency_is_classified(self):
        moveit = import_moveit_with_fk(final_pos=(0.40, 0.10, 0.50))
        state = base_state(final_status=4)
        state["/robofuzz/moveit_result_code"] = [
            (state["/move_action/_action/status"][-1][0], types.SimpleNamespace(data=99999))
        ]

        errs = moveit.check(
            types.SimpleNamespace(moveit_planning_only=False),
            [pose(0.35, 0.10, 0.45)],
            state,
            [FeedbackProbe("status_transition_anomaly_score")],
        )

        self.assertTrue(
            any("result_status_inconsistency" in err for err in errs),
            errs,
        )

    def test_new_moveit_feedback_is_registered_and_strategy_mapped(self):
        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        mutation_path = os.path.join(SRC_DIR, "mutation_profile.py")

        with open(fuzzer_path, encoding="utf-8") as fp:
            fuzzer_text = fp.read()
        with open(mutation_path, encoding="utf-8") as fp:
            mutation_text = fp.read()

        for name in (
            "desired_vel_max_ratio",
            "desired_acc_max_ratio",
            "desired_jerk_max_ratio",
            "execution_sample_count",
            "success_endpoint_outlier_score",
            "reachable_rejection_score",
            "status_transition_anomaly_score",
            "tracking_error_growth",
            "smoothness_violation_ratio",
        ):
            self.assertIn(f'Feedback("{name}"', fuzzer_text)
            self.assertIn(f'"{name}"', mutation_text)


if __name__ == "__main__":
    unittest.main()
