import json
import importlib
import os
import sys
import tempfile
import types
import unittest
from unittest import mock


SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
REPO_ROOT = os.path.dirname(SRC_DIR)
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)


class TargetProfileTests(unittest.TestCase):
    def test_loads_known_profile_and_resolves_watchlist(self):
        import target_profiles

        profile = target_profiles.load_profile("turtlebot4_jazzy", REPO_ROOT)

        self.assertEqual(profile.name, "turtlebot4_jazzy")
        self.assertEqual(profile.family, "turtlebot")
        self.assertEqual(profile.input_topic, "/cmd_vel")
        self.assertEqual(
            profile.input_type, "geometry_msgs/msg/TwistStamped"
        )
        self.assertIn("/odom", profile.watch_topics)
        self.assertIn("/scan", profile.watch_topics)

    def test_aliases_resolve_to_profiles_without_mapping_tb3(self):
        import target_profiles

        self.assertEqual(
            target_profiles.resolve_profile_name(target_profile=None, tb4=True),
            "turtlebot4_jazzy",
        )
        self.assertEqual(
            target_profiles.resolve_profile_name(
                target_profile=None, px4_v117=True
            ),
            "px4_v117_jazzy",
        )
        self.assertEqual(
            target_profiles.resolve_profile_name(
                target_profile=None, test_moveit=True
            ),
            "moveit2_jazzy",
        )
        self.assertIsNone(
            target_profiles.resolve_profile_name(
                target_profile=None, legacy_tb3=True
            )
        )

    def test_writes_resolved_metadata_files(self):
        import target_profiles

        profile = target_profiles.load_profile("moveit2_jazzy", REPO_ROOT)

        with tempfile.TemporaryDirectory() as tmpdir:
            target_profiles.write_profile_metadata(profile, tmpdir)

            profile_path = os.path.join(
                tmpdir, "target_profile.resolved.json"
            )
            watchlist_path = os.path.join(tmpdir, "watchlist.resolved.json")
            self.assertTrue(os.path.exists(profile_path))
            self.assertTrue(os.path.exists(watchlist_path))

            with open(watchlist_path) as fp:
                watchlist = json.load(fp)
            self.assertEqual(
                watchlist["/panda_arm_controller/controller_state"],
                "control_msgs/msg/JointTrajectoryControllerState",
            )

    def test_moveit_profile_enables_execution_oracle(self):
        import config as config_module
        import target_profiles

        profile = target_profiles.load_profile("moveit2_jazzy", REPO_ROOT)
        cfg = config_module.RuntimeConfig()
        target_profiles.attach_profile_to_config(cfg, profile)

        self.assertTrue(cfg.test_moveit)
        self.assertFalse(cfg.moveit_planning_only)

    def test_moveit_profile_declares_required_action_readiness(self):
        import config as config_module
        import target_profiles

        profile = target_profiles.load_profile("moveit2_jazzy", REPO_ROOT)

        self.assertEqual(
            "moveit_msgs/action/MoveGroup",
            profile.required_actions_for_readiness["/move_action"],
        )
        self.assertEqual(
            "control_msgs/action/FollowJointTrajectory",
            profile.required_actions_for_readiness[
                "/panda_arm_controller/follow_joint_trajectory"
            ],
        )
        metadata = profile.to_metadata()
        self.assertIn("required_actions_for_readiness", metadata)

        cfg = config_module.RuntimeConfig()
        target_profiles.attach_profile_to_config(cfg, profile)
        self.assertEqual(
            profile.required_actions_for_readiness,
            cfg.required_actions_for_readiness,
        )

    def test_moveit_profile_declares_fresh_log_readiness(self):
        import config as config_module
        import target_profiles

        profile = target_profiles.load_profile("moveit2_jazzy", REPO_ROOT)

        self.assertIn(
            "You can start planning now!",
            profile.required_log_patterns_for_readiness,
        )
        self.assertIn(
            "Configured and activated panda_arm_controller",
            profile.required_log_patterns_for_readiness,
        )
        metadata = profile.to_metadata()
        self.assertIn("required_log_patterns_for_readiness", metadata)

        cfg = config_module.RuntimeConfig()
        target_profiles.attach_profile_to_config(cfg, profile)
        self.assertEqual(
            profile.required_log_patterns_for_readiness,
            cfg.required_log_patterns_for_readiness,
        )

    def test_moveit_profile_records_result_diagnostic_topics(self):
        import target_profiles

        profile = target_profiles.load_profile("moveit2_jazzy", REPO_ROOT)

        self.assertEqual(
            "std_msgs/msg/Int32",
            profile.optional_watch_topics["/robofuzz/moveit_result_code"],
        )
        self.assertEqual(
            "std_msgs/msg/String",
            profile.optional_watch_topics["/robofuzz/moveit_goal_event"],
        )

    def test_fuzzer_waits_for_profile_actions_before_fuzzing(self):
        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("wait_for_actions", text)
        self.assertIn("action_graph.ready.json", text)

    def test_fuzzer_waits_for_profile_log_patterns_before_fuzzing(self):
        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("wait_for_log_patterns", text)
        self.assertIn("log_readiness.ready.json", text)

    def test_topic_aliases_normalize_current_jazzy_topic_names(self):
        import target_profiles

        state_dict = {
            "/panda_arm_controller/controller_state": [("sample", object())],
            "/fmu/out/vehicle_status_v1": [("sample", object())],
            "/fmu/out/vehicle_local_position_v1": [("sample", object())],
            "/fmu/out/vehicle_attitude": [("sample", object())],
        }
        aliases = {
            "/panda_arm_controller/controller_state":
                "/panda_arm_controller/state",
            "/fmu/out/vehicle_status_v1": "/VehicleStatus_PubSubTopic",
            "/fmu/out/vehicle_local_position_v1":
                "/VehicleLocalPosition_PubSubTopic",
            "/fmu/out/vehicle_attitude": "/VehicleAttitude_PubSubTopic",
        }

        normalized = target_profiles.normalize_state_dict(state_dict, aliases)

        self.assertIn("/panda_arm_controller/state", normalized)
        self.assertIn("/VehicleStatus_PubSubTopic", normalized)
        self.assertIn("/VehicleLocalPosition_PubSubTopic", normalized)
        self.assertIn("/VehicleAttitude_PubSubTopic", normalized)

    def test_harness_delegates_modern_profiles_to_run_target_script(self):
        import target_profiles

        profile = target_profiles.load_profile("px4_v117_jazzy", REPO_ROOT)
        rosidl_runtime_py = types.ModuleType("rosidl_runtime_py")
        rosidl_runtime_py.message_to_ordereddict = lambda msg: {}
        rosidl_runtime_py.set_message_fields = lambda msg, fields: None

        with mock.patch.dict(
            sys.modules,
            {
                "rclpy": types.ModuleType("rclpy"),
                "rosidl_runtime_py": rosidl_runtime_py,
            },
        ):
            harness = importlib.import_module("harness")

            with mock.patch("harness.sp.Popen") as popen:
                harness.run_target_profile(profile, REPO_ROOT)

        kwargs = popen.call_args.kwargs
        cmd = popen.call_args.args[0]
        self.assertIn("run_target.sh", cmd)
        self.assertIn("px4_v117_jazzy", cmd)
        self.assertNotIn("make px4_sitl", cmd)
        self.assertEqual(kwargs["cwd"], REPO_ROOT)

    def test_px4_ros_prepare_flight_uses_nonblocking_spin_once(self):
        fake_modules, fake_rclpy = self._fake_px4_utils_modules()

        with mock.patch.dict(sys.modules, fake_modules):
            sys.modules.pop("px4_utils", None)
            px4_utils = importlib.import_module("px4_utils")

            with mock.patch("px4_utils.time.sleep", lambda _secs: None):
                bridge = px4_utils.Px4BridgeNode(use_mavlink=False)
                bridge.prepare_flight("OFFBOARD")

        self.assertGreater(len(fake_rclpy.spin_calls), 0)
        self.assertTrue(
            all(call == {"timeout_sec": 0.0}
                for call in fake_rclpy.spin_calls)
        )

    def _fake_px4_utils_modules(self):
        class FakePublisher:
            def publish(self, _msg):
                pass

        class FakeNode:
            def create_publisher(self, *_args, **_kwargs):
                return FakePublisher()

            def create_subscription(self, *_args, **_kwargs):
                return object()

        fake_rclpy = types.ModuleType("rclpy")
        fake_rclpy.spin_calls = []
        fake_rclpy.create_node = lambda _name: FakeNode()

        def spin_once(_node, timeout_sec=None):
            if timeout_sec is None:
                raise AssertionError("spin_once must be non-blocking")
            fake_rclpy.spin_calls.append({"timeout_sec": timeout_sec})

        fake_rclpy.spin_once = spin_once

        class VehicleCommand:
            VEHICLE_CMD_DO_SET_MODE = 176
            VEHICLE_CMD_COMPONENT_ARM_DISARM = 400

            def __init__(self):
                self.timestamp = 0
                self.command = 0
                self.param1 = 0.0
                self.param2 = 0.0
                self.target_system = 0
                self.target_component = 0
                self.source_system = 0
                self.source_component = 0
                self.from_external = False

        class TrajectorySetpoint:
            def __init__(self):
                self.timestamp = 0
                self.position = [0.0, 0.0, 0.0]
                self.velocity = [0.0, 0.0, 0.0]
                self.acceleration = [0.0, 0.0, 0.0]
                self.yaw = 0.0
                self.yawspeed = 0.0

        class OffboardControlMode:
            def __init__(self):
                self.timestamp = 0
                self.position = False
                self.velocity = False
                self.acceleration = False
                self.attitude = False
                self.body_rate = False
                self.thrust_and_torque = False
                self.direct_actuator = False

        px4_msgs = types.ModuleType("px4_msgs")
        px4_msgs_msg = types.ModuleType("px4_msgs.msg")
        px4_msgs_msg.VehicleCommand = VehicleCommand
        px4_msgs_msg.TrajectorySetpoint = TrajectorySetpoint
        px4_msgs_msg.OffboardControlMode = OffboardControlMode
        px4_msgs.msg = px4_msgs_msg

        rosidl_runtime_py = types.ModuleType("rosidl_runtime_py")
        rosidl_runtime_py.message_to_ordereddict = lambda msg: {}
        rosidl_runtime_py.set_message_fields = lambda msg, fields: None

        rosidl_parser = types.ModuleType("rosidl_parser")
        rosidl_definition = types.ModuleType("rosidl_parser.definition")
        rosidl_definition.BasicType = lambda _name: object()
        rosidl_parser.definition = rosidl_definition

        pymavlink = types.ModuleType("pymavlink")
        mavutil = types.ModuleType("pymavlink.mavutil")
        pymavlink.mavutil = mavutil

        return {
            "rclpy": fake_rclpy,
            "px4_msgs": px4_msgs,
            "px4_msgs.msg": px4_msgs_msg,
            "rosidl_runtime_py": rosidl_runtime_py,
            "rosidl_parser": rosidl_parser,
            "rosidl_parser.definition": rosidl_definition,
            "pymavlink": pymavlink,
            "pymavlink.mavutil": mavutil,
        }, fake_rclpy


if __name__ == "__main__":
    unittest.main()
