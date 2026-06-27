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

    def test_tb4_profile_declares_drivable_readiness(self):
        import config as config_module
        import target_profiles

        profile = target_profiles.load_profile("turtlebot4_jazzy", REPO_ROOT)

        # Use the TurtleBot4 node marker plus diffdrive activation. /odom can
        # appear before the controller is ready to consume commands, so the
        # activation marker prevents publishing the whole fuzz sequence too
        # early.
        self.assertIn(
            "Turtlebot4 standard running.",
            profile.required_log_patterns_for_readiness,
        )
        self.assertIn(
            "Configured and activated diffdrive_controller",
            profile.required_log_patterns_for_readiness,
        )
        # /odom must be producing data, not merely advertised. /scan stays in
        # the oracle watchlist, but it is not a readiness gate because the
        # minimal empty debug world can advertise /scan without producing laser
        # samples.
        self.assertEqual(
            "nav_msgs/msg/Odometry",
            profile.required_topics_with_data_for_readiness["/odom"],
        )
        self.assertNotIn(
            "/scan",
            profile.required_topics_with_data_for_readiness,
        )
        metadata = profile.to_metadata()
        self.assertIn(
            "required_topics_with_data_for_readiness", metadata
        )

        cfg = config_module.RuntimeConfig()
        target_profiles.attach_profile_to_config(cfg, profile)
        self.assertEqual(
            profile.required_topics_with_data_for_readiness,
            cfg.required_topics_with_data_for_readiness,
        )

    def test_tb4_profile_records_limited_command_observation_chain(self):
        import target_profiles

        profile = target_profiles.load_profile("turtlebot4_jazzy", REPO_ROOT)

        self.assertEqual(
            "geometry_msgs/msg/TwistStamped",
            profile.optional_watch_topics["/diffdrive_controller/cmd_vel"],
        )
        self.assertNotIn(
            "/diffdrive_controller/cmd_vel",
            profile.watch_topics,
        )
        self.assertNotIn(
            "/diffdrive_controller/cmd_vel",
            profile.required_topics_with_data_for_readiness,
        )
        self.assertNotIn(
            "/diffdrive_controller/cmd_vel",
            profile.required_topics_for_readiness,
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            _, watchlist_path = target_profiles.write_profile_metadata(
                profile, tmpdir
            )
            with open(watchlist_path, encoding="utf-8") as fp:
                watchlist = json.load(fp)

        self.assertEqual(
            "geometry_msgs/msg/TwistStamped",
            watchlist["/diffdrive_controller/cmd_vel"],
        )

    def test_profiles_without_data_readiness_default_empty(self):
        import target_profiles

        profile = target_profiles.load_profile("px4_v117_jazzy", REPO_ROOT)
        self.assertEqual(
            {}, profile.required_topics_with_data_for_readiness
        )

    def test_tb4_sequence_seeds_use_conservative_envelope(self):
        import seed_generator

        class FakeTwist:
            def __init__(self):
                self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

        seqs = seed_generator.generate_sequence_seeds(
            "tb4", FakeTwist, seqlen=6
        )
        self.assertTrue(seqs)
        for seq in seqs:
            for msg in seq:
                self.assertLessEqual(abs(msg.linear.x), 0.15 + 1e-9)
                self.assertLessEqual(abs(msg.angular.z), 0.8 + 1e-9)

    def test_tb4_sequence_seeds_include_deep_oracle_patterns(self):
        import seed_generator

        class FakeTwist:
            def __init__(self):
                self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

        seqs = seed_generator.generate_sequence_seeds(
            "tb4", FakeTwist, seqlen=8
        )

        self.assertGreaterEqual(len(seqs), 11)
        # Near-zero threshold probes exercise cmd/odom sign agreement around
        # the TB4 smoke oracle's 0.05 thresholds.
        self.assertTrue(any(
            any(0.0 < abs(msg.linear.x) <= 0.05 for msg in seq)
            for seq in seqs
        ))
        # Arc/coupled-motion seeds drive linear.x and angular.z together,
        # useful for wheel/odom consistency and scan proximity feedback.
        self.assertTrue(any(
            any(abs(msg.linear.x) > 0.0 and abs(msg.angular.z) > 0.0
                for msg in seq)
            for seq in seqs
        ))
        # Timeout-tail seeds leave a nonzero final command so the oracle can
        # observe whether motion decays after publication stops.
        self.assertTrue(any(
            abs(seq[-1].linear.x) > 0.0 or abs(seq[-1].angular.z) > 0.0
            for seq in seqs
        ))

    def test_tb4_velocity_clamp_restricts_mutated_twiststamped(self):
        import seed_generator

        class FakeTwist:
            def __init__(self):
                self.linear = types.SimpleNamespace(x=5.0, y=3.0, z=-4.0)
                self.angular = types.SimpleNamespace(x=2.0, y=-2.0, z=-5.0)

        class FakeTwistStamped:
            def __init__(self):
                self.twist = FakeTwist()

        msg = FakeTwistStamped()
        seed_generator.clamp_velocity_sequence([msg], 0.15, 0.8)

        self.assertEqual(0.15, msg.twist.linear.x)
        self.assertEqual(0.0, msg.twist.linear.y)
        self.assertEqual(0.0, msg.twist.linear.z)
        self.assertEqual(0.0, msg.twist.angular.x)
        self.assertEqual(0.0, msg.twist.angular.y)
        self.assertEqual(-0.8, msg.twist.angular.z)

    def test_tb4_scan_min_range_feedback_prefers_smaller_distances(self):
        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn(
            'Feedback("scan_min_range", FeedbackType.DEC)',
            text,
        )

    def test_tb4_deep_feedback_metrics_are_registered(self):
        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, encoding="utf-8") as fp:
            text = fp.read()

        for name in [
            "tb4_cmd_linear_velocity_ratio",
            "tb4_cmd_angular_velocity_ratio",
            "tb4_odom_linear_velocity_ratio",
            "tb4_odom_angular_velocity_ratio",
            "tb4_linear_accel_ratio",
            "tb4_angular_accel_ratio",
            "tb4_cmd_timeout_motion",
            "tb4_odom_publish_gap",
            "tb4_wheel_odom_consistency_error",
        ]:
            self.assertIn(f'"{name}"', text)

    def test_tb4_deep_feedback_has_mutation_strategy_mapping(self):
        mutation_profile_path = os.path.join(SRC_DIR, "mutation_profile.py")
        with open(mutation_profile_path, encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("_TB4_FEEDBACK_STRATEGY_MAP", text)
        self.assertIn("def turtlebot4_velocity", text)
        self.assertIn(
            '"tb4_cmd_linear_velocity_ratio": STRATEGY_BOUNDARY_PUSH', text
        )
        self.assertIn('"tb4_linear_accel_ratio": STRATEGY_REVERSAL', text)
        self.assertIn(
            '"tb4_cmd_timeout_motion": STRATEGY_SINGLE_BLOCK', text
        )

    def test_tb4_empty_world_avoids_headless_sensors_crash(self):
        world_path = os.path.join(REPO_ROOT, "worlds", "empty.sdf")
        with open(world_path, encoding="utf-8") as fp:
            text = fp.read()

        self.assertNotIn("gz-sim-sensors-system", text)

    def test_tb4_gui_simple_world_uses_official_stable_sensor_policy(self):
        world_path = os.path.join(REPO_ROOT, "worlds", "simple.sdf")
        with open(world_path, encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn('<world name="simple">', text)
        self.assertNotIn("gz-sim-sensors-system", text)

    def test_run_target_uses_simple_world_for_tb4_gui(self):
        run_target_path = os.path.join(REPO_ROOT, "run_target.sh")
        with open(run_target_path, encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("TURTLEBOT4_GUI_WORLD:-simple", text)
        self.assertIn("/work/worlds:", text)
        self.assertIn("TURTLEBOT4_SPLIT_LAUNCH:-1", text)
        self.assertIn('if [[ "${headless}" == "1" || "${split_launch}" == "1" ]]', text)
        self.assertIn('TURTLEBOT4_USE_XVFB:-${headless}', text)
        self.assertNotIn("exec ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py", text)

    def test_run_target_installs_custom_tb4_world_before_official_launch(self):
        run_target_path = os.path.join(REPO_ROOT, "run_target.sh")
        with open(run_target_path, encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("install_custom_tb4_world", text)
        self.assertIn('/work/worlds/${world}.sdf', text)
        self.assertIn('${tb4_gz_share}/worlds/${world}.sdf', text)

    def test_fuzzer_initializes_running_before_profile_readiness(self):
        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("self.running = False", text)

    def test_topic_data_readiness_uses_qos_candidates(self):
        harness_path = os.path.join(SRC_DIR, "harness.py")
        with open(harness_path, encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("qos_candidates", text)
        self.assertIn("QoSDurabilityPolicy.TRANSIENT_LOCAL", text)

    def test_fuzzer_clamps_tb4_velocity_before_execution(self):
        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("clamp_velocity_sequence", text)

    def test_tb4_uses_quality_aware_seedqueue_and_resets_cycle_budget(self):
        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("or self.config.tb4_sitl", text)
        tb4_cycle = text[text.index("elif (fuzzer.config.tb4_sitl"):]
        tb4_cycle = tb4_cycle[:tb4_cycle.index("fuzzer.rounds += 1")]
        self.assertIn("scheduler._seed_interesting_count = 0", tb4_cycle)

    def test_fuzzer_routes_tb4_sequences_to_semantic_mutator(self):
        fuzzer_path = os.path.join(SRC_DIR, "fuzzer.py")
        with open(fuzzer_path, encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("mutate_sequence_tb4(config, fbk_list)", text)

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
        self.assertEqual(
            "std_msgs/msg/String",
            profile.optional_watch_topics["/robofuzz/moveit_plan_params"],
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
