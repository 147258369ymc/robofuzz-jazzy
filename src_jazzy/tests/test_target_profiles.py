import ast
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
    @staticmethod
    def _spawn_launch_tree():
        launch_path = os.path.join(
            SRC_DIR, "launch", "turtlebot4_fuzz_spawn.launch.py"
        )
        with open(launch_path, encoding="utf-8") as fp:
            return ast.parse(fp.read(), filename=launch_path)

    @staticmethod
    def _assigned_values(tree):
        assignments = {}
        for node in ast.walk(tree):
            if not isinstance(node, ast.Assign) or len(node.targets) != 1:
                continue
            target = node.targets[0]
            if isinstance(target, ast.Name):
                assignments[target.id] = node.value
        return assignments

    @staticmethod
    def _call_name(node):
        if not isinstance(node, ast.Call):
            return None
        if isinstance(node.func, ast.Name):
            return node.func.id
        if isinstance(node.func, ast.Attribute):
            return node.func.attr
        return None

    @staticmethod
    def _run_target_function_source(function_name, end_marker):
        run_target_path = os.path.join(REPO_ROOT, "run_target.sh")
        with open(run_target_path, encoding="utf-8") as fp:
            text = fp.read()

        start = text.index(f"{function_name}() {{")
        end = text.index(end_marker, start)
        return text[start:end]

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

    def test_slam_profile_records_tf_as_optional_evidence(self):
        import target_profiles

        profile = target_profiles.load_profile(
            "slam_toolbox_jazzy", REPO_ROOT
        )

        self.assertIn("/tf", profile.optional_watch_topics)
        self.assertIn("/tf_static", profile.optional_watch_topics)
        self.assertNotIn("/tf", profile.watch_topics)
        self.assertNotIn("/tf_static", profile.watch_topics)

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

    def test_px4_v117_profile_declares_bridge_readiness(self):
        import config as config_module
        import target_profiles

        profile = target_profiles.load_profile("px4_v117_jazzy", REPO_ROOT)

        # PX4 can spend a long time in the first build/boot path before the
        # uXRCE-DDS graph is meaningful. Wait for current-launch PX4 markers
        # before using the ROS graph as the fuzzing readiness gate.
        self.assertIn(
            "INFO  [uxrce_dds_client] init UDP agent IP:127.0.0.1, port:8888",
            profile.required_log_patterns_for_readiness,
        )
        self.assertIn(
            "INFO  [px4] Startup script returned successfully",
            profile.required_log_patterns_for_readiness,
        )
        self.assertEqual(
            "px4_msgs/msg/VehicleStatus",
            profile.required_topics_with_data_for_readiness[
                "/fmu/out/vehicle_status_v1"
            ],
        )
        self.assertEqual(
            "px4_msgs/msg/VehicleLocalPosition",
            profile.required_topics_with_data_for_readiness[
                "/fmu/out/vehicle_local_position_v1"
            ],
        )
        self.assertEqual(
            "px4_msgs/msg/VehicleAttitude",
            profile.required_topics_with_data_for_readiness[
                "/fmu/out/vehicle_attitude"
            ],
        )

        cfg = config_module.RuntimeConfig()
        target_profiles.attach_profile_to_config(cfg, profile)
        self.assertEqual(
            profile.required_log_patterns_for_readiness,
            cfg.required_log_patterns_for_readiness,
        )
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

        profile = target_profiles.load_profile("moveit2_jazzy", REPO_ROOT)
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

    def test_tb4_split_launch_uses_ros2_launch_argument_parser(self):
        function_source = self._run_target_function_source(
            "run_turtlebot4", "run_px4() {"
        )

        spawn_launch = (
            'ros2 launch "${ROOT_DIR}/src_jazzy/launch/'
            'turtlebot4_fuzz_spawn.launch.py"'
        )
        direct_python = (
            'python3 "${ROOT_DIR}/src_jazzy/launch/'
            'turtlebot4_fuzz_spawn.launch.py"'
        )
        self.assertIn(spawn_launch, function_source)
        self.assertNotIn(direct_python, function_source)

    def test_slam_spawn_uses_ros2_launch_argument_parser(self):
        function_source = self._run_target_function_source(
            "run_slam_toolbox", 'if [[ -z "${TARGET}"'
        )

        spawn_launch = (
            'ros2 launch "${ROOT_DIR}/src_jazzy/launch/'
            'turtlebot4_fuzz_spawn.launch.py"'
        )
        direct_python = (
            'python3 "${ROOT_DIR}/src_jazzy/launch/'
            'turtlebot4_fuzz_spawn.launch.py"'
        )
        self.assertIn(spawn_launch, function_source)
        self.assertNotIn(direct_python, function_source)

    def test_spawn_launch_conditionally_includes_official_navigation_rviz(self):
        tree = self._spawn_launch_tree()
        assignments = self._assigned_values(tree)

        rviz_declaration = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and self._call_name(node) == "DeclareLaunchArgument"
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and node.args[0].value == "rviz"
        )
        default_value = next(
            keyword.value
            for keyword in rviz_declaration.keywords
            if keyword.arg == "default_value"
        )
        self.assertEqual("false", default_value.value)

        self.assertIn("pkg_turtlebot4_viz", assignments)
        self.assertIn("rviz_launch", assignments)
        self.assertIn("rviz", assignments)
        viz_package = assignments["pkg_turtlebot4_viz"]
        self.assertEqual(
            "get_package_share_directory", self._call_name(viz_package)
        )
        self.assertEqual("turtlebot4_viz", viz_package.args[0].value)

        rviz_path = assignments["rviz_launch"]
        self.assertEqual("PathJoinSubstitution", self._call_name(rviz_path))
        path_parts = rviz_path.args[0].elts
        self.assertIsInstance(path_parts[0], ast.Name)
        self.assertEqual("pkg_turtlebot4_viz", path_parts[0].id)
        self.assertEqual(
            ["launch", "view_navigation.launch.py"],
            [part.value for part in path_parts[1:]],
        )

        rviz_include = assignments["rviz"]
        self.assertEqual(
            "IncludeLaunchDescription", self._call_name(rviz_include)
        )
        self.assertTrue(any(
            isinstance(node, ast.Name) and node.id == "rviz_launch"
            for node in ast.walk(rviz_include.args[0])
        ))

        condition = next(
            keyword.value
            for keyword in rviz_include.keywords
            if keyword.arg == "condition"
        )
        self.assertEqual("IfCondition", self._call_name(condition))
        rviz_configuration = condition.args[0]
        self.assertEqual(
            "LaunchConfiguration", self._call_name(rviz_configuration)
        )
        self.assertEqual("rviz", rviz_configuration.args[0].value)

        launch_arguments = next(
            keyword.value
            for keyword in rviz_include.keywords
            if keyword.arg == "launch_arguments"
        )
        argument_names = {
            item.elts[0].value: item.elts[1].id
            for item in launch_arguments.elts
        }
        self.assertEqual(
            {"namespace": "namespace", "use_sim_time": "use_sim_time"},
            argument_names,
        )

        self.assertTrue(any(
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "add_action"
            and len(node.args) == 1
            and isinstance(node.args[0], ast.Name)
            and node.args[0].id == "rviz"
            for node in ast.walk(tree)
        ))

    def test_spawn_launch_passes_world_configuration_to_bridge_include(self):
        tree = self._spawn_launch_tree()
        assignments = self._assigned_values(tree)

        world_configuration = assignments["world"]
        self.assertEqual(
            "LaunchConfiguration", self._call_name(world_configuration)
        )
        self.assertEqual("world", world_configuration.args[0].value)

        bridge_include = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and self._call_name(node) == "IncludeLaunchDescription"
            and any(
                isinstance(child, ast.Name)
                and child.id == "turtlebot4_ros_gz_bridge_launch"
                for child in ast.walk(node.args[0])
            )
        )
        launch_arguments = next(
            keyword.value
            for keyword in bridge_include.keywords
            if keyword.arg == "launch_arguments"
        )
        world_argument = next(
            item
            for item in launch_arguments.elts
            if item.elts[0].value == "world"
        )
        self.assertIsInstance(world_argument.elts[1], ast.Name)
        self.assertEqual("world", world_argument.elts[1].id)

    def test_slam_rviz_defaults_off_headless_and_on_in_gui(self):
        function_source = self._run_target_function_source(
            "run_slam_toolbox", 'if [[ -z "${TARGET}"'
        )

        self.assertIn(
            'local slam_rviz_flag="${SLAM_RVIZ:-0}"', function_source
        )
        self.assertIn(
            'slam_rviz_flag="${SLAM_RVIZ:-1}"', function_source
        )
        self.assertIn('"rviz:=${rviz_arg}"', function_source)

    def test_slam_launch_defaults_to_arena_and_retains_world_override(self):
        function_source = self._run_target_function_source(
            "run_slam_toolbox", 'if [[ -z "${TARGET}"'
        )

        self.assertIn('local default_world="slam_arena"', function_source)
        self.assertIn(
            'local world="${TURTLEBOT4_WORLD:-${default_world}}"',
            function_source,
        )
        self.assertNotIn("TURTLEBOT4_GUI_WORLD", function_source)

    def test_slam_stack_uses_official_lifecycle_autostart_launch(self):
        function_source = self._run_target_function_source(
            "start_slam_toolbox_when_ready", "run_slam_toolbox() {"
        )

        # A sync_slam_toolbox_node process can exist while its lifecycle state
        # is still "unconfigured".  The package's official launch owns the
        # configure/activate transitions, so a bare ros2 run is not equivalent.
        self.assertIn(
            'local launch_file="${SLAM_TOOLBOX_LAUNCH:-online_sync_launch.py}"',
            function_source,
        )
        self.assertIn(
            'exec ros2 launch slam_toolbox "${launch_file}" "${launch_args[@]}"',
            function_source,
        )
        self.assertNotIn(
            "ros2 run slam_toolbox", function_source
        )
        self.assertNotIn("SLAM_TOOLBOX_NODE", function_source)

    def test_slam_lifecycle_launch_enables_autostart_and_sim_time(self):
        function_source = self._run_target_function_source(
            "start_slam_toolbox_when_ready", "run_slam_toolbox() {"
        )

        self.assertIn('"autostart:=true"', function_source)
        self.assertIn('"use_lifecycle_manager:=false"', function_source)
        self.assertIn('"use_sim_time:=true"', function_source)

    def test_slam_lifecycle_launch_only_overrides_params_when_resolved(self):
        function_source = self._run_target_function_source(
            "start_slam_toolbox_when_ready", "run_slam_toolbox() {"
        )

        self.assertIn('if [[ -n "${params_file}" ]]; then', function_source)
        self.assertIn(
            'launch_args+=("slam_params_file:=${params_file}")',
            function_source,
        )

    def test_slam_profile_waits_for_lifecycle_activation(self):
        import target_profiles

        profile = target_profiles.load_profile(
            "slam_toolbox_jazzy", REPO_ROOT
        )

        self.assertIn(
            "[LifecycleLaunch] Slamtoolbox node is activating.",
            profile.required_log_patterns_for_readiness,
        )

    def test_slam_profile_declares_arena_as_default_world(self):
        import target_profiles

        profile = target_profiles.load_profile(
            "slam_toolbox_jazzy", REPO_ROOT
        )

        self.assertEqual(
            "slam_arena", profile.raw["launch"]["args"]["world"]
        )

    def test_run_target_help_describes_slam_arena_and_empty_worlds(self):
        run_target_path = os.path.join(REPO_ROOT, "run_target.sh")
        with open(run_target_path, encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("slam_arena (SLAM mapping arena)", text)
        self.assertIn("empty (minimal diagnostics)", text)
        self.assertIn(
            "SLAM_TOOLBOX_LAUNCH=online_sync_launch.py", text
        )

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

    def test_px4_topic_data_readiness_uses_best_effort_only(self):
        harness_path = os.path.join(SRC_DIR, "harness.py")
        with open(harness_path, encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn('topic.startswith("/fmu/out/")', text)
        self.assertIn("return [best_effort_qos]", text)

    def test_slam_ground_truth_readiness_uses_best_effort_only(self):
        harness_path = os.path.join(SRC_DIR, "harness.py")
        with open(harness_path, encoding="utf-8") as fp:
            tree = ast.parse(fp.read(), filename=harness_path)

        wait_fn = next(
            node for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "wait_for_topic_data"
        )
        qos_fn = next(
            node for node in ast.walk(wait_fn)
            if isinstance(node, ast.FunctionDef)
            and node.name == "qos_candidates"
        )
        namespace = {
            "default_qos": object(),
            "best_effort_qos": object(),
            "transient_local_qos": object(),
        }
        exec(
            compile(
                ast.fix_missing_locations(
                    ast.Module(body=[qos_fn], type_ignores=[])
                ),
                harness_path,
                "exec",
            ),
            namespace,
        )
        qos_candidates = namespace["qos_candidates"]

        self.assertEqual(
            [namespace["best_effort_qos"]],
            qos_candidates("/sim_ground_truth_pose"),
        )
        self.assertEqual(
            [namespace["best_effort_qos"]],
            qos_candidates("/fmu/out/vehicle_status_v1"),
        )
        self.assertEqual(
            [
                namespace["default_qos"],
                namespace["best_effort_qos"],
                namespace["transient_local_qos"],
            ],
            qos_candidates("/odom"),
        )

    def test_run_target_exposes_px4_headless_toggle(self):
        run_target_path = os.path.join(REPO_ROOT, "run_target.sh")
        with open(run_target_path, encoding="utf-8") as fp:
            text = fp.read()

        self.assertIn("PX4_HEADLESS=1|0", text)
        self.assertIn('local headless="${PX4_HEADLESS:-1}"', text)
        self.assertIn('if [[ "${headless}" == "1" ]]; then', text)
        self.assertIn("env HEADLESS=1 make px4_sitl_default gz_x500", text)
        self.assertIn("env -u HEADLESS make px4_sitl_default gz_x500", text)
        self.assertNotIn('env HEADLESS="${headless}" make px4_sitl_default gz_x500', text)

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
        self.assertIs(kwargs["stdin"], harness.sp.DEVNULL)

    def test_px4_log_filter_removes_prompt_spam_without_losing_info(self):
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
            sys.modules.pop("harness", None)
            harness = importlib.import_module("harness")

        filt = harness._Px4PromptFilter()
        first = filt.filter(
            b"INFO  [px4] Startup script returned successfully\npxh> \x1b"
        )
        second = filt.filter(b"[2K\rpxh> \x1b[2K\rINFO  [logger] ok\n")
        out = first + second + filt.flush()

        self.assertIn(
            b"INFO  [px4] Startup script returned successfully\n",
            out,
        )
        self.assertIn(b"INFO  [logger] ok\n", out)
        self.assertNotIn(b"pxh>", out)
        self.assertNotIn(b"\x1b[2K", out)

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

    def test_nav2_profile_loads_with_action_goal_mode(self):
        import target_profiles

        profile = target_profiles.load_profile("nav2_jazzy")
        self.assertEqual(profile.family, "turtlebot")
        self.assertEqual(profile.oracle_mode, "nav2_jazzy")
        self.assertEqual(profile.input_mode, "action_goal")
        self.assertIn("/navigate_to_pose",
                      profile.required_actions_for_readiness)

    def test_nav2_attaches_input_mode_to_config(self):
        import target_profiles
        from types import SimpleNamespace

        profile = target_profiles.load_profile("nav2_jazzy")
        cfg = SimpleNamespace()
        target_profiles.attach_profile_to_config(cfg, profile)
        self.assertEqual(cfg.input_mode, "action_goal")

    def test_existing_profiles_keep_declared_input_mode(self):
        import target_profiles

        # Isolation guard: existing profiles already declare input.mode in
        # their YAMLs. Adding input_mode parsing must read those verbatim and
        # not perturb any existing profile. Nothing consumed input_mode before
        # this change, so behavior is unchanged regardless of the value; this
        # asserts the parser leaves each existing profile's declared mode
        # exactly as-is.
        expected = {
            "turtlebot4_jazzy": "velocity_command",
            "slam_toolbox_jazzy": "velocity_command",
            "px4_v117_jazzy": "offboard_ros2",
            "moveit2_jazzy": "motion_plan_request",
        }
        for name, mode in expected.items():
            profile = target_profiles.load_profile(name)
            self.assertEqual(profile.input_mode, mode,
                             f"{name} input_mode changed unexpectedly")

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
