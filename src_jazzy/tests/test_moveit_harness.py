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


class _Auto:
    """Auto-vivifying namespace: reading a missing attribute returns a child
    _Auto; setting an attribute stores the value. Lets tests fake nested ROS
    message objects without declaring every field."""

    def __init__(self):
        object.__setattr__(self, "_store", {})

    def __getattr__(self, name):
        store = object.__getattribute__(self, "_store")
        if name not in store:
            store[name] = _Auto()
        return store[name]

    def __setattr__(self, name, value):
        object.__getattribute__(self, "_store")[name] = value


class _FakeFuture:
    def __init__(self, value):
        self._value = value

    def result(self):
        return self._value


class _FakeGoalHandle:
    def __init__(self, result_value):
        self.accepted = True
        self._result_value = result_value

    def get_result_async(self):
        return _FakeFuture(self._result_value)


def _build_moveit_fakes(record):
    """Return a sys.modules patch dict faking rclpy + moveit/shape messages.
    `record` captures what moveit_send_command did for assertions."""
    fake_rclpy = types.ModuleType("rclpy")

    class _FakePublisher:
        def publish(self, _msg):
            record["published"] = record.get("published", 0) + 1

    class _FakeNode:
        def create_publisher(self, *_a, **_k):
            return _FakePublisher()

        def destroy_node(self):
            record["destroyed"] = True

    def _create_node(name):
        record.setdefault("node_names", []).append(name)
        return _FakeNode()

    fake_rclpy.create_node = _create_node
    fake_rclpy.spin_once = lambda _node, timeout_sec=None: None
    fake_rclpy.spin_until_future_complete = (
        lambda _node, _future, timeout_sec=None: None
    )

    fake_rclpy_action = types.ModuleType("rclpy.action")

    class _FakeActionClient:
        def __init__(self, node, action_type, action_name):
            record["action_name"] = action_name
            record["action_type"] = action_type

        def wait_for_server(self, timeout_sec=None):
            return True

        def send_goal_async(self, goal):
            record["goal"] = goal
            result_wrapper = _Auto()
            result_wrapper.result.error_code.val = 1
            return _FakeFuture(_FakeGoalHandle(result_wrapper))

    fake_rclpy_action.ActionClient = _FakeActionClient

    moveit_action = types.ModuleType("moveit_msgs.action")

    class _MoveGroup:
        Goal = _Auto  # MoveGroup.Goal() -> _Auto()

    moveit_action.MoveGroup = _MoveGroup

    moveit_msg = types.ModuleType("moveit_msgs.msg")
    moveit_msg.Constraints = _Auto
    moveit_msg.MotionPlanRequest = _Auto
    moveit_msg.OrientationConstraint = _Auto
    moveit_msg.PositionConstraint = _Auto

    moveit_pkg = types.ModuleType("moveit_msgs")
    moveit_pkg.action = moveit_action
    moveit_pkg.msg = moveit_msg

    shape_msg = types.ModuleType("shape_msgs.msg")

    class _SolidPrimitive(_Auto):
        SPHERE = 2

    shape_msg.SolidPrimitive = _SolidPrimitive
    shape_pkg = types.ModuleType("shape_msgs")
    shape_pkg.msg = shape_msg

    rosidl_runtime_py = types.ModuleType("rosidl_runtime_py")
    rosidl_runtime_py.message_to_ordereddict = lambda msg: {}
    rosidl_runtime_py.set_message_fields = lambda msg, fields: None

    return {
        "rclpy": fake_rclpy,
        "rclpy.action": fake_rclpy_action,
        "moveit_msgs": moveit_pkg,
        "moveit_msgs.action": moveit_action,
        "moveit_msgs.msg": moveit_msg,
        "shape_msgs": shape_pkg,
        "shape_msgs.msg": shape_msg,
        "rosidl_runtime_py": rosidl_runtime_py,
    }


def _build_executor_fakes():
    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy_qos = types.ModuleType("rclpy.qos")
    fake_rclpy_qos.QoSProfile = _Auto
    fake_rclpy_qos.HistoryPolicy = _Auto
    fake_rclpy_qos.DurabilityPolicy = _Auto

    rosidl_runtime_py = types.ModuleType("rosidl_runtime_py")
    rosidl_runtime_py.message_to_ordereddict = lambda msg: {}
    rosidl_runtime_py.message_to_yaml = lambda msg: "{}"

    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.UInt64 = _Auto
    std_msgs_pkg = types.ModuleType("std_msgs")
    std_msgs_pkg.msg = std_msgs_msg

    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.Empty = _Auto
    std_srvs_pkg = types.ModuleType("std_srvs")
    std_srvs_pkg.srv = std_srvs_srv

    constants = types.ModuleType("constants")
    ros_utils = types.ModuleType("ros_utils")

    return {
        "rclpy": fake_rclpy,
        "rclpy.qos": fake_rclpy_qos,
        "rosidl_runtime_py": rosidl_runtime_py,
        "std_msgs": std_msgs_pkg,
        "std_msgs.msg": std_msgs_msg,
        "std_srvs": std_srvs_pkg,
        "std_srvs.srv": std_srvs_srv,
        "constants": constants,
        "ros_utils": ros_utils,
    }


class MoveItHarnessTests(unittest.TestCase):
    def test_send_command_executes_via_move_action(self):
        record = {}
        fakes = _build_moveit_fakes(record)
        with mock.patch.dict(sys.modules, fakes):
            sys.modules.pop("harness", None)
            harness = importlib.import_module("harness")

            goal_pose = _Auto()
            goal_pose.position.x = 0.4
            goal_pose.position.y = 0.1
            goal_pose.position.z = 0.5
            goal_pose.orientation.x = 0.0
            goal_pose.orientation.y = 0.0
            goal_pose.orientation.z = 0.0
            goal_pose.orientation.w = 1.0

            harness.moveit_send_command(goal_pose)

        self.assertEqual(record["action_name"], "/move_action")
        self.assertFalse(record["goal"].planning_options.plan_only)
        self.assertTrue(record.get("destroyed"))

    def test_send_command_uses_unique_profile_client_node_names(self):
        record = {}
        fakes = _build_moveit_fakes(record)
        with mock.patch.dict(sys.modules, fakes):
            sys.modules.pop("harness", None)
            harness = importlib.import_module("harness")

            goal_pose = _Auto()
            goal_pose.position.x = 0.4
            goal_pose.position.y = 0.1
            goal_pose.position.z = 0.5
            goal_pose.orientation.x = 0.0
            goal_pose.orientation.y = 0.0
            goal_pose.orientation.z = 0.0
            goal_pose.orientation.w = 1.0

            harness.moveit_send_command(goal_pose)
            harness.moveit_send_command(goal_pose)

        self.assertEqual(2, len(record["node_names"]))
        self.assertNotEqual(record["node_names"][0], record["node_names"][1])
        for node_name in record["node_names"]:
            self.assertTrue(
                node_name.startswith("_moveit_profile_client_"),
                node_name,
            )

    def test_wait_for_actions_uses_action_clients(self):
        record = {"clients": [], "lookups": []}
        fake_rclpy = types.ModuleType("rclpy")
        fake_rclpy.ok = lambda: True
        fake_rclpy.spin_once = lambda *_args, **_kwargs: None

        class _FakeNode:
            def destroy_node(self):
                record["destroyed"] = True

        def _create_node(name):
            record["node_name"] = name
            return _FakeNode()

        fake_rclpy.create_node = _create_node

        fake_rclpy_action = types.ModuleType("rclpy.action")

        class _FakeActionClient:
            def __init__(self, _node, action_type, action_name):
                record["clients"].append((action_name, action_type))

            def wait_for_server(self, timeout_sec=None):
                record["timeout_sec"] = timeout_sec
                return True

            def destroy(self):
                record["client_destroyed"] = (
                    record.get("client_destroyed", 0) + 1
                )

        fake_rclpy_action.ActionClient = _FakeActionClient

        utilities = types.ModuleType("rosidl_runtime_py.utilities")

        def _get_action(type_name):
            record["lookups"].append(type_name)
            return f"ACTION<{type_name}>"

        utilities.get_action = _get_action
        rosidl_runtime_py = types.ModuleType("rosidl_runtime_py")
        rosidl_runtime_py.message_to_ordereddict = lambda msg: {}
        rosidl_runtime_py.set_message_fields = lambda msg, fields: None
        rosidl_runtime_py.utilities = utilities

        with mock.patch.dict(
            sys.modules,
            {
                "rclpy": fake_rclpy,
                "rclpy.action": fake_rclpy_action,
                "rosidl_runtime_py": rosidl_runtime_py,
                "rosidl_runtime_py.utilities": utilities,
            },
        ):
            sys.modules.pop("harness", None)
            harness = importlib.import_module("harness")
            ok, graph = harness.wait_for_actions(
                {
                    "/move_action": "moveit_msgs/action/MoveGroup",
                    "/panda_arm_controller/follow_joint_trajectory":
                        "control_msgs/action/FollowJointTrajectory",
                },
                timeout_sec=1,
            )

        self.assertTrue(ok)
        self.assertEqual(
            [
                "moveit_msgs/action/MoveGroup",
                "control_msgs/action/FollowJointTrajectory",
            ],
            record["lookups"],
        )
        self.assertEqual(
            {
                "/move_action": "moveit_msgs/action/MoveGroup",
                "/panda_arm_controller/follow_joint_trajectory":
                    "control_msgs/action/FollowJointTrajectory",
            },
            graph["available"],
        )
        self.assertTrue(record["node_name"].startswith("_robofuzz_action_wait_"))
        self.assertEqual(2, record["client_destroyed"])
        self.assertTrue(record["destroyed"])

    def test_wait_for_log_patterns_only_accepts_fresh_log_entries(self):
        record = {}
        fakes = _build_moveit_fakes(record)
        with mock.patch.dict(sys.modules, fakes):
            sys.modules.pop("harness", None)
            harness = importlib.import_module("harness")

            with tempfile.NamedTemporaryFile("w+", encoding="utf-8") as fp:
                fp.write("You can start planning now!\n")
                fp.flush()
                since_offset = fp.tell()
                fp.write("Configured and activated panda_arm_controller\n")
                fp.flush()

                ok, graph = harness.wait_for_log_patterns(
                    fp.name,
                    [
                        "You can start planning now!",
                        "Configured and activated panda_arm_controller",
                    ],
                    since_offset=since_offset,
                    timeout_sec=0.1,
                    poll_interval=0.01,
                )

        self.assertFalse(ok)
        self.assertEqual(["You can start planning now!"], graph["missing"])
        self.assertIn(
            "Configured and activated panda_arm_controller",
            graph["matched"],
        )

    def test_sweep_moveit_processes_kills_stack_by_name(self):
        record = {}
        fakes = _build_moveit_fakes(record)
        with mock.patch.dict(sys.modules, fakes):
            sys.modules.pop("harness", None)
            harness = importlib.import_module("harness")
            with mock.patch("harness.sp.run") as run:
                harness.sweep_moveit_processes()

        names = [call.args[0][-1] for call in run.call_args_list]
        self.assertIn("rviz2", names)
        self.assertIn("move_group", names)
        self.assertIn("ros2_control_node", names)
        self.assertIn("robot_state_publisher", names)
        self.assertIn("static_transform_publisher", names)

    def test_kill_target_wires_in_moveit_sweep(self):
        with open(os.path.join(SRC_DIR, "fuzzer.py")) as fp:
            text = fp.read()
        self.assertIn("sweep_moveit_processes", text)

    def test_run_target_moveit_supports_visible_rviz(self):
        with open(os.path.join(REPO_ROOT, "run_target.sh")) as fp:
            text = fp.read()
        self.assertIn("MOVEIT_HEADLESS", text)
        self.assertIn("ros2_control_hardware_type:=mock_components", text)

    def test_run_target_moveit_uses_fuzz_rviz_config(self):
        with open(os.path.join(REPO_ROOT, "run_target.sh")) as fp:
            text = fp.read()
        self.assertIn("MOVEIT_RVIZ_CONFIG", text)
        self.assertIn("src_jazzy/rviz/moveit_fuzz.rviz", text)
        self.assertIn("rviz_config:=", text)

    def test_moveit_fuzz_rviz_hides_interactive_goal_state(self):
        rviz_path = os.path.join(SRC_DIR, "rviz", "moveit_fuzz.rviz")
        with open(rviz_path) as fp:
            text = fp.read()
        self.assertIn("Query Goal State: false", text)
        self.assertIn("Goal State Alpha: 0", text)
        self.assertIn("Interactive Marker Size: 0", text)
        self.assertIn("Name: PlanningScene", text)

    def test_executor_starts_rosbag_without_shell_wrapper(self):
        with open(os.path.join(SRC_DIR, "executor.py")) as fp:
            text = fp.read()
        start = text.index("    def start_rosbag")
        end = text.index("    def kill_rosbag")
        start_rosbag = text[start:end]
        self.assertIn("start_new_session=True", start_rosbag)
        self.assertNotIn("shell=True", start_rosbag)
        self.assertIn("self.rosbag_pgrp = None", text)

    def test_executor_cleans_up_if_profile_publish_raises(self):
        events = []
        fakes = _build_executor_fakes()
        with mock.patch.dict(sys.modules, fakes):
            sys.modules.pop("executor", None)
            executor = importlib.import_module("executor")

        class _FakeConfig:
            persistent = False
            replay = True
            rospkg = None
            rosnode = None
            exec_cmd = None

        class _FakeFuzzer:
            config = _FakeConfig()

            def run_target(self, *_args):
                events.append("run_target")

            def kill_target(self):
                events.append("kill_target")

        exec_obj = executor.Executor(_FakeFuzzer())
        exec_obj.start_rosbag = lambda _exec_cnt: events.append("start_rosbag")
        exec_obj.start_watching = lambda: events.append("start_watching")
        exec_obj.stop_watching = lambda: events.append("stop_watching")
        exec_obj.kill_rosbag = lambda: events.append("kill_rosbag")

        def fail_publish(_msg):
            events.append("publish")
            raise RuntimeError("boom")

        with self.assertRaisesRegex(RuntimeError, "boom"):
            exec_obj.execute(
                executor.ExecMode.SINGLE,
                [object()],
                "frame",
                1,
                pub_function=fail_publish,
            )

        self.assertEqual(
            [
                "run_target",
                "start_rosbag",
                "start_watching",
                "publish",
                "stop_watching",
                "kill_rosbag",
                "kill_target",
            ],
            events,
        )

    def test_docker_readme_documents_x11_for_moveit(self):
        with open(
            os.path.join(REPO_ROOT, "docker", "jazzy", "README.md")
        ) as fp:
            text = fp.read()
        self.assertIn("/tmp/.X11-unix", text)
        self.assertIn("xhost", text)

    def test_moveit_oracle_accepts_jazzy_controller_state_fields(self):
        fake_kinpy = types.ModuleType("kinpy")
        fake_kinpy.build_chain_from_urdf = lambda _urdf: None

        with mock.patch.dict(sys.modules, {"kinpy": fake_kinpy}):
            sys.modules.pop("oracles.moveit", None)
            from oracles import moveit

        cfg = types.SimpleNamespace(moveit_planning_only=False)
        point = types.SimpleNamespace(
            positions=[0.0],
            velocities=[0.0],
            accelerations=[0.0],
        )
        controller_state = types.SimpleNamespace(
            joint_names=["panda_joint1"],
            reference=point,
            feedback=point,
            error=point,
        )
        joint_state = types.SimpleNamespace(
            name=["panda_joint1"],
            position=[0.0],
        )
        state_dict = {
            "/joint_states": [(1, joint_state)],
            "/panda_arm_controller/state": [
                (i * 10_000_000, controller_state)
                for i in range(6)
            ],
        }

        errs = moveit.check(cfg, [], state_dict, [])

        self.assertEqual([], errs)


if __name__ == "__main__":
    unittest.main()
