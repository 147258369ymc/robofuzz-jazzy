import os
import sys
import signal
import subprocess as sp
import time
import json
import math
import shlex
import threading

import rclpy
from rosidl_runtime_py import message_to_ordereddict, set_message_fields


class _Px4PromptFilter:
    """Remove PX4 shell prompt redraw spam while preserving normal log lines."""

    _PROMPTS = (
        b"pxh> \x1b[2K\r",
        b"\x1b[2K\rpxh> ",
        b"\x1b[2K\r",
        b"pxh> ",
        b"pxh>",
    )

    def __init__(self):
        self._carry = b""

    @classmethod
    def _strip(cls, data):
        for prompt in cls._PROMPTS:
            data = data.replace(prompt, b"")
        return data

    def filter(self, chunk):
        data = self._carry + (chunk or b"")
        keep = max(len(prompt) for prompt in self._PROMPTS) - 1
        if len(data) <= keep:
            self._carry = data
            return b""
        body = data[:-keep]
        self._carry = data[-keep:]
        return self._strip(body)

    def flush(self):
        tail = self._strip(self._carry)
        self._carry = b""
        return tail


def _pump_stream_to_file(stream, fp, log_filter=None):
    try:
        while True:
            chunk = stream.read(8192)
            if not chunk:
                break
            if log_filter is not None:
                chunk = log_filter.filter(chunk)
            if chunk:
                fp.write(chunk)
                fp.flush()
        if log_filter is not None:
            chunk = log_filter.flush()
            if chunk:
                fp.write(chunk)
                fp.flush()
    except (OSError, ValueError):
        pass


def run_target_profile(profile, proj_dir, log_dir=None):
    cmd_parts = [os.path.join(proj_dir, "run_target.sh"), profile.name]

    cmd = " ".join(shlex.quote(str(part)) for part in cmd_parts)
    run_cmd_dir = log_dir if log_dir else os.path.join(proj_dir, "logs")
    run_cmd_path = os.path.join(run_cmd_dir, "last_profile_run_cmd")
    try:
        os.makedirs(os.path.dirname(run_cmd_path), exist_ok=True)
        with open(run_cmd_path, "w") as fp:
            fp.write(cmd)
    except OSError:
        pass

    print(f"[*] Starting target profile {profile.name}: {cmd}")
    stdout = sys.stdout
    stderr = sys.stderr
    log_handles = None
    log_threads = ()
    stdout_filter = None
    if log_dir:
        try:
            os.makedirs(log_dir, exist_ok=True)
            stdout_file = open(os.path.join(log_dir, "target.stdout.log"), "ab")
            stderr = open(os.path.join(log_dir, "target.stderr.log"), "ab")
            log_handles = (stdout_file, stderr)
            if getattr(profile, "family", "") == "px4":
                stdout = sp.PIPE
                stdout_filter = _Px4PromptFilter()
            else:
                stdout = stdout_file
        except OSError:
            stdout = sys.stdout
            stderr = sys.stderr
            log_handles = None

    proc = sp.Popen(
        cmd,
        shell=True,
        cwd=proj_dir,
        preexec_fn=os.setpgrp,
        stdin=sp.DEVNULL,
        stdout=stdout,
        stderr=stderr,
    )
    if stdout_filter is not None and proc.stdout is not None:
        thread = threading.Thread(
            target=_pump_stream_to_file,
            args=(proc.stdout, log_handles[0], stdout_filter),
            daemon=True,
        )
        thread.start()
        log_threads = (thread,)
    proc._robofuzz_log_handles = log_handles
    proc._robofuzz_log_threads = log_threads
    return proc


def wait_for_topics(topic_map, timeout_sec=120, include_types=True):
    if not topic_map:
        return True, {}

    deadline = time.time() + timeout_sec
    expected = dict(topic_map)
    seen = {}

    while time.time() < deadline:
        seen = get_topic_type_map()
        missing = []
        mismatched = []
        for topic, msg_type in expected.items():
            if topic not in seen:
                missing.append(topic)
                continue
            if include_types and msg_type and seen[topic] != msg_type:
                mismatched.append((topic, msg_type, seen[topic]))

        if not missing and not mismatched:
            return True, seen

        if missing:
            print("[topic wait] missing:", ", ".join(missing))
        if mismatched:
            for topic, expected_type, actual_type in mismatched:
                print(
                    "[topic wait] type mismatch:",
                    topic,
                    "expected",
                    expected_type,
                    "actual",
                    actual_type,
                )
        time.sleep(2)

    return False, seen


def wait_for_actions(action_map, timeout_sec=120):
    if not action_map:
        return True, {"expected": {}, "available": {}, "missing": []}

    from rclpy.action import ActionClient
    from rosidl_runtime_py.utilities import get_action

    expected = dict(action_map)
    graph = {
        "expected": expected,
        "available": {},
        "missing": sorted(expected),
        "errors": {},
    }
    created_context = False
    if not rclpy.ok():
        rclpy.init(args=None)
        created_context = True

    node_name = f"_robofuzz_action_wait_{os.getpid()}_{time.time_ns()}"
    node = rclpy.create_node(node_name)
    clients = {}
    try:
        for action_name, type_name in expected.items():
            try:
                action_type = get_action(type_name)
            except Exception as exc:
                graph["errors"][action_name] = str(exc)
                return False, graph
            clients[action_name] = ActionClient(node, action_type, action_name)

        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            missing = []
            available = {}
            for action_name, client in clients.items():
                if client.wait_for_server(timeout_sec=0.2):
                    available[action_name] = expected[action_name]
                else:
                    missing.append(action_name)

            graph["available"] = available
            graph["missing"] = sorted(missing)
            if not missing:
                return True, graph

            print("[action wait] missing:", ", ".join(sorted(missing)))
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(1)

        return False, graph
    finally:
        for client in clients.values():
            try:
                client.destroy()
            except Exception:
                pass
        try:
            node.destroy_node()
        finally:
            if created_context:
                rclpy.shutdown()


def wait_for_log_patterns(
    log_path,
    patterns,
    since_offset=0,
    timeout_sec=120,
    poll_interval=0.5,
):
    expected = list(patterns or [])
    graph = {
        "log_path": log_path,
        "since_offset": since_offset,
        "expected": expected,
        "matched": [],
        "missing": list(expected),
        "last_size": 0,
        "errors": {},
    }
    if not expected:
        graph["missing"] = []
        return True, graph

    deadline = time.time() + timeout_sec
    last_report = 0
    while time.time() < deadline:
        try:
            with open(log_path, "r", encoding="utf-8", errors="ignore") as fp:
                fp.seek(since_offset)
                content = fp.read()
                graph["last_size"] = fp.tell()
        except OSError as exc:
            graph["errors"]["read"] = str(exc)
            content = ""

        matched = [pattern for pattern in expected if pattern in content]
        missing = [pattern for pattern in expected if pattern not in content]
        graph["matched"] = matched
        graph["missing"] = missing
        if not missing:
            return True, graph

        now = time.time()
        if now - last_report >= 2.0:
            print("[log wait] missing:", ", ".join(missing))
            last_report = now
        time.sleep(poll_interval)

    return False, graph


def wait_for_topic_data(topic_map, timeout_sec=120, min_msgs=1):
    """Wait until each topic is not just advertised but actually publishing.

    ``topic_map`` maps topic name -> message type string (e.g.
    ``"nav_msgs/msg/Odometry"``). A topic is considered ready once at least
    ``min_msgs`` messages are received on it within the timeout. This guards
    against readiness gates that pass on mere topic presence while the
    underlying producer (e.g. a ros2_control controller still spawning) has
    not begun streaming data yet.

    Returns ``(ok, graph)`` mirroring the other ``wait_for_*`` helpers.
    """
    expected = dict(topic_map or {})
    graph = {
        "expected": expected,
        "received": {},
        "missing": sorted(expected),
        "errors": {},
    }
    if not expected:
        graph["missing"] = []
        return True, graph

    from rosidl_runtime_py.utilities import get_message

    created_context = False
    if not rclpy.ok():
        rclpy.init(args=None)
        created_context = True

    node_name = f"_robofuzz_data_wait_{os.getpid()}_{time.time_ns()}"
    node = rclpy.create_node(node_name)
    counts = {topic: 0 for topic in expected}
    subs = []

    def _make_cb(topic):
        def _cb(_msg):
            counts[topic] += 1
        return _cb

    try:
        from rclpy.qos import (
            QoSProfile,
            QoSReliabilityPolicy,
            QoSHistoryPolicy,
            QoSDurabilityPolicy,
        )

        default_qos = QoSProfile(depth=10)
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        transient_local_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        def qos_candidates(topic):
            if topic == "/sim_ground_truth_pose":
                return [best_effort_qos]
            # PX4 v1.17 uXRCE-DDS state streams publish best-effort. Creating
            # reliable readiness subscriptions for /fmu/out/* receives no data
            # and spams reliability-incompatible warnings every fuzz round.
            if topic.startswith("/fmu/out/"):
                return [best_effort_qos]
            # Other readiness topics come from different ROS 2 QoS worlds.
            # TurtleBot4 /odom can be reliable + transient-local, while sensor
            # streams are often best-effort. Try a compact compatibility set.
            return [default_qos, best_effort_qos, transient_local_qos]

        for topic, type_name in expected.items():
            try:
                msg_class = get_message(type_name)
            except Exception as exc:
                graph["errors"][topic] = str(exc)
                return False, graph
            for qos in qos_candidates(topic):
                subs.append(
                    node.create_subscription(
                        msg_class, topic, _make_cb(topic), qos
                    )
                )

        deadline = time.time() + timeout_sec
        last_report = 0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            received = {t: c for t, c in counts.items() if c >= min_msgs}
            missing = sorted(t for t in expected if counts[t] < min_msgs)
            graph["received"] = dict(received)
            graph["missing"] = missing
            if not missing:
                return True, graph

            now = time.time()
            if now - last_report >= 2.0:
                print("[data wait] no data yet on:", ", ".join(missing))
                last_report = now

        return False, graph
    finally:
        for sub in subs:
            try:
                node.destroy_subscription(sub)
            except Exception:
                pass
        try:
            node.destroy_node()
        finally:
            if created_context:
                rclpy.shutdown()


def get_topic_type_map():
    try:
        proc = sp.run(
            ["ros2", "topic", "list", "-t", "--include-hidden-topics"],
            check=False,
            stdout=sp.PIPE,
            stderr=sp.PIPE,
            text=True,
            timeout=10,
        )
    except (OSError, sp.TimeoutExpired):
        return {}

    topics = {}
    for line in proc.stdout.splitlines():
        line = line.strip()
        if not line or "[" not in line or "]" not in line:
            continue
        topic, type_part = line.rsplit("[", 1)
        topics[topic.strip()] = type_part.rstrip("]").strip()
    return topics


def run_px4_stack_proc():
    px4_root = "/home/seulbae/workspace/ros-security/targets/PX4-Autopilot"

    model = "iris"
    world = "fuzzing"
    bin_server = "gzserver"
    opt_server = os.path.join(
        px4_root, f"Tools/sitl_gazebo/worlds/{world}.world"
    )
    cmd1 = f"{bin_server} {opt_server}"

    bin_model = "gz model"
    opt_model = "--spawn-file={} --model-name={} -x {} -y {} -z {}".format(
        os.path.join(
            px4_root, f"Tools/sitl_gazebo/models/{model}/{model}.sdf"
        ),
        f"{model}",
        "1.01",
        "0.98",
        "0.83",
    )
    cmd2 = f"{bin_model} {opt_model}"

    env_client = os.environ.copy()
    env_client["GAZEBO_PLUGIN_PATH"] = os.path.join(
        px4_root, "build/px4_sitl_rtps/build_gazebo"
    )
    env_client["GAZEBO_MODEL_PATH"] = os.path.join(
        px4_root, "Tools/sitl_gazebo/models"
    )
    env_client["LD_LIBRARY_PATH"] = os.path.join(
        px4_root, "build/px4_sitl_rtps/build_gazebo"
    )
    bin_client = "gzclient"
    opt_client = "--gui-client-plugin libgazebo_user_camera_plugin.so"
    cmd3 = f"{bin_client} {opt_client}"

    env_px4 = os.environ.copy()
    env_px4["PX4_SIM_MODEL"] = "iris"
    bin_px4 = os.path.join(px4_root, "build/px4_sitl_rtps/bin/px4")
    etc_dir = os.path.join(px4_root, "build/px4_sitl_rtps", "etc")
    rcs = os.path.join(px4_root, "build/px4_sitl_rtps/etc/init.d-posix/rcS")
    data_dir = os.path.join(px4_root, "test_data")
    opt_px4 = f"{etc_dir} -s {rcs} -t {data_dir}"

    cwd = os.getcwd()
    wd = os.path.join(px4_root, "build/px4_sitl_rtps/tmp/rootfs")
    os.chdir(wd)
    cmd4 = f"{bin_px4} {opt_px4}"

    print(cmd1)
    pgrp1 = sp.Popen(cmd1, shell=True, preexec_fn=os.setpgrp)
    time.sleep(5)

    print(cmd2)
    pgrp2 = sp.Popen(cmd2, shell=True, preexec_fn=os.setpgrp)
    time.sleep(5)

    print(cmd3)
    pgrp3 = sp.Popen(cmd3, shell=True, preexec_fn=os.setpgrp, env=env_client)
    time.sleep(5)

    print(cmd4)
    pgrp4 = sp.Popen(cmd4, shell=True, preexec_fn=os.setpgrp, env=env_px4)
    time.sleep(5)

    os.killpg(pgrp1.pid, signal.SIGKILL)
    os.killpg(pgrp2.pid, signal.SIGKILL)
    os.killpg(pgrp3.pid, signal.SIGKILL)
    os.killpg(pgrp4.pid, signal.SIGKILL)


def run_px4_stack_sh(proj_dir):
    px4_dir = os.path.join(proj_dir, "targets", "PX4-Autopilot")
    # use below if measuring coverage:
    # px4_dir = "/home/seulbae/workspace/px4-cov"
    devnull = "2>&1 > /dev/null"

    simulator = "gazebo"
    model = "iris"

    cmd_list = [
        "PX4_SITL_WORLD=empty",
        os.path.join(px4_dir, "Tools", "sitl_run.sh"),
        os.path.join(px4_dir, "build", "px4_sitl_rtps", "bin", "px4"),
        "none",
        simulator,
        model,
        "none",
        px4_dir,
        os.path.join(px4_dir, "build", "px4_sitl_rtps"),
        devnull,
    ]
    cmd = " ".join(cmd_list)
    # print("command:", cmd)

    proc = sp.Popen(cmd, shell=True, stdout=sp.DEVNULL, stderr=sp.DEVNULL)
    return proc


def run_tb3_sitl(proj_dir):
    tb3_dir = os.path.join(proj_dir, "targets", "turtlebot3_ws")
    devnull = "2>&1 > /dev/null"

    ros_pkg = "turtlebot3_gazebo"
    sim_map = "empty_world.launch.py"

    cmd_list = [
        f"DISPLAY={os.getenv('DISPLAY')}",
        "TURTLEBOT3_MODEL=burger",
        "ros2",
        "launch",
        ros_pkg,
        sim_map,
    ]
    cmd = " ".join(cmd_list)
    # print("command:", cmd)

    pgrp = sp.Popen(
        cmd,
        shell=True,
        preexec_fn=os.setpgrp,
        stdout=sp.DEVNULL,
        stderr=sp.DEVNULL,
    )
    return pgrp


def run_tb3_hitl(tb3_uri):
    cmd = f"ssh -i keys/tb3 {tb3_uri} ./run.sh"

    proc = sp.Popen(cmd, shell=True, stdout=sp.DEVNULL, stderr=sp.DEVNULL)
    return proc


def run_rcl_api_harness(features, targets, job):
    feature_str = " ".join(features)
    target_str = " ".join(targets)

    cmd = f"python3 rcl_harness.py -f {feature_str} -t {target_str} -j {job}"

    pgrp = sp.Popen(
        cmd,
        shell=True,
        preexec_fn=os.setpgrp,
        stdout=sys.stdout,
        stderr=sys.stderr,
    )

    return pgrp


def run_cli_harness():
    cmd = "python3 cli_harness.py"

    pgrp = sp.Popen(
        cmd,
        shell=True,
        preexec_fn=os.setpgrp,
        stdout=sys.stdout,
        stderr=sys.stderr,
    )

    return pgrp


def run_rosidl_harness(lang, shmid, ros_type="empty"):
    # lang, shm, ros_type
    # need shm for both lang targets as they need to fetch data from shm

    # if lang == "py":
        # cmd = ""
    # elif lang == "cpp":
        # cmd = ""

    cmd = "ros2 run idltest_target idltest_target"

    pgrp = sp.Popen(
        cmd,
        shell=True,
        preexec_fn=os.setpgrp,
        stdout=sys.stdout,
        stderr=sys.stderr,
    )

    return pgrp


def run_moveit_harness():
    cmd = f"DISPLAY={os.getenv('DISPLAY')} ros2 launch moveit2_tutorials move_group.launch.py 2>&1 > /dev/null"

    pgrp = sp.Popen(
        cmd,
        shell=True,
        preexec_fn=os.setpgrp,
        stdout=sys.stdout,
        stderr=sys.stderr,
    )

    return pgrp


def get_init_moveit_msg():
    from moveit_msgs.msg import MotionPlanRequest

    # initial (ready) position
    with open("moveit_panda_msg.json", "r") as f:
        msg_json = json.load(f)

    msg = MotionPlanRequest()
    set_message_fields(msg, msg_json)

    # re-assign timestamp
    # ts = time.time_ns()
    # sec = int(ts / pow(10, 9))
    # nsec = ts - sec * pow(10, 9)
    # msg.workspace_parameters.header.stamp.sec = sec
    # msg.workspace_parameters.header.stamp.nanosec = nsec

    return msg


def get_init_joint_constraints():
    msg = get_init_moveit_msg()
    joint_constraints = msg["goal_constraints"][0]["joint_constraints"]
    return joint_constraints


def get_init_moveit_pose():
    from geometry_msgs.msg import Pose
    msg = Pose()
    # msg.position.x = 0.28
    # msg.position.y = -0.2
    # msg.position.z = 0.5
    # msg.orientation.w = 1.0
    msg.position.x = 0.5
    msg.position.y = 0.5
    msg.position.z = 0.5
    msg.orientation.w = 1.0

    return msg

def moveit_send_command(msg, plan_params=None):
    print("[moveit harness] sending goal command (execute)")
    from moveit_msgs.action import MoveGroup
    from rclpy.action import ActionClient
    from std_msgs.msg import Int32, String
    from moveit_plan_params import normalize_plan_params, plan_params_to_json

    node_name = f"_moveit_profile_client_{os.getpid()}_{time.time_ns()}"
    node = rclpy.create_node(node_name)
    try:
        normalized_plan_params = normalize_plan_params(plan_params)
        request = _build_panda_motion_plan_request(
            msg,
            plan_params=normalized_plan_params,
        )

        # move_group does not emit /motion_plan_request for action goals, but
        # the oracle uses the last request as endpoint ground truth.
        request_pub = node.create_publisher(
            type(request),
            "/motion_plan_request",
            10,
        )
        result_code_pub = node.create_publisher(
            Int32,
            "/robofuzz/moveit_result_code",
            10,
        )
        goal_event_pub = node.create_publisher(
            String,
            "/robofuzz/moveit_goal_event",
            10,
        )
        plan_params_pub = node.create_publisher(
            String,
            "/robofuzz/moveit_plan_params",
            10,
        )
        plan_params_msg = String()
        plan_params_msg.data = plan_params_to_json(normalized_plan_params)
        for _ in range(3):
            request_pub.publish(request)
            plan_params_pub.publish(plan_params_msg)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)

        client = ActionClient(node, MoveGroup, "/move_action")
        if not client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("MoveIt /move_action server unavailable")

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=30.0)
        goal_handle = send_future.result()
        if goal_handle is None:
            raise RuntimeError("MoveIt goal request returned no goal handle")
        if not goal_handle.accepted:
            raise RuntimeError("MoveIt goal rejected by move_group")

        result_future = goal_handle.get_result_async()
        action_timeout = float(os.environ.get("MOVEIT_ACTION_TIMEOUT_SEC", "60.0"))
        rclpy.spin_until_future_complete(
            node,
            result_future,
            timeout_sec=action_timeout,
        )
        result_wrapper = result_future.result()
        if result_wrapper is None:
            try:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(
                    node,
                    cancel_future,
                    timeout_sec=2.0,
                )
            except Exception:
                pass
            timeout_msg = Int32()
            timeout_msg.data = -6
            result_code_pub.publish(timeout_msg)
            event_msg = String()
            event_msg.data = f"timeout:-6:{action_timeout}"
            goal_event_pub.publish(event_msg)
            raise RuntimeError("MoveIt execution timed out")

        err_val = getattr(result_wrapper.result.error_code, "val", None)
        result_code_msg = Int32()
        result_code_msg.data = int(err_val) if err_val is not None else 0
        result_code_pub.publish(result_code_msg)
        event_msg = String()
        event_msg.data = f"result:{result_code_msg.data}"
        goal_event_pub.publish(event_msg)
        print(f"                 + executed (error_code={err_val})")
    finally:
        node.destroy_node()


def _build_panda_motion_plan_request(goal_pose, plan_params=None):
    from moveit_plan_params import normalize_plan_params
    from moveit_msgs.msg import (
        Constraints,
        MotionPlanRequest,
        OrientationConstraint,
        PositionConstraint,
    )
    from shape_msgs.msg import SolidPrimitive

    plan_params = normalize_plan_params(plan_params)

    request = MotionPlanRequest()
    request.workspace_parameters.header.frame_id = "panda_link0"
    request.workspace_parameters.min_corner.x = -1.0
    request.workspace_parameters.min_corner.y = -1.0
    request.workspace_parameters.min_corner.z = -1.0
    request.workspace_parameters.max_corner.x = 1.0
    request.workspace_parameters.max_corner.y = 1.0
    request.workspace_parameters.max_corner.z = 1.5
    request.start_state.is_diff = True
    request.group_name = "panda_arm"
    request.num_planning_attempts = 5
    request.allowed_planning_time = plan_params["planning_time"]
    request.max_velocity_scaling_factor = plan_params["velocity_scaling"]
    request.max_acceleration_scaling_factor = plan_params[
        "acceleration_scaling"]

    region = SolidPrimitive()
    region.type = SolidPrimitive.SPHERE
    region.dimensions = [plan_params["position_tolerance"]]

    position = PositionConstraint()
    position.header.frame_id = "panda_link0"
    position.link_name = "panda_hand"
    position.constraint_region.primitives = [region]
    position.constraint_region.primitive_poses = [goal_pose]
    position.weight = 1.0

    orientation = OrientationConstraint()
    orientation.header.frame_id = "panda_link0"
    orientation.link_name = "panda_hand"
    orientation.orientation = goal_pose.orientation
    orientation.absolute_x_axis_tolerance = plan_params[
        "orientation_tolerance"]
    orientation.absolute_y_axis_tolerance = plan_params[
        "orientation_tolerance"]
    orientation.absolute_z_axis_tolerance = plan_params[
        "orientation_tolerance"]
    orientation.weight = 1.0

    constraints = Constraints()
    constraints.position_constraints = [position]
    constraints.orientation_constraints = [orientation]
    request.goal_constraints = [constraints]

    return request


def _kill_process_group(proc):
    """Kill the whole process group of a `ros2 launch` invocation, then sweep
    any stragglers from the move_group stack that escaped the group."""
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGKILL)
    except (ProcessLookupError, PermissionError):
        pass
    try:
        proc.wait(timeout=5)
    except sp.TimeoutExpired:
        pass
    # Safety net: ros2 launch sometimes daemonizes children into new sessions,
    # so a group kill can miss them. Sweep the known stack by name.
    sp.run(
        ["pkill", "-9", "-f", "move_group_interface_tutorial"],
        stdout=sp.DEVNULL, stderr=sp.DEVNULL,
    )


def sweep_moveit_processes():
    """Best-effort cleanup for MoveIt/RViz processes that escaped launch PGID."""
    for name in (
        "rviz2",
        "move_group",
        "ros2_control_node",
        "robot_state_publisher",
        "static_transform_publisher",
    ):
        sp.run(
            ["pkill", "-9", "-f", name],
            stdout=sp.DEVNULL,
            stderr=sp.DEVNULL,
        )


def sweep_turtlebot_processes():
    """Best-effort cleanup for TurtleBot/Gazebo processes that escaped launch PGID.

    Needed for GUI mode multi-round stability: gz sim gui processes can survive
    killpg() and interfere with subsequent rounds. This ensures clean slate.
    """
    for name in (
        "gz sim",
        "ruby.*gz",
        "ign gazebo",
        "gzserver",
        "gzclient",
        "turtlebot4_node",
        "turtlebot4_gz_hmi_node",
        "motion_control",
        "controller_manager",
        "spawner_.*controller",
        "component_container",
        "robot_state_publisher",
        "static_transform_publisher",
        "ros_gz_bridge",
        "parameter_bridge",
    ):
        sp.run(
            ["pkill", "-9", "-f", name],
            stdout=sp.DEVNULL,
            stderr=sp.DEVNULL,
        )


def _nav2_resolve_goal(msg, map_ctx, start_xy=(0.0, 0.0)):
    """Read (x,y,yaw) from a mutated PoseStamped; if a map is given and the
    goal is not reachable from start, snap to the nearest reachable free cell.
    Returns (x, y, yaw, snapped)."""
    import math as _m
    pose = getattr(msg, "pose", None)
    inner = getattr(pose, "pose", None)
    if inner is not None and hasattr(inner, "position"):
        pose = inner
    px = float(pose.position.x)
    py = float(pose.position.y)
    q = pose.orientation
    yaw = _m.atan2(2.0 * (q.w * q.z + q.x * q.y),
                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    if map_ctx is None:
        return px, py, yaw, False
    if map_ctx.reachable(start_xy, (px, py)):
        return px, py, yaw, False
    col, row = map_ctx.world_to_cell(px, py)
    nearest = map_ctx.nearest_free_cell(col, row)
    if nearest is None:
        return px, py, yaw, False
    nx, ny = map_ctx.cell_to_world(*nearest)
    return nx, ny, yaw, True


def nav2_send_command(msg, map_ctx=None, start_xy=(0.0, 0.0)):
    """Send a NavigateToPose goal (mirrors moveit_send_command). Republishes
    the actual goal + terminal result + events onto /robofuzz/nav2_* so the
    rosbag captures them and oracles/nav2.py can judge them offline."""
    print("[nav2 harness] sending NavigateToPose goal")
    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Int32, String
    from rclpy.action import ActionClient

    node = rclpy.create_node(
        f"_nav2_profile_client_{os.getpid()}_{time.time_ns()}")
    try:
        gx, gy, gyaw, snapped = _nav2_resolve_goal(msg, map_ctx, start_xy)

        # goal_pub must use TRANSIENT_LOCAL (latched) so the rosbag (which
        # subscribes slightly after the publish at t~0) receives the goal. With
        # VOLATILE (default), the three quick publishes finish before rosbag
        # subscribes, leaving /robofuzz/nav2_goal Count=0 in the bag -> oracle
        # has no goal to judge arrival against.
        from rclpy.qos import QoSProfile, DurabilityPolicy
        latched_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        goal_pub = node.create_publisher(PoseStamped, "/robofuzz/nav2_goal", latched_qos)
        result_pub = node.create_publisher(Int32, "/robofuzz/nav2_result", 10)
        event_pub = node.create_publisher(String, "/robofuzz/nav2_goal_event", 10)

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = gx
        goal_msg.pose.position.y = gy
        goal_msg.pose.orientation.z = math.sin(gyaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(gyaw / 2.0)
        for _ in range(3):
            goal_pub.publish(goal_msg)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        if snapped:
            ev = String(); ev.data = f"snapped:{gx:.3f},{gy:.3f}"
            event_pub.publish(ev)

        client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
        if not client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("nav2 /navigate_to_pose server unavailable")

        goal = NavigateToPose.Goal()
        goal.pose = goal_msg
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=30.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("nav2 goal rejected")

        # Default 300s: the gz Harmonic + full-sensor TB4 sim is CPU-bound and
        # runs well below real time (the nav2 controller logs "Control loop
        # missed its desired rate"), so a far goal (~6-7 m across slam_arena)
        # legitimately needs more than 120s of wall time even while navigating
        # correctly. Smoke confirmed nav2 plans + drives fine; the old 120s
        # budget just cancelled mid-navigation. Override via NAV2_ACTION_TIMEOUT_SEC.
        timeout = float(os.environ.get("NAV2_ACTION_TIMEOUT_SEC", "300.0"))
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout)
        wrapper = result_future.result()
        code = Int32()
        if wrapper is None:
            try:
                cf = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cf, timeout_sec=2.0)
            except Exception:
                pass
            code.data = -6  # TIMEOUT
            result_pub.publish(code)
            ev = String(); ev.data = f"timeout:{timeout}"
            event_pub.publish(ev)
            raise RuntimeError("nav2 navigation timed out")
        code.data = int(getattr(wrapper, "status", 0))
        result_pub.publish(code)
        ev = String(); ev.data = f"result:{code.data}"
        event_pub.publish(ev)
        for _ in range(3):
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        print(f"                 + nav2 result status={code.data}")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    # run_px4_stack_sh()
    # pgrp = run_tb3_sitl(os.path.join(os.getcwd(), "../"))
    pgrp = run_moveit2_harness()
    print("pgroup pid:", pgrp.pid)
    time.sleep(20)
    try:
        os.killpg(pgrp.pid, signal.SIGKILL)
    except:
        print("err")
