import os
import sys
import signal
import subprocess as sp
import time
import json
import shlex

import rclpy
from rosidl_runtime_py import message_to_ordereddict, set_message_fields


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
    if log_dir:
        try:
            os.makedirs(log_dir, exist_ok=True)
            stdout = open(os.path.join(log_dir, "target.stdout.log"), "ab")
            stderr = open(os.path.join(log_dir, "target.stderr.log"), "ab")
            log_handles = (stdout, stderr)
        except OSError:
            stdout = sys.stdout
            stderr = sys.stderr
            log_handles = None

    proc = sp.Popen(
        cmd,
        shell=True,
        cwd=proj_dir,
        preexec_fn=os.setpgrp,
        stdout=stdout,
        stderr=stderr,
    )
    proc._robofuzz_log_handles = log_handles
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

def moveit_send_command(msg):
    print("[moveit harness] sending goal command (execute)")
    from moveit_msgs.action import MoveGroup
    from rclpy.action import ActionClient

    node_name = f"_moveit_profile_client_{os.getpid()}_{time.time_ns()}"
    node = rclpy.create_node(node_name)
    try:
        request = _build_panda_motion_plan_request(msg)

        # move_group does not emit /motion_plan_request for action goals, but
        # the oracle uses the last request as endpoint ground truth.
        request_pub = node.create_publisher(
            type(request),
            "/motion_plan_request",
            10,
        )
        for _ in range(3):
            request_pub.publish(request)
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
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=60.0)
        result_wrapper = result_future.result()
        if result_wrapper is None:
            raise RuntimeError("MoveIt execution timed out")

        err_val = getattr(result_wrapper.result.error_code, "val", None)
        print(f"                 + executed (error_code={err_val})")
    finally:
        node.destroy_node()


def _build_panda_motion_plan_request(goal_pose):
    from moveit_msgs.msg import (
        Constraints,
        MotionPlanRequest,
        OrientationConstraint,
        PositionConstraint,
    )
    from shape_msgs.msg import SolidPrimitive

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
    request.allowed_planning_time = 5.0
    request.max_velocity_scaling_factor = 0.1
    request.max_acceleration_scaling_factor = 0.1

    region = SolidPrimitive()
    region.type = SolidPrimitive.SPHERE
    region.dimensions = [0.01]

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
    orientation.absolute_x_axis_tolerance = 0.5
    orientation.absolute_y_axis_tolerance = 0.5
    orientation.absolute_z_axis_tolerance = 0.5
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
