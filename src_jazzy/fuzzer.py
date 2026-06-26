#!/usr/bin/python3

from collections import defaultdict
import sched

import time
import os
import sys
import traceback
from functools import partial, reduce
import random
import struct
import pickle
import argparse
from datetime import datetime
import asyncio
import math
import subprocess as sp
import signal
import pprint
import importlib
from collections import deque  # kept for non-PX4 targets
from copy import deepcopy
import json
import shutil
import glob

import numpy as np
import sysv_ipc as ipc

import constants as c
import config
import ros_utils
import mutator
import harness
import checker
import target_profiles
import moveit_feedback_buckets
from rosbag_parser import RosbagParser
from ulg_parser import UlgStateParser, find_latest_ulg, cleanup_ulg_files, preserve_ulg_on_error
from checker import StateMonitorNode
from executor import Executor, ExecMode
from scheduler import Scheduler, Campaign
from feedback import Feedback, FeedbackType

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from ros2node.api import *

# from ros2msg.api import get_all_message_types, get_message_path
# from ros2srv.api import get_all_service_types
try:
    from rqt_graph.rosgraph2_impl import Graph
except ImportError:
    Graph = None
from ros2topic.api import get_msg_class
from rosidl_runtime_py import message_to_ordereddict, set_message_fields

from ros2_fuzzer import ros_commons
from ros2_fuzzer.process_handling import FuzzedNodeHandler

from std_msgs.msg import Bool, String
from tracer import APITracer
try:
    from turtlesim.msg import Pose as TurtleSimPose
except ImportError:
    TurtleSimPose = None


def safe_rclpy_shutdown():
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception as e:
        print(f"[-] rclpy shutdown skipped: {e}")


def moveit_error_signature(errs):
    """Collapse a list of MoveIt oracle error strings into a set of coarse
    signatures, so repeated discoveries of the SAME bug (e.g. joint1 velocity
    overshoot, over and over) collapse to one signature.

    Signature drops timestamps and exact magnitudes, keeping only:
      - the violation TYPE (velocity / acceleration / position / deviation / ...)
      - the joint name (if present)
      - a coarse magnitude bucket for numeric violations
    """
    import re
    sigs = set()
    for e in errs:
        e = str(e)
        joint_m = re.search(r"panda_\w+", e)
        joint = joint_m.group(0) if joint_m else ""
        if "TOPP-RA planned velocity" in e:
            num = re.search(r":\s*([0-9.]+)\s*>", e)
            bucket = int(float(num.group(1)) / 0.25) if num else 0
            sigs.add(("vel", joint, bucket))
        elif "planned acceleration" in e:
            num = re.search(r":\s*([0-9.]+)\s*>", e)
            bucket = int(float(num.group(1)) / 1.0) if num else 0
            sigs.add(("acc", joint, bucket))
        elif "deviation too high" in e:
            num = re.search(r":\s*([0-9.]+)", e)
            bucket = int(float(num.group(1)) / 0.25) if num else 0
            sigs.add(("deviation", "", bucket))
        elif "success_but_endpoint_outlier" in e:
            num = re.search(r"([0-9.]+)m", e)
            bucket = int(float(num.group(1)) / 0.01) if num else 0
            sigs.add(("endpoint_outlier", "", bucket))
        elif "oracle_ir_numeric_violation" in e:
            sigs.add(("oracle_ir_numeric", joint, 0))
        elif "execution_state_missing" in e:
            sigs.add(("execution_state_missing", "", 0))
        elif "unexpected_planning_rejection" in e:
            sigs.add(("reachable_rejection", "", 0))
        elif "reachable_failure_candidate" in e:
            code = re.search(r"error_code=(-?[0-9]+)", e)
            bucket = int(code.group(1)) if code else 0
            sigs.add(("reachable_failure", "", bucket))
        elif "result_status_inconsistency" in e:
            code = re.search(r"error_code=(-?[0-9]+)", e)
            bucket = int(code.group(1)) if code else 0
            sigs.add(("result_status_inconsistency", "", bucket))
        elif "trajectory_smoothness_violation_candidate" in e:
            num = re.search(r"ratio\s+([0-9.]+)", e)
            bucket = int(float(num.group(1)) / 0.25) if num else 0
            sigs.add(("smoothness_candidate", "", bucket))
        elif "scaling_violation" in e:
            kind = "acceleration" if "acceleration_scaling" in e else "velocity"
            num = re.search(r"desired_(?:vel|acc)_ratio=([0-9.]+)", e)
            bucket = int(float(num.group(1)) / 0.1) if num else 0
            sigs.add(("scaling_violation", kind, bucket))
        elif "action_status_anomaly" in e:
            sigs.add(("status_anomaly", "", 0))
        elif "outside [" in e or "position" in e and "NaN" in e:
            sigs.add(("position", joint, 0))
        elif "tracking error" in e:
            sigs.add(("tracking", joint, 0))
        elif "abort drift" in e:
            sigs.add(("abort_drift", "", 0))
        elif "unexpected terminal status" in e:
            sigs.add(("status_anomaly", "", 0))
        else:
            sigs.add(("other", "", hash(e) % 1000))
    return sigs


class SeedQueue:
    """Quality-aware seed queue with deduplication, staleness decay, and warmup.

    Seeds stay in the pool until they become "stale" (selected MAX_SELECTIONS
    times). Weighted random selection favors fresher seeds. During warmup,
    each initial seed is guaranteed at least one selection before random
    selection begins.
    """

    MAX_SIZE = 50
    MAX_SELECTIONS = 5

    def __init__(self):
        self._items = []
        self._select_count = []
        self._warmup_done = False
        self._initial_count = 0

    def append(self, seed, is_readd=False):
        """Add seed. Duplicates are silently rejected on re-add."""
        if is_readd and self._is_duplicate(seed):
            return
        if len(self._items) >= self.MAX_SIZE:
            self._items.pop(0)
            self._select_count.pop(0)
        self._items.append(seed)
        self._select_count.append(0)
        if not is_readd:
            self._initial_count += 1

    def popleft(self):
        """Weighted random selection, favoring fresher seeds."""
        if not self._items:
            raise IndexError("pop from empty queue")
        # Warmup: rotate through initial seeds sequentially
        if not self._warmup_done:
            for idx, sc in enumerate(self._select_count):
                if sc == 0:
                    self._select_count[idx] += 1
                    return self._items[idx]
            self._warmup_done = True
        # Normal: weight = 1/(1+select_count)
        weights = [1.0 / (1 + sc) for sc in self._select_count]
        idx = random.choices(range(len(self._items)), weights=weights, k=1)[0]
        self._select_count[idx] += 1
        # Evict stale seeds — but never evict the last one
        if self._select_count[idx] >= self.MAX_SELECTIONS:
            if len(self._items) > 1:
                self._items.pop(idx)
                self._select_count.pop(idx)
                return self.popleft()
            else:
                # Last seed: reset its counter instead of evicting
                self._select_count[idx] = 0
        return self._items[idx]

    def purge_crashed(self):
        """Remove most-selected seeds on crash detection.

        When a crash is detected, the seeds that have been selected most
        often are likely the ones producing crashes. Remove them to allow
        the queue to recover with fresh seeds. Always keeps at least 2
        seeds to avoid empty queue.
        """
        if len(self._items) <= 2:
            return
        max_sc = max(self._select_count)
        if max_sc < 2:
            return
        to_remove = []
        for i in range(len(self._items) - 1, -1, -1):
            if (self._select_count[i] >= max_sc
                    and len(self._items) - len(to_remove) > 2):
                to_remove.append(i)
        for i in to_remove:
            self._items.pop(i)
            self._select_count.pop(i)
        if to_remove:
            print(f"[SeedQueue] purged {len(to_remove)} stale crash seeds")

    def _is_duplicate(self, seed):
        for existing in self._items:
            if self._seeds_similar(seed, existing):
                return True
        return False

    @staticmethod
    def _seeds_similar(a, b, threshold=0.05):
        if type(a) != type(b):
            return False
        if isinstance(a, list) and isinstance(b, list):
            if len(a) != len(b):
                return False
            for i in [0, len(a) // 2, len(a) - 1]:
                if not SeedQueue._msg_similar(a[i], b[i], threshold):
                    return False
            return True
        return SeedQueue._msg_similar(a, b, threshold)

    @staticmethod
    def _msg_similar(a, b, threshold):
        if hasattr(a, 'position') and hasattr(a.position, 'x'):
            # geometry_msgs/Pose
            dx = abs(a.position.x - b.position.x)
            dy = abs(a.position.y - b.position.y)
            dz = abs(a.position.z - b.position.z)
            return (dx + dy + dz) < threshold
        for attr in ('vx', 'vy', 'vz', 'yawspeed', 'x', 'y', 'z', 'r'):
            va = getattr(a, attr, None)
            vb = getattr(b, attr, None)
            if va is not None and vb is not None:
                if abs(va - vb) > threshold:
                    return False
        return True

    def __len__(self):
        return len(self._items)

    def __bool__(self):
        return len(self._items) > 0


class Fuzzer:
    node_ptr = None

    def __init__(self, node_name, config):
        self.config = config
        self.node_name = node_name
        self.node_ptr = rclpy.create_node(node_name)
        self.subs = []
        self.coverage_map = {}
        self.cov_last_update = time.time()
        self.loop = 0
        self.rounds = 0
        self.pub = None
        self.client = None
        self.shm = None
        self.shm_data = None
        self.running = False
        self.state_monitor_pgrp = None

    def init_cov_map(self):
        print("[*] Initializing shm for coverage tracking")
        if self.config.px4_sitl:
            self.shm = None
            return

        if self.config.sros2:
            self.shm = None
            return

        if self.config.test_moveit:
            self.shm = None
            return

        if self.config.no_cov:
            self.shm = None
            return

        key = ipc.ftok(self.config.proj_root, 2333)
        try:
            self.shm = ipc.SharedMemory(
                key,
                flags=ipc.IPC_CREX,
                size=pow(2, 16),
                init_character=b"\x00",
            )
            print("shmid:", self.shm)
            with open("/tmp/shmid", "w") as fp:
                fp.write(str(self.shm.id))
        except ipc.ExistentialError:
            print("cannot create shm")
            exit(-1)

        if self.config.debug_wait:
            x = input("waiting. press any key")

        self.cov_map = [0] * pow(2, 16)

    def init_shm_data(self):
        print("[*] Initializing shm for data transmission")

        key = ipc.ftok(self.config.proj_root, 2334)

        try:
            self.shm_data = ipc.SharedMemory(
                key,
                flags=ipc.IPC_CREX,
                size=pow(2, 16), # should be bigger?
                init_character=b"\x00",
            )
            print("shm_data id:", self.shm_data, self.shm_data.id)
            self.config.test_rosidl_shmid = self.shm_data.id
            # with open("/tmp/shmid", "w") as fp:
                # fp.write(str(self.shm_data.id))
        except ipc.ExistentialError:
            print("cannot create shm_data")
            exit(-1)

    def init_queue(self):
        print("[*] Initializing test case queue")

        # PX4 and MoveIt use quality-aware SeedQueue; others keep FIFO deque
        if self.config.px4_sitl or self.config.test_moveit:
            self.queue = SeedQueue()
        else:
            self.queue = deque()

        if self.config.px4_sitl:
            if self.config.fuzz_seed:
                msg_list = px4_utils.read_trajectory_seed(self.config.fuzz_seed)

            elif self.config.exp_pgfuzz:
                # campaign: single
                msg = px4_utils.get_init_parameter_msg()
                msg_list = msg

            else:
                # campaign: sequence
                if self.config.use_mavlink:
                    msg = px4_utils.get_init_manual_control_msg()
                else:
                    msg = px4_utils.get_init_trajectory_msg()

                msg_list = list()
                for i in range(self.config.seqlen):
                    msg_list.append(deepcopy(msg))

            self.queue.append(msg_list)

            # --- Boundary seed injection (方案C) ---
            # Add extreme multi-axis seeds to help fuzzer reach oracle boundaries faster.
            if self.config.use_mavlink:
                boundary_seeds = [
                    # Multi-axis extreme: max pitch + max roll → tilt > 45°
                    {"x": 1.0, "y": 1.0, "z": 0.5, "r": 0.0},
                    {"x": -1.0, "y": -1.0, "z": 0.5, "r": 0.0},
                    {"x": 1.0, "y": -1.0, "z": 0.5, "r": 1.0},
                    # Max throttle + full pitch → high velocity + altitude
                    {"x": 1.0, "y": 0.0, "z": 1.0, "r": 0.0},
                    {"x": 0.0, "y": 1.0, "z": 1.0, "r": 0.0},
                    # Direction flip seed: first half +1, second half -1
                    "flip_x",
                    "flip_y",
                ]
                for seed_spec in boundary_seeds:
                    seed_list = []
                    for i in range(self.config.seqlen):
                        msg = px4_utils.get_init_manual_control_msg()
                        if isinstance(seed_spec, dict):
                            msg.x = seed_spec["x"]
                            msg.y = seed_spec["y"]
                            msg.z = seed_spec["z"]
                            msg.r = seed_spec["r"]
                        elif seed_spec == "flip_x":
                            msg.x = 1.0 if i < self.config.seqlen // 2 else -1.0
                        elif seed_spec == "flip_y":
                            msg.y = 1.0 if i < self.config.seqlen // 2 else -1.0
                        seed_list.append(msg)
                    self.queue.append(seed_list)

            elif (
                self.config.px4_ros
                and self.config.target_profile_name != "px4_v117_jazzy"
            ):
                # --- ROS velocity mode boundary seeds ---
                # Domain: vx/vy [-12,12], vz [-1,5], yaw [-pi,pi], yawspeed [-3.49,3.49]
                from px4_msgs.msg import TrajectorySetpoint
                ros_boundary_seeds = [
                    # Multi-axis extreme: max horizontal velocity diagonal
                    {"vx": 12.0, "vy": 12.0, "vz": 0.0, "yaw": 0.0, "yawspeed": 0.0},
                    {"vx": -12.0, "vy": -12.0, "vz": 0.0, "yaw": 0.0, "yawspeed": 0.0},
                    # Max climb + lateral speed (NED: negative vz = climb)
                    {"vx": 12.0, "vy": 0.0, "vz": -5.0, "yaw": 0.0, "yawspeed": 0.0},
                    # Yaw spin + forward velocity (gyroscopic coupling)
                    {"vx": 8.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0, "yawspeed": 3.14},
                    # Direction flip seeds
                    "flip_vx",
                    "flip_vy",
                    # Spiral: forward + climb + yaw spin
                    {"vx": 6.0, "vy": 6.0, "vz": -3.0, "yaw": 0.0, "yawspeed": 2.0},
                ]
                for seed_spec in ros_boundary_seeds:
                    seed_list = []
                    for i in range(self.config.seqlen):
                        msg = TrajectorySetpoint()
                        if isinstance(seed_spec, dict):
                            msg.vx = seed_spec["vx"]
                            msg.vy = seed_spec["vy"]
                            msg.vz = seed_spec["vz"]
                            msg.yaw = seed_spec["yaw"]
                            msg.yawspeed = seed_spec["yawspeed"]
                        elif seed_spec == "flip_vx":
                            msg.vx = 12.0 if i < self.config.seqlen // 2 else -12.0
                        elif seed_spec == "flip_vy":
                            msg.vy = 12.0 if i < self.config.seqlen // 2 else -12.0
                        seed_list.append(msg)
                    self.queue.append(seed_list)
            elif self.config.tb3_sitl or self.config.tb3_hitl:
                # Seed pool: boundary values derived from TurtleBot3 Burger specs
                # Max linear velocity: 0.22 m/s, Max angular velocity: 2.84 rad/s
                from geometry_msgs.msg import Twist
                import seed_generator

                # Single-message seeds (for RND_SINGLE and RND_REPEATED campaigns)
                tb3_seeds = [
                    (0.22, 0.0),     # max forward
                    (-0.22, 0.0),    # max reverse
                    (0.0, 2.84),     # max left turn
                    (0.0, -2.84),    # max right turn
                    (0.22, 2.84),    # max forward + max left
                    (0.22, -2.84),   # max forward + max right
                    (-0.22, 2.84),   # max reverse + max left
                    (0.11, 1.42),    # mid-range values
                    (0.20, 0.0),     # near-boundary forward
                    (0.0, 2.80),     # near-boundary turn
                ]
                for (lin_x, ang_z) in tb3_seeds:
                    msg = Twist()
                    msg.linear.x = lin_x
                    msg.angular.z = ang_z
                    self.queue.append(msg)

                # Sequence seeds only for RND_SEQUENCE campaign
                if self.config.schedule == Campaign.RND_SEQUENCE:
                    seq_seeds = seed_generator.generate_sequence_seeds(
                        "tb3", Twist, seqlen=self.config.seqlen
                            if hasattr(self.config, 'seqlen') else 10
                    )
                    for seq in seq_seeds:
                        self.queue.append(seq)

        elif self.config.tb4_sitl or self.config.tb3_sitl or self.config.tb3_hitl:
            import seed_generator

            if self.config.input_type == "geometry_msgs/msg/TwistStamped":
                from geometry_msgs.msg import TwistStamped as VelocityMsg
                platform = "tb4"
                # Conservative Phase-1 envelope (see seed_generator.py):
                # linear.x in [-0.15, 0.15], angular.z in [-0.8, 0.8].
                velocity_seeds = [
                    (0.15, 0.0),
                    (-0.15, 0.0),
                    (0.0, 0.8),
                    (0.0, -0.8),
                    (0.10, 0.5),
                    (0.10, -0.5),
                ]
            else:
                from geometry_msgs.msg import Twist as VelocityMsg
                platform = "tb3"
                velocity_seeds = [
                    (0.22, 0.0),
                    (-0.22, 0.0),
                    (0.0, 2.84),
                    (0.0, -2.84),
                    (0.22, 2.84),
                    (0.22, -2.84),
                    (-0.22, 2.84),
                    (0.11, 1.42),
                    (0.20, 0.0),
                    (0.0, 2.80),
                ]

            for (lin_x, ang_z) in velocity_seeds:
                msg = VelocityMsg()
                target = msg.twist if hasattr(msg, "twist") else msg
                target.linear.x = lin_x
                target.angular.z = ang_z
                self.queue.append(msg)

            if self.config.schedule == Campaign.RND_SEQUENCE:
                seq_seeds = seed_generator.generate_sequence_seeds(
                    platform,
                    VelocityMsg,
                    seqlen=self.config.seqlen
                        if hasattr(self.config, "seqlen") else 10,
                )
                for seq in seq_seeds:
                    self.queue.append(seq)

        elif self.config.test_moveit:
            # Multi-goal sequence mode with semantic seed injection
            from geometry_msgs.msg import Pose

            # If user provided a seed file, use it as first seed
            if self.config.fuzz_seed:
                msg = harness.get_init_moveit_pose()
                f = open(self.config.fuzz_seed, "rb")
                msg_dict = pickle.load(f)
                msg.position.x = msg_dict["position"]["x"]
                msg.position.y = msg_dict["position"]["y"]
                msg.position.z = msg_dict["position"]["z"]
                msg.orientation.w = msg_dict["orientation"]["w"]
                f.close()
                self.queue.append([msg])

            # --- Semantic seed injection (3 categories, 8 seeds) ---
            def _make_seed(coords):
                seq = []
                for (x, y, z) in coords:
                    msg = harness.get_init_moveit_pose()
                    msg.position.x = x
                    msg.position.y = y
                    msg.position.z = z
                    seq.append(msg)
                return seq

            # Category 1: Reachable seeds (establish feedback baseline)
            self.queue.append(_make_seed([
                (0.4, 0.0, 0.5), (0.0, 0.4, 0.6), (-0.3, 0.2, 0.4)]))
            self.queue.append(_make_seed([
                (0.3, 0.3, 0.3), (0.3, 0.3, 0.7),
                (0.3, 0.3, 0.58), (0.3, 0.3, 0.5)]))
            self.queue.append(_make_seed([
                (0.48, 0.0, 0.48), (0.35, 0.35, 0.48),
                (0.0, 0.48, 0.48), (-0.35, 0.35, 0.48)]))

            # Category 2: Boundary exploration seeds
            self.queue.append(_make_seed([
                (0.40, 0.0, 0.50), (0.52, 0.0, 0.45),
                (0.62, 0.0, 0.35), (0.68, 0.0, 0.28)]))
            self.queue.append(_make_seed([
                (0.3, 0.0, 0.18), (0.3, 0.0, 0.68),
                (0.3, 0.0, 0.12), (0.3, 0.0, 0.6)]))
            self.queue.append(_make_seed([
                (0.42, 0.42, 0.38), (-0.42, -0.42, 0.32),
                (0.42, -0.42, 0.38), (-0.42, 0.42, 0.32)]))

            # Category 3: Semantic scenario seeds
            self.queue.append(_make_seed([
                (0.48, 0.2, 0.4), (0.48, 0.2, 0.55),
                (-0.30, 0.4, 0.4), (-0.30, 0.4, 0.55),
                (0.48, 0.2, 0.4)]))
            self.queue.append(_make_seed([
                (0.55, 0.0, 0.45), (-0.55, 0.0, 0.45),
                (0.0, 0.55, 0.45), (0.0, -0.55, 0.45)]))

    # def start_virtual_display(self):
    # self.display = Display(visible=1, size=(1280, 720))
    # self.display.start()

    def init_px4_bridge(self):
        print("[*] Target: PX4 SITL")
        print("[*] Initializing PX4-ROS bridge")
        self.px4_bridge = px4_utils.Px4BridgeNode(
            use_mavlink=self.config.use_mavlink
        )

    def init_state_monitor(self, watchlist_file):
        """
        Run state monitor node in the background to keep it spinning
        for listening to the subscribed topics and dumping messages
        """

        monitor_run_cmd = f"python3 state_monitor.py {watchlist_file}"

        self.state_monitor_pgrp = sp.Popen(
            monitor_run_cmd,
            shell=True,
            preexec_fn=os.setpgrp,
            stdout=sp.PIPE,
            stderr=sp.PIPE,
        )

    def run_target(self, ros_pkg, ros_node, exec_cmd):

        # os.system("ros2 daemon start")

        if self.node_ptr is None:
            self.node_ptr = rclpy.create_node("_fuzzer")

        if self.config.target_profile is not None:
            profile = self.config.target_profile
            stdout_log_path = os.path.join(
                self.config.log_dir,
                "target.stdout.log",
            )
            try:
                stdout_log_offset = os.path.getsize(stdout_log_path)
            except OSError:
                stdout_log_offset = 0
            self.ros_pgrp = harness.run_target_profile(
                profile,
                self.config.proj_root,
                log_dir=self.config.log_dir,
            )
            ok, log_graph = harness.wait_for_log_patterns(
                stdout_log_path,
                profile.required_log_patterns_for_readiness,
                since_offset=stdout_log_offset,
                timeout_sec=180 if profile.family == "px4" else 120,
            )
            log_graph_path = os.path.join(
                self.config.meta_dir,
                "log_readiness.ready.json",
            )
            with open(log_graph_path, "w") as fp:
                json.dump(log_graph, fp, indent=2, sort_keys=True)
            if not ok:
                raise RuntimeError(
                    f"target profile {profile.name} did not emit "
                    "required launch log patterns before timeout"
                )
            ok, topic_graph = harness.wait_for_topics(
                profile.required_topics_for_readiness,
                timeout_sec=180 if profile.family == "px4" else 120,
            )
            graph_path = os.path.join(
                self.config.meta_dir,
                "topic_graph.ready.json",
            )
            with open(graph_path, "w") as fp:
                json.dump(topic_graph, fp, indent=2, sort_keys=True)
            if not ok:
                raise RuntimeError(
                    f"target profile {profile.name} did not expose "
                    "required topics before timeout"
                )
            ok, action_graph = harness.wait_for_actions(
                profile.required_actions_for_readiness,
                timeout_sec=180 if profile.family == "px4" else 120,
            )
            action_graph_path = os.path.join(
                self.config.meta_dir,
                "action_graph.ready.json",
            )
            with open(action_graph_path, "w") as fp:
                json.dump(action_graph, fp, indent=2, sort_keys=True)
            if not ok:
                raise RuntimeError(
                    f"target profile {profile.name} did not expose "
                    "required actions before timeout"
                )
            ok, data_graph = harness.wait_for_topic_data(
                profile.required_topics_with_data_for_readiness,
                timeout_sec=180 if profile.family == "px4" else 120,
            )
            data_graph_path = os.path.join(
                self.config.meta_dir,
                "topic_data.ready.json",
            )
            with open(data_graph_path, "w") as fp:
                json.dump(data_graph, fp, indent=2, sort_keys=True)
            if not ok:
                raise RuntimeError(
                    f"target profile {profile.name} did not publish data on "
                    "required topics before timeout"
                )
            self.running = True
            return

        if self.config.px4_sitl:
            print("[*] Starting PX4 SITL stack & Gazebo simulator")
            proc = harness.run_px4_stack_sh(self.config.proj_root)
            time.sleep(10)  # TODO: check gazebo status rather than waiting
            print("[px4] started px4 sitl stack")
            self.running = True
            return

        elif self.config.tb3_sitl:
            print("[*] Starting TurtleBot3 SITL stack & Gazebo simulator")
            self.ros_pgrp = harness.run_tb3_sitl(self.config.proj_root)
            time.sleep(10)
            print("[tb3] started tb3 sitl stack", self.ros_pgrp.pid)
            self.running = True
            return

        elif self.config.tb3_hitl:
            print("[*] Starting TurtleBot3 hardware")
            proc = harness.run_tb3_hitl(self.config.tb3_uri)
            time.sleep(10)
            print("[tb3] started turtlebot3 burger")
            self.running = True
            return

        if self.config.test_rcl:
            print("[*] Starting RCL harness")
            self.ros_pgrp = harness.run_rcl_api_harness(
                self.config.test_rcl_feature,
                self.config.test_rcl_targets,
                self.config.test_rcl_job,
            )

            self.running = True

            # add slight delay for topic discovery
            time.sleep(5)

            return

        if self.config.test_cli:
            print("[*] Starting CLI harness")

            self.ros_pgrp = harness.run_cli_harness()

            self.running = True

            # add slight delay for topic discovery
            time.sleep(5)

            return

        if self.config.test_rosidl:
            print("[*] Starting ROSIDL harness")
            self.ros_pgrp = harness.run_rosidl_harness(
                self.config.test_rosidl_lang,
                0, # self.config.test_rosidl_shmid,
                "empty",
            )

            self.running = True

            time.sleep(1)

            return

        if self.config.test_moveit:
            print("[*] Starting moveit2 harness")
            self.ros_pgrp = harness.run_moveit_harness()

            self.running = True

            time.sleep(15)

            return

        if exec_cmd is not None:
            # Non-ROS testing (e.g., PX4)
            # ros_run_cmd = exec_cmd
            ros_run_cmd = exec_cmd
            sleep_interval = 1
        else:
            # ROS testing
            if os.path.exists(self.config.node_executable):
                ros_run_cmd = self.config.node_executable
                sleep_interval = 0.25
            else:
                ros_run_cmd = "ros2 run {} {}".format(ros_pkg, ros_node)
                sleep_interval = 1

            if self.config.sros2:
                env = ""
                env += f"ROS_SECURITY_KEYSTORE={self.config.sros2_keystore} "
                env += f"ROS_SECURITY_ENABLE={self.config.sros2_enable} "
                env += f"ROS_SECURITY_STRATEGY={self.config.sros2_strategy} "
                ros_run_cmd = env + ros_run_cmd
                ros_run_cmd += (
                    f" --ros-args --enclave {self.config.sros2_enclave}"
                )

        if not os.path.exists(os.path.join(self.config.log_dir, "run_cmd")):
            with open(os.path.join(self.config.log_dir, "run_cmd"), "w") as fp:
                fp.write(ros_run_cmd)

        if self.config.sros2:
            try:
                os.remove("/tmp/sros2_started")
            except:
                pass

        self.ros_pgrp = sp.Popen(
            ros_run_cmd,
            shell=True,
            preexec_fn=os.setpgrp,
            stdout=sp.PIPE,
            stderr=sp.PIPE,
        )

        if self.config.sros2:
            while True:
                if os.path.exists("/tmp/sros2_started"):
                    break
                time.sleep(0.05)
        else:
            time.sleep(sleep_interval)

        print("[ros] started target system")
        self.running = True

    def kill_target(self):
        if not self.running and self.config.target_profile is None:
            print("[-] nothing to kill")
            return

        if self.config.target_profile is not None:
            proc = getattr(self, "ros_pgrp", None)
            if proc is not None:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                    for fp in getattr(
                        proc, "_robofuzz_log_handles", None
                    ) or ():
                        try:
                            fp.close()
                        except OSError:
                            pass
                except ProcessLookupError as e:
                    print("[-] profile target killpg error:", e)
                except Exception as e:
                    print("[-] profile target killpg failed:", e)
                try:
                    proc.wait(timeout=5)
                except Exception:
                    pass
                self.ros_pgrp = None
            else:
                for fp in getattr(
                    self, "_robofuzz_log_handles", None
                ) or ():
                    try:
                        fp.close()
                    except OSError:
                        pass

            self.running = False

            if getattr(self.config, "target_family", None) == "moveit":
                harness.sweep_moveit_processes()
            elif getattr(self.config, "target_family", None) == "turtlebot":
                harness.sweep_turtlebot_processes()

        elif self.config.px4_sitl:
            # Only kill PX4, keep Gazebo alive for reuse across iterations
            # (original RoboFuzz behavior — avoids zombie/port issues)
            os.system("pkill px4")
            self.running = False

        elif self.config.tb3_hitl:
            os.system(f"ssh -i keys/tb3 {self.config.tb3_uri} ./kill.sh")
            self.running = False

        else:
            try:
                # send SIGINT instead of SIGTERM so turtle can handle signal
                # and terminate gracefully to create gcda files.
                os.killpg(self.ros_pgrp.pid, signal.SIGKILL)
                self.running = False
            except ProcessLookupError as e:
                print("[-] killpg error:", e)
            except Exception as e:
                print("[-] killpg failed for some reason:", e)

            if self.config.tb3_sitl:
                os.system("pkill gzserver")

        # os.system("ros2 daemon stop")

    def kill_monitor(self):
        # kill state monitor
        if self.state_monitor_pgrp is None:
            return
        try:
            print("killing state monitor")
            os.killpg(self.state_monitor_pgrp.pid, signal.SIGKILL)
            self.state_monitor_pgrp = None
        except ProcessLookupError as e:
            print("[-] state monitor killpg error:", e)
        except AttributeError as e:
            print("[-] state monitor not initialized:", e)

    def destroy_fuzzer_node(self):
        if self.node_ptr is not None:
            self.node_ptr.destroy_node()
            self.node_ptr = None

    def destroy(self):
        # kill target if still exists
        self.kill_target()
        self.kill_monitor()
        try:
            self.executor.kill_rosbag()
        except AttributeError:
            pass

        # clear subscriptions
        if self.node_ptr is not None:
            for sub in self.subs:
                self.node_ptr.destroy_subscription(sub)
            self.subs = []
            self.destroy_fuzzer_node()

        try:
            self.display.stop()
        except AttributeError:
            pass

        if self.shm is not None:
            if self.shm.attached:
                self.shm.detach()
            self.shm.remove()

        if self.shm_data is not None:
            if self.shm_data.attached:
                self.shm_data.detach()
            self.shm_data.remove()


def inspect_target(fuzzer):
    fuzz_targets = []

    built_in_msg_types = ros_utils.get_all_message_types()
    subscriptions = ros_utils.get_subscriptions(fuzzer.node_ptr)

    if fuzzer.config.target_profile is not None:
        profile = fuzzer.config.target_profile
        topic_name = profile.input_topic
        msg_type = profile.input_type
        if profile.family == "moveit":
            topic_name = "/metatopic"
            msg_type = "geometry_msgs/msg/Pose"
        msg_type_class = _msg_class_from_type(msg_type)
        fuzz_targets.append([topic_name, msg_type_class, profile.name])
        return fuzz_targets

    if fuzzer.config.px4_sitl:
        if fuzzer.config.use_mavlink:
            topic_name = "/dummy_mavlink_topic"
            msg_type_class = ros_utils.get_msg_class_from_name(
                "px4_msgs", "ManualControlSetpoint"
            )
        else:
            topic_name = "/TrajectorySetpoint_PubSubTopic"
            msg_type_class = ros_utils.get_msg_class_from_name(
                "px4_msgs", "TrajectorySetpoint"
            )

        fuzz_targets.append([topic_name, msg_type_class, "drone"])
        return fuzz_targets

    elif fuzzer.config.tb3_sitl:
        topic_name = "/cmd_vel"
        msg_type_class = ros_utils.get_msg_class_from_name(
            "geometry_msgs", "Twist"
        )
        fuzz_targets.append(
            [topic_name, msg_type_class, "/turtlebot3_diff_drive"]
        )

        return fuzz_targets

    elif fuzzer.config.tb3_hitl:
        topic_name = "/cmd_vel"
        msg_type_class = ros_utils.get_msg_class_from_name(
            "geometry_msgs", "Twist"
        )
        fuzz_targets.append([topic_name, msg_type_class, "/turtlebot3_node"])

        return fuzz_targets

    elif fuzzer.config.test_rosidl:
        topic_name = "/metatopic" # fake topic (topic tbd by mutator)
        msg_type_class = ros_utils.get_msg_class_from_name(
            "std_msgs", "Empty"
        )
        fuzz_targets.append([topic_name, msg_type_class, "/rosidl_node"])

        return fuzz_targets

    elif fuzzer.config.test_moveit:
        # use below if testing without commander harness
        topic_name = "/motion_plan_request"
        msg_type_class = ros_utils.get_msg_class_from_name(
            "moveit_msgs", "MotionPlanRequest"
        )
        # for now, use commander harness for convenience
        topic_name = "/metatopic"
        msg_type_class = ros_utils.get_msg_class_from_name(
            "geometry_msgs", "Pose"
        )

        fuzz_targets.append([topic_name, msg_type_class, "/moveit_node"])

        return fuzz_targets

    for subscriber_node in subscriptions:
        print()
        print("[+] processing", subscriber_node)

        if subscriber_node.replace("/", "").startswith("_"):
            print("[-] skip internal node")
            continue

        for ti, topic in enumerate(subscriptions[subscriber_node]):
            topic_name = topic[0]
            msg_type_full = topic[1]

            print("[{}] {} {}".format(ti, topic_name, msg_type_full))

            if len(msg_type_full) > 1:
                print("[check] MULTIPLE MESSAGE TYPES!")

            msg_type = msg_type_full[0]
            msg_pkg = msg_type.split("/")[0]
            msg_name = msg_type.split("/")[-1]

            if msg_name == "ParameterEvent":
                print("[-] skip ParameterEvents")
                continue

            if msg_pkg in built_in_msg_types.keys():
                msg_type_class = ros_utils.get_msg_class_from_name(
                    msg_pkg, msg_name
                )
            else:
                msg_type_class = ros_utils.find_custom_msg(msg_type)

            if msg_type_class is None:
                print("[-] couldn't find msg class")
                continue

            print()
            print("Found fuzzing topic", topic_name, "of type", msg_type_class)
            print("- target node:", subscriber_node)
            fuzz_targets.append([topic_name, msg_type_class, subscriber_node])

    return fuzz_targets


def _msg_class_from_type(msg_type):
    msg_pkg = msg_type.split("/")[0]
    msg_name = msg_type.split("/")[-1]
    if msg_pkg in ros_utils.get_all_message_types().keys():
        return ros_utils.get_msg_class_from_name(msg_pkg, msg_name)
    return ros_utils.find_custom_msg(msg_type)


def inspect_secure_target(fuzzer):
    fuzz_targets = []

    built_in_msg_types = ros_utils.get_all_message_types()
    subscriptions = ros_utils.get_secure_subscriptions(fuzzer.node_ptr)

    for subscriber_node in subscriptions:
        print()
        print("[+] processing", subscriber_node)

        if subscriber_node.replace("/", "").startswith("_"):
            print("[-] skip internal node")
            continue

        for ti, topic in enumerate(subscriptions[subscriber_node]):
            topic_name = topic[0]
            msg_type_full = topic[1]

            print("[{}] {} {}".format(ti, topic_name, msg_type_full))

            if len(msg_type_full) > 1:
                print("[check] MULTIPLE MESSAGE TYPES!")

            msg_type = msg_type_full[0]
            msg_pkg = msg_type.split("/")[0]
            msg_name = msg_type.split("/")[-1]

            if msg_name == "ParameterEvent":
                print("[-] skip ParameterEvents")
                continue

            if msg_pkg in built_in_msg_types.keys():
                msg_type_class = ros_utils.get_msg_class_from_name(
                    msg_pkg, msg_name
                )
            else:
                msg_type_class = ros_utils.find_custom_msg(msg_type)

            if msg_type_class is None:
                print("[-] couldn't find msg class")
                continue

            print()
            print("Found fuzzing topic", topic_name, "of type", msg_type_class)
            print("- target node:", subscriber_node)
            fuzz_targets.append([topic_name, msg_type_class, subscriber_node])

    fuzzer.kill_target()
    return fuzz_targets


def _advance_moveit_cycle(scheduler, fbk_list, fuzzer):
    """Run the adaptive MoveIt cycle-end bookkeeping once per round.

    Returns True when a cycle boundary was crossed. ``mutate_sequence_moveit``
    increments ``round_cnt`` *before* a sequence executes, so this must run on
    every round outcome — including the execution-failure path. Otherwise a
    persistently-failing target inflates ``round_cnt`` without ever ending the
    cycle (the cycle-16 "523 rounds, never switched seed" symptom).
    """
    if scheduler.round_cnt < scheduler.CYCLE_MIN:
        return False

    recent = getattr(scheduler, '_recent_interesting_rounds', [])
    recent_in_window = [
        r for r in recent
        if r > scheduler.round_cnt - scheduler.EXTEND_WINDOW]
    if (len(recent_in_window) == 0
            or scheduler.round_cnt >= scheduler.CYCLE_MAX):
        scheduler.round_cnt = 0
        scheduler.cycle_cnt += 1
        scheduler.is_new_cycle = True
        scheduler._recent_interesting_rounds = []
        scheduler._seed_interesting_count = 0
        print("--- cycle finished ---")

        # Stagnation detection: 10 cycles without interesting
        if not hasattr(scheduler, '_cycles_without_interesting'):
            scheduler._cycles_without_interesting = 0
        scheduler._cycles_without_interesting += 1
        if scheduler._cycles_without_interesting >= 10:
            print("[!] STAGNATION: 10 cycles without "
                  "interesting, resetting exploration")
            for fbk in fbk_list:
                fbk.reset()
            scheduler._cycles_without_interesting = 0
            fuzzer.init_queue()
        return True

    return False


def _record_moveit_harness_failure(fuzzer, scheduler, frame, reason):
    """Mark a round as a harness/infrastructure failure, not a target finding.

    A no-goal-handle / readiness-timeout round produces metadata + queue
    entries but no rosbag and no error file, so historically it could only be
    identified by cross-referencing "metadata without rosbag". Writing an
    explicit ``harness_fail-<frame>`` marker makes the 576-style infrastructure
    rounds self-describing for later analysis instead of inferred. This never
    touches the rosbags/ or errors/ directories, so the bug-bearing dataset
    stays clean.
    """
    meta_dir = getattr(fuzzer.config, "meta_dir", None)
    if not meta_dir:
        return
    cycle_cnt = getattr(scheduler, "cycle_cnt", -1)
    round_cnt = getattr(scheduler, "round_cnt", -1)
    marker_path = os.path.join(meta_dir, f"harness_fail-{frame}")
    try:
        with open(marker_path, "w") as fp:
            json.dump(
                {
                    "frame": frame,
                    "cycle": cycle_cnt,
                    "round": round_cnt,
                    "reason": str(reason),
                },
                fp,
                sort_keys=True,
            )
    except OSError as e:
        print(f"[-] could not write harness-failure marker: {e}")


def fuzz_msg(fuzzer, fuzz_targets):

    if len(fuzz_targets) == 0:
        print("[-] Could not discover ROS topic")

    for target in fuzz_targets:
        print("\n----- TARGET INFO -----")
        print("  - TOPIC:", target[0])
        print("  - message type:", target[1])
        print("")

        print("----- BEGIN FUZZING -----")

        # scheduler determines how to mutate, and when to publish mutated messages

        topic_name = target[0]
        msg_type_class = target[1]
        subscriber_node = target[2]

        # campaign = Campaign.RND_SINGLE
        # campaign = Campaign.RND_SEQUENCE
        # campaign = Campaign.RND_REPEATED
        # campaign = Campaign.INTERCEPTION
        campaign = fuzzer.config.schedule
        # Use fast float deterministic stages for TB3 (skip meaningless
        # bit flips on float64 fields)
        fast_float = (
            fuzzer.config.tb4_sitl
            or fuzzer.config.tb3_sitl
            or fuzzer.config.tb3_hitl
        )
        scheduler = Scheduler(fuzzer, campaign, target,
                              fast_float_determ=fast_float)

        field_blacklist = None
        field_whitelist = None
        fbk_list = list()

        # Per-target configuration
        # - whitelist and blacklist
        # - feedback attrs
        if fuzzer.config.px4_sitl:
            # field_blacklist = ["acclelration", "jerk", "thrust"]
            if fuzzer.config.exp_pgfuzz:
                field_whitelist = None

            elif fuzzer.config.use_mavlink:
                field_whitelist = [
                    # ["yawspeed", np.dtype("float64")],
                    ["x", np.dtype("float32")],
                    ["y", np.dtype("float32")],
                    ["z", np.dtype("float32")],
                    ["r", np.dtype("float32")],
                    # ["vx", np.dtype("float64")],
                    # ["vy", np.dtype("float64")],
                    # ["vz", np.dtype("float64")],
                ]

            else:
                if fuzzer.config.target_profile_name == "px4_v117_jazzy":
                    field_whitelist = [
                        ["yaw", np.dtype("float32")],
                        ["yawspeed", np.dtype("float32")],
                    ]
                else:
                    field_whitelist = [
                        # ["x", np.dtype("float32")],
                        # ["y", np.dtype("float32")],
                        # ["z", np.dtype("float32")],
                        ["yaw", np.dtype("float64")],
                        ["yawspeed", np.dtype("float64")],
                        ["vx", np.dtype("float64")],
                        ["vy", np.dtype("float64")],
                        ["vz", np.dtype("float64")],
                    ]

            fbk = Feedback("imu_accel_inconsistency", FeedbackType.INC)
            fbk_list.append(fbk)
            fbk = Feedback("imu_gyro_inconsistency", FeedbackType.INC)
            fbk_list.append(fbk)

            # gps raw vs estimation
            fbk = Feedback("gps_lat_inconsistency", FeedbackType.INC)
            fbk_list.append(fbk)
            fbk = Feedback("gps_lon_inconsistency", FeedbackType.INC)
            fbk_list.append(fbk)

            # Deep physical oracle feedback metrics
            fbk = Feedback("max_tilt_angle", FeedbackType.INC)
            fbk_list.append(fbk)
            fbk = Feedback("max_xy_velocity", FeedbackType.INC)
            fbk_list.append(fbk)
            fbk = Feedback("max_angular_rate", FeedbackType.INC)
            fbk_list.append(fbk)
            fbk = Feedback("max_jerk", FeedbackType.INC)
            fbk_list.append(fbk)
            fbk = Feedback("vel_pos_inconsistency", FeedbackType.INC)
            fbk_list.append(fbk)
            fbk = Feedback("max_altitude", FeedbackType.INC)
            fbk_list.append(fbk)
            # Combined multi-axis feedback: guides toward simultaneous extreme inputs
            fbk = Feedback("combined_tilt_velocity", FeedbackType.INC)
            fbk_list.append(fbk)
            # New: guides fuzzer toward control authority loss
            fbk = Feedback("actuator_saturation", FeedbackType.INC)
            fbk_list.append(fbk)
            # New: guides fuzzer toward EKF routing inconsistencies
            fbk = Feedback("odom_pos_divergence", FeedbackType.INC)
            fbk_list.append(fbk)
            # New: guides fuzzer toward control loop stalls
            fbk = Feedback("control_loop_gap", FeedbackType.INC)
            fbk_list.append(fbk)

        elif fuzzer.config.tb4_sitl:
            if fuzzer.config.input_type == "geometry_msgs/msg/TwistStamped":
                field_whitelist = [
                    ["twist", "linear", "x", np.dtype("float64")],
                    ["twist", "angular", "z", np.dtype("float64")],
                ]
            else:
                field_whitelist = [
                    ["linear", "x", np.dtype("float64")],
                    ["angular", "z", np.dtype("float64")],
                ]

            # Phase-1 TurtleBot4 feedback metrics (populated by
            # oracles.turtlebot._check_turtlebot4_smoke). These are basic
            # semantic signals, not deep oracle thresholds. DEC favors
            # smaller minimum scan distances, guiding toward obstacle/contact
            # boundaries; the other metrics use INC for larger anomalies.
            fbk = Feedback("scan_min_range", FeedbackType.DEC)
            fbk_list.append(fbk)
            fbk = Feedback("scan_invalid_ratio", FeedbackType.INC)
            fbk_list.append(fbk)
            fbk = Feedback("cmd_odom_linear_agreement", FeedbackType.INC)
            fbk_list.append(fbk)
            fbk = Feedback("cmd_odom_angular_agreement", FeedbackType.INC)
            fbk_list.append(fbk)

        elif fuzzer.config.tb3_hitl:
            field_whitelist = [
                ["linear", "x", np.dtype("float64")],
                ["angular", "z", np.dtype("float64")],
            ]

            # Thresholds calibrated from empirical data in empty_world:
            # theta_diff: ~1e-7 range, no threshold needed (INC is self-regulating)
            # max_linear_accel: ~0.99-1.58 range, low threshold to filter idle noise
            # max_angular_accel: ~0.001-0.063 range, original 2.0 was unreachable
            # quat_norm_deviation: ~2e-16 in simulation, keep as correctness check
            # vel_pos_inconsistency: ~0.001-0.041 range
            # imu_odom_accel_diff: ~1.07-1.51 range
            fbk = Feedback("theta_diff", FeedbackType.INC)
            fbk_list.append(fbk)
            fbk = Feedback("max_linear_accel", FeedbackType.INC, min_threshold=0.1)
            fbk_list.append(fbk)
            fbk = Feedback("max_angular_accel", FeedbackType.INC, min_threshold=0.005)
            fbk_list.append(fbk)
            fbk = Feedback("quat_norm_deviation", FeedbackType.INC)
            fbk_list.append(fbk)
            fbk = Feedback("vel_pos_inconsistency", FeedbackType.INC, min_threshold=0.001)
            fbk_list.append(fbk)
            fbk = Feedback("imu_odom_accel_diff", FeedbackType.INC, min_threshold=0.1)
            fbk_list.append(fbk)

        elif fuzzer.config.tb3_sitl:
            field_whitelist = [
                ["angular", "z", np.dtype("float64")],
                ["linear", "x", np.dtype("float64")],
            ]

            # Thresholds calibrated from empirical data in empty_world:
            # theta_diff: ~1e-7 range, no threshold (like original RoboFuzz)
            # max_linear_accel: ~0.99-1.58, threshold filters idle jitter
            # max_angular_accel: ~0.001-0.063, original 2.0 was unreachable
            # quat_norm_deviation: ~2e-16, no threshold (correctness metric)
            # vel_pos_inconsistency: ~0.001-0.041
            # imu_odom_accel_diff: ~1.07-1.51
            fbk = Feedback("theta_diff", FeedbackType.INC)
            fbk_list.append(fbk)
            fbk = Feedback("max_linear_accel", FeedbackType.INC, min_threshold=0.1)
            fbk_list.append(fbk)
            fbk = Feedback("max_angular_accel", FeedbackType.INC, min_threshold=0.005)
            fbk_list.append(fbk)
            fbk = Feedback("quat_norm_deviation", FeedbackType.INC)
            fbk_list.append(fbk)
            fbk = Feedback("vel_pos_inconsistency", FeedbackType.INC, min_threshold=0.001)
            fbk_list.append(fbk)
            fbk = Feedback("imu_odom_accel_diff", FeedbackType.INC, min_threshold=0.1)
            fbk_list.append(fbk)

        elif fuzzer.config.rospkg == "turtlesim":
            field_whitelist = [
                ["linear", "x", np.dtype("float64")],
                ["linear", "y", np.dtype("float64")],
                ["angular", "z", np.dtype("float64")]
            ]
            # field_whitelist = None

        elif "turtlebot3_drive" in fuzzer.config.exec_cmd:
            field_whitelist = [
                # ["pose", "pose", "orientation", "x", np.dtype("float64")],
                # ["pose", "pose", "orientation", "y", np.dtype("float64")],
                # ["pose", "pose", "orientation", "z", np.dtype("float64")],
                ["pose", "pose", "orientation", "w", np.dtype("float64")],
            ]

        elif fuzzer.config.test_rosidl:
            # - Gets a metatopic of type std_msgs/Empty
            # - Need to test a moving target (type)
            # - Already know that there's only one field, so the scheduler
            #   can be optimized
            # - Control topic_name, msg_type_class, and subscriber_node
            pass

        elif fuzzer.config.test_moveit:
            # when testing with commander harness
            field_whitelist = [
                ["position", "x", np.dtype("float64")],
                ["position", "y", np.dtype("float64")],
                ["position", "z", np.dtype("float64")],
                # ["orientation", "w", np.dtype("float64")],
            ]

            fbk = Feedback("end_point_deviation", FeedbackType.INC,
                           default_value=0.0, min_threshold=0.005)
            fbk_list.append(fbk)

            fbk = Feedback("mean_joint_pos_error", FeedbackType.INC,
                           default_value=0.0, min_threshold=0.02)
            fbk_list.append(fbk)

            fbk = Feedback("max_joint_pos_error", FeedbackType.INC,
                           default_value=0.0, min_threshold=0.05)
            fbk_list.append(fbk)

            fbk = Feedback("mean_joint_vel_error", FeedbackType.INC,
                           default_value=0.0, min_threshold=0.05)
            fbk_list.append(fbk)

            fbk = Feedback("max_joint_vel_error", FeedbackType.INC,
                           default_value=0.0, min_threshold=0.1)
            fbk_list.append(fbk)

            fbk = Feedback("max_velocity_margin", FeedbackType.DEC,
                           default_value=10.0, min_threshold=0.5)
            fbk_list.append(fbk)

            fbk = Feedback("trajectory_tracking_rms", FeedbackType.INC,
                           default_value=0.0, min_threshold=0.03)
            fbk_list.append(fbk)

            fbk = Feedback("abort_joint_drift", FeedbackType.INC,
                           default_value=0.0, min_threshold=0.005)
            fbk_list.append(fbk)

            fbk = Feedback("workspace_boundary_distance", FeedbackType.INC,
                           default_value=0.0, min_threshold=5.0)
            fbk_list.append(fbk)

            fbk = Feedback("planning_duration", FeedbackType.INC,
                           default_value=0.0, min_threshold=2.0)
            fbk_list.append(fbk)

            fbk = Feedback("max_joint_jerk", FeedbackType.INC,
                           default_value=0.0, min_threshold=100.0)
            fbk_list.append(fbk)

            fbk = Feedback("goal_success_ratio", FeedbackType.INC,
                           default_value=0.0, min_threshold=0.2)
            fbk_list.append(fbk)

            fbk = Feedback("velocity_roughness", FeedbackType.INC,
                           default_value=0.0, min_threshold=0.5)
            fbk_list.append(fbk)

            fbk = Feedback("joint_motion_range", FeedbackType.INC,
                           default_value=0.0, min_threshold=0.5)
            fbk_list.append(fbk)

            fbk = Feedback("goal_transition_error", FeedbackType.INC,
                           default_value=0.0, min_threshold=0.01)
            fbk_list.append(fbk)

            fbk = Feedback("desired_vel_max_ratio", FeedbackType.INC,
                           default_value=0.0, min_threshold=0.8)
            fbk_list.append(fbk)

            fbk = Feedback("desired_acc_max_ratio", FeedbackType.INC,
                           default_value=0.0, min_threshold=0.8)
            fbk_list.append(fbk)

            fbk = Feedback("desired_jerk_max_ratio", FeedbackType.INC,
                           default_value=0.0, min_threshold=0.8)
            fbk_list.append(fbk)

            fbk = Feedback("execution_sample_count", FeedbackType.INC,
                           default_value=0.0, min_threshold=5.0)
            fbk_list.append(fbk)

            fbk = Feedback("success_endpoint_outlier_score",
                           FeedbackType.INC,
                           default_value=0.0,
                           min_threshold=0.005)
            fbk_list.append(fbk)

            fbk = Feedback("reachable_rejection_score", FeedbackType.INC,
                           default_value=0.0, min_threshold=1.0)
            fbk_list.append(fbk)

            fbk = Feedback("status_transition_anomaly_score",
                           FeedbackType.INC,
                           default_value=0.0,
                           min_threshold=1.0)
            fbk_list.append(fbk)

            fbk = Feedback("tracking_error_growth", FeedbackType.INC,
                           default_value=0.0, min_threshold=0.01)
            fbk_list.append(fbk)

            fbk = Feedback("smoothness_violation_ratio", FeedbackType.INC,
                           default_value=0.0, min_threshold=0.8)
            fbk_list.append(fbk)

        scheduler.filter_field_list(field_whitelist, field_blacklist)
        scheduler.init_schedule()

        # fuzzer.pub = fuzzer.node_ptr.create_publisher(msg_type_class, topic_name, 10)
        # fuzzer.state_pub = fuzzer.node_ptr.create_publisher(
        #     Bool,
        #     "_listen_flag",
        #     10
        # )

        executor = Executor(fuzzer)
        fuzzer.executor = executor

        state_monitor = StateMonitorNode(fuzzer)

        if scheduler.campaign == Campaign.RND_SINGLE:
            # mutate and publish one message
            mode = ExecMode.SINGLE
            # MoveIt uses multi-goal sequences even in RND_SINGLE campaign
            if fuzzer.config.test_moveit:
                mode = ExecMode.SEQUENCE
        elif scheduler.campaign == Campaign.RND_SEQUENCE:
            # mutate one from a sequence of messages, publish the sequence
            length = fuzzer.config.seqlen  # todo: apply in scheduler
            mode = ExecMode.SEQUENCE
        elif scheduler.campaign == Campaign.RND_REPEATED:
            # mutate one message and repeatedly publish it for length times
            length = fuzzer.config.seqlen
            mode = ExecMode.SEQUENCE
        elif scheduler.campaign == Campaign.INTERCEPTION:
            mode = ExecMode.SEQUENCE
        elif scheduler.campaign == Campaign.SROS_AUTH:
            mode = ExecMode.SINGLE
        elif scheduler.campaign == Campaign.IDL_CHECK:
            mode = ExecMode.SINGLE

        frequency = 1.0 / config.interval
        repeat = config.repeat

        msg_list = None
        while True:
            errs = []

            if scheduler.campaign == Campaign.RND_SINGLE:
                if fuzzer.config.test_moveit:
                    # Multi-goal sequence mutation (feedback-adaptive)
                    (msg_list, frame) = scheduler.mutate_sequence_moveit(
                        config, fbk_list)
                elif fuzzer.config.exp_pgfuzz:
                    # param value mutation like what pgfuzz does
                    (mut_msg, frame) = scheduler.mutate_px4_param(config)

                else:
                    (mut_msg, frame) = scheduler.mutate_generic(config)

                if not fuzzer.config.test_moveit:
                    if mut_msg is None:
                        continue

                    msg_list = [mut_msg]

            elif scheduler.campaign == Campaign.RND_SEQUENCE:
                if fuzzer.config.use_mavlink:
                    (msg_list, frame) = scheduler.mutate_sequence_mav(config)
                elif fuzzer.config.px4_ros:
                    (msg_list, frame) = scheduler.mutate_sequence_ros(
                        config, fbk_list)
                else:
                    (msg_list, frame) = scheduler.mutate_sequence(config)

            elif scheduler.campaign == Campaign.RND_REPEATED:
                (mut_msg, frame) = scheduler.mutate_generic(config)

                if mut_msg is None:
                    continue

                # msg_list = [mut_msg] * random.randint(20, 30)
                msg_list = [mut_msg] * length

            elif scheduler.campaign == Campaign.IDL_CHECK:
                # if (scheduler.round_cnt % 256) == 0:
                (mut_msg, frame, error, expecting) = scheduler.mutate_typemsg(config)
                # print(mut_msg)

                # check for errors that appear early in rosidl_py
                if expecting:
                    if error is None:
                        print(f"{c.RED}[-] Expecting '{expecting}', nothing caught{c.END}")
                        errs.append(f"Expecting '{expecting}', nothing caught")

                        # log early errs
                        if errs:
                            err_file = os.path.join(
                                fuzzer.config.error_dir, f"error-{frame}"
                            )

                            with open(err_file, "a") as fp:
                                fp.write(str(errs))

                    else:
                        print("[+] Expected error caught:")
                        print(error)

                    # error is expected, so don't publish
                    continue

                else:
                    if error:
                        print("{c.RED}[-] Not expecting an error, but caught: {error}{c.END}")
                        errs.append(f"Not expecting an error, caught '{error}'")

                        # log early errs
                        if errs:
                            err_file = os.path.join(
                                fuzzer.config.error_dir, f"error-{frame}"
                            )

                            with open(err_file, "a") as fp:
                                fp.write(str(errs))

                        continue

                msg_list = [mut_msg]

                # publish (via rclpy) to target (rclcpp), and re-publish from
                # target (rclcpp). Whatever msg reached the rclcpp target
                # should not fail while assigning received data to the
                # message. In the meantime, monitor (rosbag) the rclcpp pub
                # topic and check if the messages are identical (checker).

                # XXX: get msg_type_class and topic_name from mut_msg
                # as they're dynamic.
                msg_pkg = "idltest_msgs"
                msg_name = type(mut_msg).__name__

                msg_type_class = ros_utils.get_msg_class_from_name(
                    msg_pkg, msg_name
                )
                topic_name = f"/idltest_{msg_name}_in"

            if msg_list is None:
                continue

            if fuzzer.config.tb4_sitl:
                import seed_generator
                seed_generator.clamp_velocity_sequence(
                    msg_list,
                    max_linear=0.15,
                    max_angular=0.8,
                )

            executor.prep_execution(msg_type_class, topic_name)

            # register pre_exec functions and custom publisher function
            if fuzzer.config.px4_sitl:
                collision_checker = checker.CollisionChecker()
                collision_topics = [
                    # "/gazebo/default/iris/base_link/px4_base_contact/contacts",
                    "/gazebo/default/iris/rotor_0/px4_rotor0_contact/contacts",
                    "/gazebo/default/iris/rotor_1/px4_rotor1_contact/contacts",
                    "/gazebo/default/iris/rotor_2/px4_rotor2_contact/contacts",
                    "/gazebo/default/iris/rotor_3/px4_rotor3_contact/contacts",
                ]

                # list of tuple (function, (args))
                pre_exec_list = list()
                if fuzzer.config.use_mavlink:
                    pre_exec_list.append((fuzzer.px4_bridge.init_mavlink, ()))

                if fuzzer.config.exp_pgfuzz:
                    # test px4 by mutating parameter values
                    pre_exec_list.extend([
                        (fuzzer.px4_bridge.prepare_flight, ()),
                        (time.sleep, (5,)),
                        (fuzzer.px4_bridge.mav_set_flight_mode, ("LOITER",)),
                        (time.sleep, (3,)),
                        (collision_checker.listen, (collision_topics,)),
                    ])

                    pub_function = fuzzer.px4_bridge.mav_set_param_msg

                    post_exec_list = [
                        (fuzzer.px4_bridge.mav_revert_param, ()),
                        (collision_checker.stop, ()),
                    ]

                elif fuzzer.config.px4_ros:
                    # test px4 over ROS
                    pre_exec_list.extend([
                        (fuzzer.px4_bridge.prepare_flight, (fuzzer.config.flight_mode,)),
                    ])
                    if fuzzer.config.target_profile_name != "px4_v117_jazzy":
                        pre_exec_list.append(
                            (collision_checker.listen, (collision_topics,))
                        )

                    pub_function = fuzzer.px4_bridge.send_command

                    post_exec_list = []
                    if fuzzer.config.target_profile_name != "px4_v117_jazzy":
                        post_exec_list.append((collision_checker.stop, ()))

                else:
                    # test px4 over mavlink using manual control commands
                    pre_exec_list.extend([
                        (fuzzer.px4_bridge.prepare_flight, (fuzzer.config.flight_mode,)),
                        (fuzzer.px4_bridge.put_in_air, ()),
                        (collision_checker.listen, (collision_topics,)),
                    ])

                    pub_function = fuzzer.px4_bridge.send_command

                    post_exec_list = [
                        (collision_checker.stop, ()),
                    ]

            # ditch the shm stuff
            # elif fuzzer.config.test_rosidl:
                # pub_function = None # writes to the shared memory

            # recover full MotionPlanRequest msg by injecting JointConstraint
            elif fuzzer.config.test_moveit:
                # use below for generic joint constraints
                # # should contain seven (mutated) joint constraints
                # joint_constraints = msg_list[0] # list contains one list
                # print("check")
                # print(joint_constraints)

                # full_msg = harness.get_init_moveit_msg()
                # # goal_constraints is a list of single element
                # goal_constraint = Constraints()
                # goal_constraint.joint_constraints = joint_constraints
                # full_msg.goal_constraints = [goal_constraint]
                # msg_list = [full_msg]

                pre_exec_list = None
                post_exec_list = None
                plan_params = getattr(scheduler, "_plan_params", None)
                pub_function = partial(
                    harness.moveit_send_command,
                    plan_params=plan_params,
                )

            else:
                pre_exec_list = None
                post_exec_list = None
                pub_function = None

            wait_lock = None
            if fuzzer.config.test_rcl or fuzzer.config.test_cli:
                wait_lock = ".waitlock"

            try:
                (retval, failure_msg) = executor.execute(
                    mode,
                    msg_list,
                    frame,
                    frequency,
                    repeat,
                    pre_exec_list=pre_exec_list,
                    post_exec_list=post_exec_list,
                    pub_function=pub_function,
                    wait_lock=wait_lock,
                )
            except RuntimeError as e:
                print(f"[!] Execution failed: {e}, skipping cycle")
                fuzzer.kill_target()
                if fuzzer.config.test_moveit:
                    # Mark this round as a harness/infra failure so the
                    # bookkeeping-only round is self-describing (no rosbag, no
                    # error file) instead of inferred from "metadata without
                    # rosbag" during later analysis.
                    _record_moveit_harness_failure(
                        fuzzer, scheduler, frame, e)
                    # round_cnt was already incremented by
                    # mutate_sequence_moveit. Advance the cycle bookkeeping and
                    # the global round accounting here too, otherwise a
                    # persistently-failing target loops forever on one seed and
                    # never reaches maxloop (the cycle-16 runaway).
                    _advance_moveit_cycle(scheduler, fbk_list, fuzzer)
                    fuzzer.rounds += 1
                    if (fuzzer.config.maxloop > 0
                            and fuzzer.rounds >= fuzzer.config.maxloop):
                        print(f"[*] Reached maxloop={fuzzer.config.maxloop}; "
                              "stopping")
                        break
                elif fuzzer.config.target_profile is not None:
                    # Modern non-MoveIt profiles can fail in readiness before
                    # a rosbag/oracle phase starts. Count the attempt so
                    # maxloop still terminates short smoke tests instead of
                    # retrying forever.
                    fuzzer.rounds += 1
                    if (fuzzer.config.maxloop > 0
                            and fuzzer.rounds >= fuzzer.config.maxloop):
                        print(f"[*] Reached maxloop={fuzzer.config.maxloop}; "
                              "stopping")
                        break
                continue
            # fuzzer.oh_.check_oracle() # will move everything into checker
            # (turtlesim, sros, ...)
            executor.clear_execution()

            if retval:
                errs.append(f"publish failed: {failure_msg}")

                err_file = os.path.join(
                    fuzzer.config.error_dir, f"error-{frame}"
                )

                with open(err_file, "a") as fp:
                    fp.write(str(errs))

                continue # don't check states as nothing's published

            state_dict_list = []
            ulg_path = None  # track for error preservation
            # repeated campaigns result in multiple bag files
            # MoveIt: all goals sent in one execution, single bag file
            parse_repeat = 1 if fuzzer.config.test_moveit else repeat
            for exec_cnt in range(parse_repeat):

                if fuzzer.config.use_ulg and fuzzer.config.px4_sitl:
                    # --- ULG path: read PX4 internal log (bypasses bridge) ---
                    ulg_path = find_latest_ulg()
                    if ulg_path is None:
                        print("[-] no ULG file found. Skipping oracle.")
                        continue
                    parser = UlgStateParser(ulg_path)
                    if parser.abort:
                        print("[-] corrupted ULG file. Skipping.")
                        continue
                    state_msgs_dict = parser.process_messages()
                    if len(state_msgs_dict) == 0:
                        print("[-] ULG empty after takeoff filter, using all")
                        state_msgs_dict = parser.process_all_messages()
                else:
                    # --- Rosbag path: original behavior ---
                    bag_db = _find_rosbag_db(exec_cnt)
                    if bag_db is None:
                        errs.append(f"rosbag db for execution {exec_cnt} not found")
                        continue
                    parse_started = time.time()
                    parser = RosbagParser(bag_db)
                    try:
                        if parser.abort:
                            print("[-] corrupted recorded states. Abort.")
                            continue

                        state_msgs_dict = parser.process_messages()
                        # if dict is empty, fallback to all messages w/o ts filtering
                        if len(state_msgs_dict) == 0:
                            print(
                                "[fuzzer] time-window parser returned no "
                                "messages; using full rosbag fallback"
                            )
                            state_msgs_dict = parser.process_all_messages()
                    finally:
                        parser.close()
                        print(
                            "[fuzzer] parsed rosbag in "
                            f"{time.time() - parse_started:.3f}s"
                        )

                # state_dict = checker.group_msgs_by_topic(state_msgs_dict)

                if scheduler.campaign == Campaign.RND_REPEATED:
                    # for repeated campaign
                    state_dict_list.append(state_msgs_dict)

                # state_monitor.retrieve_states(exec_cnt)
                # while True:
                #     # loop until ros2 bag play terminates
                #     if state_monitor.rosbag_proc is None:
                #         state_monitor.play_rosbag(exec_cnt)
                #     elif state_monitor.rosbag_proc.poll() is not None:
                #         print("[state monitor] rosbag play done")
                #         break

                #     if len(state_monitor.msg_queue) == 0:
                #         # prevent timing out before receiving the first msg
                #         # TODO: test with very small rosbags as subscription
                #         #       often runs pretty slow
                #         rclpy.spin_once(state_monitor)
                #     else:
                #         rclpy.spin_once(state_monitor, timeout_sec=5)

                # # state_msgs = checker.retrieve_states(exec_cnt)
                # state_dict = checker.group_msgs_by_topic(
                #     state_monitor.msg_queue
                # )
                # for topic in state_dict:
                #     print(topic, len(state_dict[topic]))
                # state_monitor.msg_queue = list()
                # state_monitor.rosbag_proc = None

                # print("run checks")
                # Reset feedback values before oracle check to avoid
                # stale values from previous round leaking through
                for fbk in fbk_list:
                    fbk.value = fbk.default_value
                oracle_started = time.time()
                errs = checker.run_checks(fuzzer.config, msg_list,
                        state_msgs_dict, fbk_list)
                print(
                    "[fuzzer] oracle checked in "
                    f"{time.time() - oracle_started:.3f}s"
                )
                errs = list(set(errs))
                # TODO: bring error logging and is_interesting here

                # P5: novelty by error signature. A seed that only re-discovers
                # an already-seen bug signature is NOT worth re-queueing as
                # interesting — it just churns the queue with duplicates.
                moveit_error_is_novel = False
                if fuzzer.config.test_moveit and errs:
                    if not hasattr(scheduler, '_seen_error_signatures'):
                        scheduler._seen_error_signatures = set()
                    sigs = moveit_error_signature(errs)
                    new_sigs = sigs - scheduler._seen_error_signatures
                    if new_sigs:
                        moveit_error_is_novel = True
                        scheduler._seen_error_signatures |= new_sigs
                        print(f"[dedup] NEW error signature(s): {new_sigs}")
                    else:
                        print(f"[dedup] duplicate error signature(s): {sigs} "
                              f"— will not re-queue as interesting")

                if fuzzer.config.px4_sitl:
                    if collision_checker.found_collision():
                        col_topics = list(
                            collision_checker.collision_events.keys()
                        )
                        print(f"{c.RED}COLLISION: {col_topics}{c.END}")
                        errs.append(col_topics)

                if fuzzer.config.test_rcl:
                    # API cross-check here
                    api_checker = checker.APIChecker(
                        fuzzer.config.test_rcl_feature,
                        fuzzer.config.test_rcl_targets,
                        fuzzer.config.test_rcl_job,
                    )

                    err = api_checker.check_deviant()
                    if err:
                        print(
                            f"{c.RED}{err}: {fuzzer.config.test_rcl_feature}{c.END}"
                        )
                        errs.append(err)
                        for i in range(len(fuzzer.config.test_rcl_targets)):
                            shutil.copyfile(
                                f"out-{i}",
                                os.path.join(
                                    fuzzer.config.error_dir,
                                    f"error-{frame}-out-{i}",
                                ),
                            )
                            shutil.copyfile(
                                f"trace-{i}",
                                os.path.join(
                                    fuzzer.config.error_dir,
                                    f"error-{frame}-trace-{i}",
                                ),
                            )

            if errs:
                err_file = os.path.join(
                    fuzzer.config.error_dir, f"error-{frame}"
                )

                with open(err_file, "a") as fp:
                    fp.write(str(errs))

                if fuzzer.config.use_ulg and fuzzer.config.px4_sitl:
                    # Preserve .ulg as evidence
                    if ulg_path:
                        preserve_ulg_on_error(
                            ulg_path, fuzzer.config.log_dir, frame)
                else:
                    copy_repeat = 1 if fuzzer.config.test_moveit else repeat
                    for exec_cnt in range(copy_repeat):
                        # copy rosbags to {log_dir}/rosbags/{frame}/
                        bag_dir = f"states-{exec_cnt}.bag"
                        try:
                            os.makedirs(os.path.join(fuzzer.config.rosbag_dir, frame), exist_ok=True)
                            shutil.copytree(
                                bag_dir,
                                os.path.join(fuzzer.config.rosbag_dir, frame, bag_dir),
                                dirs_exist_ok=True
                            )
                        except Exception as e:
                            print(f"[!] rosbag copy failed: {e}")

            else:
                print("[+] no error found")

            # Clean up old .ulg files to prevent disk fill
            if fuzzer.config.use_ulg and fuzzer.config.px4_sitl:
                cleanup_ulg_files(keep_latest=1)

            if scheduler.campaign == Campaign.RND_REPEATED:
                rpt_errs = checker.run_rpt_checks(
                    fuzzer.config, state_dict_list
                )

                if rpt_errs:
                    err_file = os.path.join(
                        fuzzer.config.error_dir, f"error-{frame}"
                    )

                    with open(err_file, "a") as fp:
                        fp.write(str(set(rpt_errs)))

            # Check feedback BEFORE reset so high-error-rate runs can
            # still accumulate interesting seeds for exploration
            is_interesting = False
            print(f"[debug] fbk_list len={len(fbk_list)}")
            for fbk in fbk_list:
                # check if any feedback element is interesting
                cur_interesting = fbk.is_interesting()
                print("check is_interesting -",
                    fbk.name,
                    cur_interesting,
                    "({})".format(fbk.value)
                )
                is_interesting = is_interesting or cur_interesting

            if fuzzer.config.test_moveit:
                if not hasattr(scheduler, '_seen_feedback_buckets'):
                    scheduler._seen_feedback_buckets = set()
                new_buckets = (
                    moveit_feedback_buckets.collect_new_feedback_buckets(
                        fbk_list,
                        scheduler._seen_feedback_buckets,
                    )
                )
                if new_buckets:
                    print(
                        "[feedback] NEW MoveIt feedback bucket(s): "
                        f"{new_buckets}"
                    )
                    is_interesting = True

            if is_interesting:
                # Flight quality gate: reject seeds where drone never flew.
                # This prevents crash-inducing inputs from polluting the queue.
                # Only applies to PX4 targets; other targets skip this check.
                flight_quality_ok = True
                if fuzzer.config.px4_sitl:
                    for fbk in fbk_list:
                        if fbk.name == "max_tilt_angle":
                            if fbk.value is None or fbk.value < 2.0:
                                flight_quality_ok = False
                                break
                        if fbk.name == "max_xy_velocity":
                            if fbk.value is None or fbk.value < 0.3:
                                flight_quality_ok = False
                                break

                # MoveIt quality gate: require at least 1 successful goal
                if fuzzer.config.test_moveit:
                    goal_ratio_fbk = None
                    for fbk in fbk_list:
                        if fbk.name == "goal_success_ratio":
                            goal_ratio_fbk = fbk
                            break
                    if goal_ratio_fbk and goal_ratio_fbk.value is not None:
                        if goal_ratio_fbk.value >= 1.0:
                            # fail_ratio=1.0 means all goals failed. Keep a
                            # novel semantic error as evidence; otherwise this
                            # is usually unhelpful unreachable-goal churn.
                            if not moveit_error_is_novel:
                                flight_quality_ok = False
                                print("[feedback] REJECTED — no goal succeeded "
                                      "(quality gate)")
                    else:
                        flight_quality_ok = False

                if not flight_quality_ok:
                    print("[feedback] REJECTED — drone did not fly (quality gate)")
                    is_interesting = False

            if is_interesting:
                print("[feedback] INTERESTING!")
                msg_to_queue = msg_list[0]

                if scheduler.campaign == Campaign.RND_SEQUENCE:
                    # takes the entire list
                    msg_to_queue = msg_list

                # MoveIt multi-goal: always save the full goal sequence
                if fuzzer.config.test_moveit and len(msg_list) > 1:
                    msg_to_queue = msg_list

                # Diminishing returns: limit re-queue per cycle
                if not hasattr(scheduler, '_seed_interesting_count'):
                    scheduler._seed_interesting_count = 0
                scheduler._seed_interesting_count += 1

                # P5: if this round only re-found already-seen bug signatures,
                # don't re-queue — it would just churn duplicates. Seeds that are
                # interesting via feedback with NO errors are still re-queued
                # (valuable for exploration).
                skip_dup_requeue = (
                    fuzzer.config.test_moveit
                    and errs
                    and not moveit_error_is_novel
                )

                if skip_dup_requeue:
                    print("[dedup] skip re-queue — duplicate bug signature only")
                elif scheduler._seed_interesting_count <= 3:
                    if isinstance(fuzzer.queue, SeedQueue):
                        fuzzer.queue.append(deepcopy(msg_to_queue), is_readd=True)
                    else:
                        fuzzer.queue.append(deepcopy(msg_to_queue))
                else:
                    print("[feedback] diminishing — skip re-queue")

                # Reset stagnation counter
                if hasattr(scheduler, '_no_interesting_rounds'):
                    scheduler._no_interesting_rounds = 0
                if hasattr(scheduler, '_cycles_without_new_cov'):
                    scheduler._cycles_without_new_cov = 0
                if hasattr(scheduler, '_cycles_without_interesting'):
                    scheduler._cycles_without_interesting = 0

                # MoveIt: track recent interesting rounds + greedy update
                if fuzzer.config.test_moveit:
                    if not hasattr(scheduler, '_recent_interesting_rounds'):
                        scheduler._recent_interesting_rounds = []
                    scheduler._recent_interesting_rounds.append(
                        scheduler.round_cnt)
                    # Greedy: in Phase 1, replace msg_list for next round
                    if scheduler.round_cnt <= scheduler.EXPLOIT_PHASE_END:
                        scheduler.msg_list = deepcopy(msg_list)
                        print("[feedback] greedy update — Phase 1 seed evolved")

                with open(
                    os.path.join(fuzzer.config.cov_dir, frame),
                    "w",
                ) as fp:
                    for fbk in fbk_list:
                        fp.write(f"{fbk.name} {fbk.value}\n")

            # Do not reset feedback on error for targets with high error rates.
            # Resetting would prevent feedback from ever accumulating, making
            # coverage-guided exploration impossible.
            # Exception: reset crash-sensitive metrics when a crash is detected
            # (attitude > 170 deg), as crash data poisons the feedback ceiling.
            if not (fuzzer.config.tb3_sitl or fuzzer.config.tb3_hitl
                    or fuzzer.config.px4_sitl
                    or fuzzer.config.test_moveit):
                if errs:
                    for fbk in fbk_list:
                        fbk.reset()

            if fuzzer.config.px4_sitl and errs:
                is_crash = any("Attitude" in e and
                    float(e.split()[-2]) > 170
                    for e in errs if "Attitude" in e)
                # Detect ground crash: all motors at MIN for >5s = drone on ground
                is_ground_crash = False
                for e in errs:
                    if "Actuator saturation" in e and "MIN" in e:
                        try:
                            dur = float(e.split("for ")[1].split("s")[0])
                            if dur > 5.0:
                                is_ground_crash = True
                                break
                        except (IndexError, ValueError):
                            pass
                if is_crash or is_ground_crash:
                    crash_metrics = {"max_angular_rate", "max_jerk",
                                     "vel_pos_inconsistency",
                                     "max_xy_velocity", "max_tilt_angle",
                                     "combined_tilt_velocity",
                                     "actuator_saturation"}
                    for fbk in fbk_list:
                        if fbk.name in crash_metrics:
                            fbk.reset()
                    if is_ground_crash:
                        print("[feedback] ground crash detected (motors off >5s), reset polluted metrics")
                    else:
                        print("[feedback] crash detected, reset polluted metrics")
                    # Purge stale crash seeds from queue
                    if isinstance(fuzzer.queue, SeedQueue):
                        fuzzer.queue.purge_crashed()

            # MoveIt: reset feedback when ALL goals abort (no useful data)
            if fuzzer.config.test_moveit and errs:
                goal_ratio_fbk = None
                for fbk in fbk_list:
                    if fbk.name == "goal_success_ratio":
                        goal_ratio_fbk = fbk
                        break
                if goal_ratio_fbk and goal_ratio_fbk.value is not None:
                    if goal_ratio_fbk.value >= 1.0:
                        for fbk in fbk_list:
                            fbk.reset()
                        print("[feedback] all goals aborted, reset feedback")

            if fuzzer.config.px4_sitl:

                # uncomment to collect coverage info
                # lcov_cmd = "lcov "
                # lcov_cmd += "-c "
                # lcov_cmd += "-q "
                # lcov_cmd += "-d /home/seulbae/workspace/px4-cov/build/px4_sitl_rtps "
                # lcov_cmd += "-b /home/seulbae/workspace/px4-cov/build/px4_sitl_rtps "
                # lcov_cmd += "--gcov-tool gcov "
                # lcov_cmd += "--rc lcov_branch_coverage=1 "
                # lcov_cmd += f"-o /home/seulbae/workspace/drone-sec/src/px4-coverage/lcov-{scheduler.cycle_cnt}-{scheduler.round_cnt-1}.info"

                # os.system(lcov_cmd)

                if fuzzer.config.exp_pgfuzz:
                    # cycles better be smaller
                    if scheduler.round_cnt >= 8:
                        scheduler.round_cnt = 0
                        scheduler.cycle_cnt += 1
                        scheduler.is_new_cycle = True
                        scheduler._seed_interesting_count = 0
                        print("--- cycle finished ---")
                else:
                    if scheduler.round_cnt >= 5:
                        scheduler.round_cnt = 0
                        scheduler.cycle_cnt += 1
                        scheduler.is_new_cycle = True
                        scheduler._seed_interesting_count = 0
                        print("--- cycle finished ---")

                        # Stagnation detection: if no new coverage for many
                        # cycles, reset feedback and re-seed the queue.
                        # This prevents the fuzzer from endlessly replaying
                        # crash inputs that produce no new exploration.
                        if not hasattr(scheduler, '_cycles_without_new_cov'):
                            scheduler._cycles_without_new_cov = 0
                        scheduler._cycles_without_new_cov += 1
                        if scheduler._cycles_without_new_cov >= 20:
                            print("[!] STAGNATION: 20 cycles without new "
                                  "coverage, resetting exploration")
                            for fbk in fbk_list:
                                fbk.reset()
                            scheduler._cycles_without_new_cov = 0
                            # Re-initialize queue with fresh boundary seeds
                            fuzzer.init_queue()

            elif fuzzer.config.test_moveit:
                # Adaptive cycle length: min 20, max 30, extend if recent
                # interesting. Shared with the execution-failure path so a
                # broken target cannot stall on one seed indefinitely.
                _advance_moveit_cycle(scheduler, fbk_list, fuzzer)

            elif (fuzzer.config.tb4_sitl or fuzzer.config.tb3_sitl
                    or fuzzer.config.tb3_hitl):
                # For TB3/TB4 sequence mode: cycle every 10 rounds to consume
                # interesting seeds from the queue regularly
                if (scheduler.campaign == Campaign.RND_SEQUENCE
                        and scheduler.round_cnt >= 10):
                    scheduler.round_cnt = 0
                    scheduler.cycle_cnt += 1
                    scheduler.is_new_cycle = True
                    print("--- cycle finished ---")

            fuzzer.rounds += 1
            if fuzzer.config.maxloop > 0 and fuzzer.rounds >= fuzzer.config.maxloop:
                print(f"[*] Reached maxloop={fuzzer.config.maxloop}; stopping")
                break


def _find_rosbag_db(exec_cnt):
    bag_dir = f"states-{exec_cnt}.bag"
    candidates = [
        f"{bag_dir}/{bag_dir}_0.db3",
        f"{bag_dir}/states-{exec_cnt}_0.db3",
    ]
    candidates.extend(sorted(glob.glob(os.path.join(bag_dir, "*.db3"))))
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return None


def main(config):
    print(
        """\
                          ____
   _________  _____      / __/_  __________
  / ___/ __ \/ ___/_____/ /_/ / / /_  /_  /
 / /  / /_/ (__  )_____/ __/ /_/ / / /_/ /_
/_/   \____/____/     /_/  \__,_/ /___/___/
"""
    )

    if config.sros2:
        args = ["--ros-args", "--enclave", "/fuzzer/_fuzzer"]
    else:
        args = None

    rclpy.init(args=args)
    fuzzer = Fuzzer("_fuzzer", config)
    fuzzer.init_cov_map()
    fuzzer.init_queue()

    """
    if config.test_rosidl:
        fuzzer.init_shm_data()
    """

    # fuzzer.start_virtual_display()

    # fuzzer.init_state_monitor(config.watchlist)

    if config.px4_sitl:
        # uncomment to collect coverage info
        # lcov_cmd = "lcov "
        # lcov_cmd += "-d /home/seulbae/workspace/px4-cov/build/px4_sitl_rtps "
        # lcov_cmd += "-b /home/seulbae/workspace/px4-cov/build/px4_sitl_rtps "
        # lcov_cmd += "--zerocounters"
        # os.system(lcov_cmd)

        fuzzer.init_px4_bridge()

    if config.sros2:
        fuzzer.oh_ = OracleHandler(fuzzer, "sros2")
    # else:
    #     fuzzer.oh_ = OracleHandler(fuzzer, "turtlesim")

    # Legacy targets need graph inspection. Modern profiles already declare
    # their input topic/type and launch the target during execution.
    if config.target_profile is None:
        fuzzer.run_target(config.rospkg, config.rosnode, config.exec_cmd)
    else:
        print(f"[*] Using target profile {config.target_profile_name}")

    # pp = pprint.PrettyPrinter(indent=2)

    np.random.seed(config.seed)

    if config.method == "message":
        if config.sros2:
            try:
                fuzz_targets = inspect_secure_target(fuzzer)

                if len(fuzz_targets) == 0:
                    # dealing with slow node discovery
                    mtc = ros_utils.get_msg_class_from_name(
                        "std_msgs", "String"
                    )
                    fuzz_targets.append(["/sros2_input", mtc, "/sros2_node"])

                if not config.persistent:
                    fuzzer.kill_target()

                fuzz_msg(fuzzer, fuzz_targets)

            except Exception as e:
                print("Runtime error:", str(e))
                exc_type, exc_obj, exc_tb = sys.exc_info()
                traceback.print_tb(exc_tb)

            except KeyboardInterrupt as e:
                print("Exiting on user request")

            finally:
                fuzzer.kill_target()
                fuzzer.destroy()
                safe_rclpy_shutdown()

        else:
            try:
                fuzz_targets = inspect_target(fuzzer)

                if not config.persistent and config.target_profile is None:
                    fuzzer.kill_target()

                fuzz_msg(fuzzer, fuzz_targets)

            except Exception as e:
                print("Runtime error:", str(e))
                exc_type, exc_obj, exc_tb = sys.exc_info()
                traceback.print_tb(exc_tb)

            except KeyboardInterrupt as e:
                print("Exiting on user request")

            finally:
                fuzzer.kill_target()
                fuzzer.destroy()
                safe_rclpy_shutdown()

    elif config.method == "service":
        try:
            fuzz_srv(fuzzer)
        except Exception as e:
            print("Runtime error:", str(e))
            exc_type, exc_obj, exc_tb = sys.exc_info()
            traceback.print_tb(exc_tb)
        finally:
            fuzzer.destroy()
            safe_rclpy_shutdown()


if __name__ == "__main__":
    # Ignore SIGHUP/SIGPIPE so terminal disconnection doesn't kill the fuzzer.
    # This allows long-running experiments to survive SSH/docker session drops.
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    signal.signal(signal.SIGPIPE, signal.SIG_IGN)

    argparser = argparse.ArgumentParser()

    argparser.add_argument(
        "--method",
        default="message",
        type=str,
        help="Method to test: message / service / action",
    )
    argparser.add_argument(
        "--schedule",
        default="single",
        type=str,
        help="Test scheduling: single / sequence / repeated",
    )
    argparser.add_argument(
        "--seqlen",
        default=10,
        type=int,
        help="(if schedule is sequence or repeated) number of messages in a sequence",
    )
    argparser.add_argument(
        "--repeat",
        default=1,
        type=int,
        help="Number of repetitions of a (sequence of) message",
    )
    argparser.add_argument(
        "--logdir",
        default="logs",
        type=str,
        help="dir to save messages and fuzzing metadata",
    )
    argparser.add_argument(
        "--startover",
        action="store_true",
        help="restart fuzzing from clean slate every time",
    )
    argparser.add_argument(
        "--maxloop",
        default=100,
        type=int,
        help="number of iterations, 0 for infinite",
    )
    argparser.add_argument(
        "--interval",
        default=1.0,
        type=float,
        help="interval b/w published messages in float",
    )
    argparser.add_argument(
        "--ros-pkg",
        type=str,
        help="name of ROS package to test \
                                 (e.g., turtlesim)",
    )
    argparser.add_argument(
        "--ros-node",
        type=str,
        help="name of core node to fuzz \
                                 (e.g., turtlesim_node)",
    )
    argparser.add_argument(
        "--exec-cmd",
        type=str,
        help="command to execute target system, if it is \
                                 not a ROS package",
    )
    argparser.add_argument(
        "--watchlist",
        default="watchlist/empty.json",
        type=str,
        help="path to the file containing topic watchlist",
    )
    argparser.add_argument(
        "--determ-seed",
        type=int,
        help="seed value for deterministic execution",
    )
    argparser.add_argument(
        "--fuzz-seed", type=str, help="seed file for mutation"
    )
    argparser.add_argument(
        "--target-profile",
        choices=sorted(target_profiles.KNOWN_PROFILES),
        help="Jazzy target profile to run",
    )
    argparser.add_argument(
        "--tb4-sitl",
        action="store_true",
        help="shortcut to testing TurtleBot4 Jazzy in SITL mode",
    )
    argparser.add_argument(
        "--px4-v117-sitl",
        action="store_true",
        help="shortcut to testing PX4 v1.17 Jazzy SITL over uXRCE-DDS",
    )
    argparser.add_argument(
        "--px4-sitl-ros",
        action="store_true",
        help="shortcut to testing px4 drones in sitl mode (use ros)",
    )
    argparser.add_argument(
        "--px4-sitl-mav",
        action="store_true",
        help="shortcut to testing px4 drones in sitl mode (use mavlink)",
    )
    argparser.add_argument(
        "--px4-flight-mode",
        type=str,
        default="MANUAL",
        help="PX4 flight mode to test: (MANUAL/POSCTL/ALTCTL/ACRO/TAKEOFF)",
    )
    argparser.add_argument(
        "--px4-sitl-pgfuzz",
        action="store_true",
        help="shortcut to testing px4 drones with PGFUZZ style mutation",
    )
    argparser.add_argument(
        "--tb3-sitl",
        action="store_true",
        help="shortcut to testing turtlebot3 in sitl mode",
    )
    argparser.add_argument(
        "--tb3-hitl",
        action="store_true",
        help="shortcut to testing turtlebot3 in hitl mode",
    )
    argparser.add_argument(
        "--tb3-uri",
        type=str,
        default="ubuntu@192.168.0.154",
        help="(if testing hitl) ip address of TurtleBot3",
    )
    argparser.add_argument(
        "--px4-mission", default="", type=str, help="offboard mission file"
    )
    argparser.add_argument(
        "--sros2", action="store_true", help="shortcut to testing SROS2"
    )
    argparser.add_argument(
        "--no-cov",
        action="store_true",
        help="assume no coverage data (do not create shm)",
    )
    argparser.add_argument(
        "--persistent",
        action="store_true",
        help="keep target running after a round of fuzzing",
    )
    argparser.add_argument(
        "--wait",
        action="store_true",
        help="[debugging] wait after creating shm",
    )

    argparser.add_argument(
        "--test-rcl",
        action="store_true",
        help="shortcut for testing RCL API consistency",
    )
    argparser.add_argument(
        "--rcl-api",
        type=str,
        default="",
        help="(if testing rcl) specify API group to trace (e.g., publisher)",
    )
    argparser.add_argument(
        "--rcl-job",
        type=str,
        default="",
        help="(if testing rcl) specify API to execute (e.g., create_publisher)",
    )

    argparser.add_argument(
        "--test-cli",
        action="store_true",
        help="shortcut for testing RCL CLI + API consistency",
    )

    argparser.add_argument(
        "--test-rosidl",
        action="store_true",
        help="shortcut for testing ROS IDL (type system)",
    )
    argparser.add_argument(
        "--test-moveit",
        action="store_true",
        help="shortcut for testing moveit library",
    )
    argparser.add_argument(
        "--use-ulg",
        action="store_true",
        help="use PX4 internal .ulg logs instead of rosbag for oracle (bypasses microRTPS bridge)",
    )

    args = argparser.parse_args()

    now = datetime.now()
    now_str = now.strftime("%Y%m%d-%H%M%S")
    log_dir = os.path.join(args.logdir, now_str)
    try:
        os.makedirs(log_dir)
    except FileExistsError:
        pass
    latest_link = os.path.join(args.logdir, "latest")
    if os.path.islink(latest_link) or os.path.exists(latest_link):
        os.unlink(latest_link)
    os.symlink(now_str, latest_link, target_is_directory=True)

    queue_dir = os.path.join(log_dir, "queue")
    cov_dir = os.path.join(log_dir, "cov")
    error_dir = os.path.join(log_dir, "errors")
    meta_dir = os.path.join(log_dir, "metadata")
    rosbag_dir = os.path.join(log_dir, "rosbags")
    try:
        os.makedirs(queue_dir)
        os.makedirs(cov_dir)
        os.makedirs(error_dir)
        os.makedirs(meta_dir)
        os.makedirs(rosbag_dir)
    except FileExistsError:
        pass

    with open(os.path.join(log_dir, "cmd"), "w") as f:
        f.write(" ".join(sys.argv))

    with open(os.path.join(log_dir, "args"), "w") as f:
        json.dump(args.__dict__, f, indent=2)

    config = config.RuntimeConfig()
    config.method = args.method
    config.fuzz_mode = c.M_STARTOVER if args.startover else c.M_STATEFUL
    config.log_dir = log_dir
    config.queue_dir = queue_dir
    config.error_dir = error_dir
    config.cov_dir = cov_dir
    config.meta_dir = meta_dir
    config.rosbag_dir = rosbag_dir
    config.maxloop = args.maxloop
    config.interval = args.interval
    config.rospkg = args.ros_pkg
    config.rosnode = args.ros_node
    config.watchlist = args.watchlist

    selected_profile_name = target_profiles.resolve_profile_name(
        target_profile=args.target_profile,
        tb4=args.tb4_sitl,
        px4_v117=args.px4_v117_sitl,
        test_moveit=args.test_moveit,
        legacy_tb3=args.tb3_sitl,
    )
    selected_profile = None
    if selected_profile_name:
        selected_profile = target_profiles.load_profile(
            selected_profile_name,
            config.proj_root,
        )
        target_profiles.attach_profile_to_config(config, selected_profile)
        _, resolved_watchlist = target_profiles.write_profile_metadata(
            selected_profile,
            config.meta_dir,
        )
        config.watchlist = resolved_watchlist

    if args.schedule == "single":
        config.schedule = Campaign.RND_SINGLE
        config.seqlen = 1
    elif args.schedule == "sequence":
        config.schedule = Campaign.RND_SEQUENCE
        config.seqlen = args.seqlen
    elif args.schedule == "repeated":
        config.schedule = Campaign.RND_REPEATED
        config.seqlen = args.seqlen
        assert config.seqlen > 1

    config.repeat = args.repeat

    if args.determ_seed:
        config.seed = args.determ_seed
    else:
        config.seed = int(time.time())

    if args.wait:
        config.debug_wait = True
    else:
        config.debug_wait = False

    if selected_profile is not None and selected_profile.family == "px4":
        import px4_utils
        config.px4_sitl = True
        config.px4_ros = True
        config.use_mavlink = False
        config.exp_pgfuzz = False
        config.px4_mission_file = args.px4_mission
        config.flight_mode = "OFFBOARD"
    elif args.px4_sitl_ros:
        print("[legacy] --px4-sitl-ros uses the pre-Jazzy PX4 path; "
              "prefer --target-profile px4_v117_jazzy")
        config.px4_sitl = True
        config.px4_ros = True
        config.use_mavlink = False
        config.exp_pgfuzz = False
        from px4_msgs.msg import VehicleCommand
        import px4_utils
        config.px4_mission_file = args.px4_mission
        config.flight_mode = args.px4_flight_mode.upper()
    elif args.px4_sitl_mav:
        print("[legacy] --px4-sitl-mav uses the pre-Jazzy PX4 path")
        config.px4_sitl = True
        config.px4_ros = False
        config.use_mavlink = True
        config.flight_mode = args.px4_flight_mode.upper()
        config.exp_pgfuzz = False
        import px4_utils
    elif args.px4_sitl_pgfuzz:
        print("[legacy] --px4-sitl-pgfuzz uses the pre-Jazzy PX4 path")
        config.px4_sitl = True
        config.px4_ros = False
        config.use_mavlink = True
        config.exp_pgfuzz = True
        import px4_utils
        config.flight_mode = "LOITER"
    else:
        config.px4_sitl = False
        config.px4_ros = False
        config.exp_pgfuzz = False
        config.use_mavlink = False

    if selected_profile is not None and selected_profile.family == "turtlebot":
        config.tb4_sitl = True
        config.tb3_sitl = False
    elif args.tb3_sitl:
        config.tb3_sitl = True
    else:
        config.tb3_sitl = False

    if args.tb3_hitl:
        config.tb3_hitl = True
        config.tb3_uri = args.tb3_uri
    else:
        config.tb3_hitl = False

    if args.sros2:
        config.sros2 = True
        config.sros2_keystore = os.path.join(
            config.proj_root, "src", "fuzzing_keys"
        )
        config.sros2_enable = "true"
        config.sros2_strategy = "Enforce"
        config.sros2_enclave = "/fuzzer/sros2_node"
        config.rospkg = "sros2_node"
        config.rosnode = "sros2_node"
    else:
        config.sros2 = False

    if args.fuzz_seed:
        config.fuzz_seed = args.fuzz_seed
    else:
        config.fuzz_seed = None

    if args.no_cov:
        config.no_cov = True
    else:
        config.no_cov = False

    if args.persistent:
        config.persistent = True
    else:
        config.persistent = False

    if args.test_rcl:
        config.test_rcl = True

        valid_features = [
            "publisher",
            "subscriber",
            "service",
            "client",
            "node",
            "timer",
            "graph",
            "expand_topic_name",
            "guard_condition",
            "init",
            "time",
            "validate_topic_name",
            "wait",
        ]

        if args.rcl_api == "":
            print("--rcl-api not set. Assuming publisher")
            config.rcl_api = "publisher"
        elif args.rcl_api in valid_features:
            config.rcl_api = args.rcl_api
        else:
            print("[-] Invalid --rcl-api. Please check librcl_apis/")
            exit(1)

        valid_jobs = [
            "create_publisher",
            "create_subscriber",
            "create_node"
        ]

        # specify task (e.g., publisher, create_publisher, ...)
        # should be in sync with directory name containing target program
        if args.rcl_job == "":
            print("--rcl-job not set. Assuming create_publisher")
            config.test_rcl_job = "create_publisher"
        elif args.rcl_job in valid_jobs:
            config.test_rcl_job = args.rcl_job
        else:
            print("[-] --rcl-job is invalid or not supported.")
            exit(1)

        features_to_test = [
            config.rcl_api
        ]
        config.test_rcl_feature = features_to_test

        targets = ["py", "cpp"]# , "rs"]
        config.test_rcl_targets = targets

        # TODO:
        # - make sure rcl_lang_ws is present and compiled
        # - set path to the harness
        # - execute harness through harness.py
        #   - harness listens to data topic, executes each lang target w/ received data
        # - executor publishes msg, harness will do its job
        #   - ltrace trace dump will be generated for each lang target
        # - checker checks for inconsistency by parsing and cross-checking the trace dumps
    else:
        config.test_rcl = False

    if args.test_cli:
        config.test_cli = True
    else:
        config.test_cli = False

    if args.test_rosidl:
        # one target with multiple topics - why not
        config.test_rosidl = True

        lang = ["py", "cpp"]
        # don't need to search target
        # just make sure the harness is launched

        config.test_rosidl_lang = lang[0]
        config.schedule = Campaign.IDL_CHECK
    else:
        config.test_rosidl = False

    if selected_profile is not None and selected_profile.family == "moveit":
        from moveit_msgs.msg import Constraints, JointConstraint
        config.test_moveit = True
    elif args.test_moveit:
        from moveit_msgs.msg import Constraints, JointConstraint
        config.test_moveit = True
    else:
        config.test_moveit = False

    config.use_ulg = args.use_ulg

    config.exec_cmd = None
    if selected_profile is not None:
        ret = -1
    else:
        ret = config.find_package_metadata()

    if ret < 0:
        if args.exec_cmd == "px4":
            # PX4-specific
            px4_root = os.path.join(
                config.proj_root, "targets", "PX4-Autopilot"
            )
            px4_build_dir = os.path.join(px4_root, "build", "px4_sitl_rtps")
            sitl_script = os.path.join(px4_root, "Tools", "sitl_run.sh")
            sitl_opts = [
                os.path.join(px4_build_dir, "bin", "px4"),
                "none",
                "gazebo",
                "none",
                "none",
                px4_root,
                px4_build_dir,
            ]
            px4_cmd = "{} {}".format(sitl_script, " ".join(sitl_opts))
            config.exec_cmd = px4_cmd
        elif args.exec_cmd is not None:
            config.exec_cmd = args.exec_cmd
        else:
            config.exec_cmd = ""

    main(config)
