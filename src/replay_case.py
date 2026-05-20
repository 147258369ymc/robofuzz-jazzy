#!/usr/bin/env python3
"""
Case回放工具 - 在Gazebo中重现fuzzer发现的典型case

使用方法:
  1. 先启动Gazebo仿真环境（带GUI）:
     docker exec -it <container> bash
     source /robofuzz/targets/turtlebot3_ws/install/setup.bash
     DISPLAY=:1 TURTLEBOT3_MODEL=burger ros2 launch turtlebot3_gazebo empty_world.launch.py

  2. 在另一个终端运行本脚本:
     docker exec -it <container> bash
     source /robofuzz/targets/turtlebot3_ws/install/setup.bash
     python3 /robofuzz/src/replay_case.py --case <case_name>

  可用case:
    baseline       - [+0.22, 0] x5 (正常前进，无错误)
    coupling       - [+0.22, -2.84] x5 (运动学耦合bug)
    pure_rotation  - [0, +2.84, 0, -2.84, ...] x5 (角速度交替)
    reversal       - [+0.22, +2.84] -> [+0.22, -2.84] x4 (角速度突变)
    backward       - [-0.22, 0] x5 (纯后退)
    minimal        - [+0.22,0]x3 -> [+0.22,+1.82] -> [+0.22,0] (最小变异)
    cmd_track      - [+0.22,0]x3 -> [-2.84,+1.82] -> [+0.22,0] (跟踪失败)
    extreme        - [+1.82,-10] -> [+0.22,-2.83] -> ... (极端变异)
    custom         - 自定义序列 (通过 --seq 参数)

  额外选项:
    --interval N   - 消息间隔秒数 (默认1.0)
    --repeat N     - 重复执行次数 (默认1)
    --reset        - 每次执行前重置机器人位置
    --seq          - 自定义序列, 格式: "lin,ang;lin,ang;..."
"""

import sys
import time
import argparse
import subprocess

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_srvs.srv import Empty
except ImportError:
    print("ERROR: ROS2 not sourced. Run: source /opt/ros/foxy/setup.bash")
    sys.exit(1)


# ============================================================
# 预定义Case
# ============================================================
CASES = {
    "baseline": {
        "desc": "纯前进 (无错误基准)",
        "seq": [(0.22, 0.0)] * 5,
    },
    "coupling": {
        "desc": "线速度+最大角速度 (运动学耦合bug, 1950 errors)",
        "seq": [(0.22, -2.84)] * 5,
    },
    "coupling_pos": {
        "desc": "线速度+最大正角速度 (运动学耦合bug, 1855 errors)",
        "seq": [(0.22, 2.84)] * 5,
    },
    "pure_rotation": {
        "desc": "角速度交替 ±2.84 (117 errors)",
        "seq": [(0.0, 2.84), (0.0, -2.84), (0.0, 2.84), (0.0, -2.84), (0.0, 2.84)],
    },
    "reversal": {
        "desc": "角速度方向突变 +2.84 -> -2.84 (1206 errors)",
        "seq": [(0.22, 2.84), (0.22, -2.84), (0.22, -2.84), (0.22, -2.84), (0.22, -2.84)],
    },
    "backward": {
        "desc": "纯后退 (556 errors, 疑似不对称bug)",
        "seq": [(-0.22, 0.0)] * 5,
    },
    "minimal": {
        "desc": "最小变异: 仅第4条加入角速度 (11 errors)",
        "seq": [(0.22, 0.0), (0.22, 0.0), (0.22, 0.0), (0.22, 1.82), (0.22, 0.0)],
    },
    "cmd_track": {
        "desc": "超限后恢复失败 (133 errors, cmd_vel tracking)",
        "seq": [(0.22, 0.0), (0.22, 0.0), (0.22, 0.0), (-2.84, 1.82), (0.22, 0.0)],
    },
    "extreme": {
        "desc": "极端变异 (7340 errors)",
        "seq": [(1.82, -10.0), (0.22, -2.83), (1.83, -0.001), (0.0, -2.84), (-10.0, 0.0)],
    },
    "zigzag_lin": {
        "desc": "线速度前后交替 (19 errors)",
        "seq": [(0.22, 0.0), (-0.22, 0.0), (0.22, 0.0), (-0.22, 0.0), (0.22, 0.0)],
    },
    "gradual": {
        "desc": "渐进加速 (0 errors, 安全模式)",
        "seq": [(0.0, 0.0), (0.055, 0.0), (0.11, 0.0), (0.165, 0.0), (0.22, 0.0)],
    },
}


# ============================================================
# 回放器
# ============================================================
class CaseReplayer(Node):
    def __init__(self):
        super().__init__("case_replayer")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.reset_client = self.create_client(Empty, "/reset_simulation")
        time.sleep(0.5)  # wait for publisher to connect

    def reset_robot(self):
        """Reset Gazebo simulation to initial state"""
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Reset service not available, trying gz command...")
            subprocess.run(
                ["gz", "service", "-s", "/world/default/control",
                 "--reqtype", "gz.msgs.WorldControl",
                 "--reptype", "gz.msgs.Boolean",
                 "--req", "reset: {all: true}"],
                capture_output=True, timeout=5
            )
            time.sleep(2.0)
            return

        req = Empty.Request()
        future = self.reset_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        time.sleep(2.0)  # wait for reset to settle

    def send_stop(self):
        """Send zero velocity to stop the robot"""
        msg = Twist()
        self.pub.publish(msg)

    def replay_sequence(self, seq, interval=1.0):
        """Replay a sequence of (linear_x, angular_z) commands"""
        for i, (lin_x, ang_z) in enumerate(seq):
            msg = Twist()
            msg.linear.x = float(lin_x)
            msg.angular.z = float(ang_z)
            print(f"  [{i+1}/{len(seq)}] Publishing: "
                  f"linear.x={lin_x:.4f}, angular.z={ang_z:.4f}")
            self.pub.publish(msg)
            time.sleep(interval)

        # Send stop after sequence
        print(f"  [STOP] Sending zero velocity")
        self.send_stop()


def parse_custom_seq(seq_str):
    """Parse custom sequence string: 'lin,ang;lin,ang;...'"""
    pairs = seq_str.strip().split(";")
    seq = []
    for pair in pairs:
        parts = pair.strip().split(",")
        if len(parts) != 2:
            print(f"ERROR: Invalid pair '{pair}', expected 'linear,angular'")
            sys.exit(1)
        seq.append((float(parts[0]), float(parts[1])))
    return seq


def main():
    parser = argparse.ArgumentParser(description="Replay fuzzer case in Gazebo")
    parser.add_argument("--case", type=str, default="coupling",
                        help=f"Case name: {', '.join(CASES.keys())}, or 'custom'")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="Interval between messages (seconds)")
    parser.add_argument("--repeat", type=int, default=1,
                        help="Number of times to repeat")
    parser.add_argument("--reset", action="store_true",
                        help="Reset simulation before each run")
    parser.add_argument("--seq", type=str, default=None,
                        help="Custom sequence: 'lin,ang;lin,ang;...'")
    parser.add_argument("--list", action="store_true",
                        help="List all available cases")
    args = parser.parse_args()

    if args.list:
        print("\nAvailable cases:\n")
        for name, info in CASES.items():
            seq_str = " -> ".join([f"({l:.2f}, {a:.2f})" for l, a in info["seq"]])
            print(f"  {name:15s} {info['desc']}")
            print(f"  {'':15s} {seq_str}\n")
        return

    # Determine sequence
    if args.case == "custom" or args.seq:
        if not args.seq:
            print("ERROR: --seq required for custom case")
            sys.exit(1)
        seq = parse_custom_seq(args.seq)
        desc = f"Custom: {args.seq}"
    elif args.case in CASES:
        seq = CASES[args.case]["seq"]
        desc = CASES[args.case]["desc"]
    else:
        print(f"ERROR: Unknown case '{args.case}'")
        print(f"Available: {', '.join(CASES.keys())}")
        sys.exit(1)

    # Initialize ROS2
    rclpy.init()
    replayer = CaseReplayer()

    print(f"\n{'='*60}")
    print(f"Case: {args.case}")
    print(f"Description: {desc}")
    print(f"Sequence length: {len(seq)}")
    print(f"Interval: {args.interval}s")
    print(f"Repeat: {args.repeat}x")
    print(f"{'='*60}\n")

    try:
        for run in range(args.repeat):
            if args.repeat > 1:
                print(f"\n--- Run {run+1}/{args.repeat} ---")

            if args.reset:
                print("  Resetting simulation...")
                replayer.reset_robot()

            replayer.replay_sequence(seq, interval=args.interval)

            # Wait to observe effects
            print(f"  Waiting 3s to observe result...")
            time.sleep(3.0)

    except KeyboardInterrupt:
        print("\n\nInterrupted! Sending stop command...")
        replayer.send_stop()

    finally:
        replayer.send_stop()
        replayer.destroy_node()
        rclpy.shutdown()

    print("\nDone.")


if __name__ == "__main__":
    main()
