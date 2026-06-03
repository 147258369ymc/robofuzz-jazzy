#!/usr/bin/env python3
"""
PX4 Bug回放工具 (ROS2/OFFBOARD 模式) — 在PX4 SITL中重现fuzzer发现的bug

与 replay_px4_bug.py (MAVLink/POSCTL) 不同，本脚本通过 ROS2 DDS 发送
TrajectorySetpoint (velocity mode) 直接控制PX4 OFFBOARD模式。

使用方法:
  1. 启动容器:
     docker run -it --rm --name px4_replay \
       -e DISPLAY="$DISPLAY" \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v /home/ymc/RoboFuzz/src:/robofuzz/src \
       -v /home/ymc/RoboFuzz/logs_px4:/robofuzz/logs_px4 \
       robofuzz:latest bash

  2. 终端1 — 启动 PX4 SITL + Gazebo:
     source /ros_entrypoint.sh
     PX4_SITL_WORLD=church /robofuzz/targets/PX4-Autopilot/Tools/sitl_run.sh \
       /robofuzz/targets/PX4-Autopilot/build/px4_sitl_rtps/bin/px4 \
       none gazebo iris none /robofuzz/targets/PX4-Autopilot \
       /robofuzz/targets/PX4-Autopilot/build/px4_sitl_rtps

  3. 终端2 — 启动 microRTPS bridge:
     source /ros_entrypoint.sh
     micrortps_agent -t UDP

  4. 终端3 — 运行回放:
     source /ros_entrypoint.sh
     cd /robofuzz/src
     python3 replay_px4_ros_bug.py --list
     python3 replay_px4_ros_bug.py --case fd_fail_extreme
     python3 replay_px4_ros_bug.py --case jerk_vz_reversal
     python3 replay_px4_ros_bug.py --from-log ../logs_px4/20260602-123026 --iter 1780405037

  从日志直接回放:
     python3 replay_px4_ros_bug.py --from-log <experiment_dir> --iter <timestamp>
"""

import sys
import os
import time
import math
import argparse
import pickle
from collections import OrderedDict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    Timesync,
    VehicleCommand,
    TrajectorySetpoint,
    OffboardControlMode,
)


# ============================================================
# Bug Case 定义
# 格式: list of (vx, vy, vz, yawspeed, count)
#   vx/vy: m/s 水平速度, vz: m/s (NED, 负=爬升)
#   yawspeed: rad/s, count: 重复次数 (每次间隔 interval)
# ============================================================
CASES = {
    "fd_fail_extreme": {
        "desc": "RC1: 极端输入校验缺失 → 灾难性翻转 (FD_FAIL_R)",
        "frame": "1780405037.0285208",
        "interval": 0.02,
        "seq": [
            # Phase 1-5: 高速爬升+旋转建立状态
            (11.81, 6.0, -3.0, 2.0, 5),
            (-11.81, 6.0, -3.0, 2.0, 6),
            (6.0, 6.0, -3.0, 2.0, 3),
            (6.0, 8.84, -3.0, 2.0, 12),
            (6.0, 6.0, -3.0, 2.0, 23),
            # Phase 6: 极端突变 (根因触发点)
            (-1000.0, 0.0, -2.36, -3.49, 1),
            # Phase 7-9: 恢复后的快速方向变化
            (-8.88, 2.59, -4.33, 3.08, 1),
            (50.62, 5.12, -2.55, 1.09, 1),
            (7.75, -5.16, -4.35, -2.95, 1),
            # Phase 10+: 持续yaw振荡
            (7.75, -5.16, -4.35, 1.79, 1),
            (7.75, -5.16, -4.35, -2.05, 1),
            (7.75, -5.16, -4.35, 3.19, 1),
            (7.75, -5.16, -4.35, 0.46, 195),
        ],
        "expected_bugs": [
            "FD_FAIL_R: roll > 65° (实测70°)",
            "ACT_SAT_MAX: 电机饱和0.7s",
            "MC_ROLLRATE_MAX: 258 deg/s",
            "Tilt: 74.3°",
        ],
    },
    "fd_fail_no_extreme": {
        "desc": "RC1验证: 移除极端值(-1000/50.62)，仅保留合理范围输入",
        "frame": "1780405037.0285208 (sanitized)",
        "interval": 0.02,
        "seq": [
            (11.81, 6.0, -3.0, 2.0, 5),
            (-11.81, 6.0, -3.0, 2.0, 6),
            (6.0, 6.0, -3.0, 2.0, 3),
            (6.0, 8.84, -3.0, 2.0, 12),
            (6.0, 6.0, -3.0, 2.0, 23),
            # Phase 6: 用 clamp 后的值替代 -1000
            (-12.0, 0.0, -2.36, -3.49, 1),
            (-8.88, 2.59, -4.33, 3.08, 1),
            # Phase 8: 用 clamp 后的值替代 50.62
            (12.0, 5.12, -2.55, 1.09, 1),
            (7.75, -5.16, -4.35, -2.95, 1),
            (7.75, -5.16, -4.35, 1.79, 1),
            (7.75, -5.16, -4.35, -2.05, 1),
            (7.75, -5.16, -4.35, 3.19, 1),
            (7.75, -5.16, -4.35, 0.46, 195),
        ],
        "expected_bugs": [
            "对比: 如果无FD_FAIL_R → 极端值是必要触发条件",
            "如果仍有FD_FAIL_R → 根因是快速方向变化+yaw振荡",
        ],
    },
    "jerk_vz_reversal": {
        "desc": "RC2: vz方向突变触发JERK+ACC超限 (最小化触发序列)",
        "frame": "1780404069.5243006",
        "interval": 0.02,
        "seq": [
            # 恒速水平飞行 + 最大爬升
            (12.0, 0.0, -5.0, 0.0, 87),
            # vz减小
            (12.0, 0.0, -3.33, 0.0, 15),
            # 突然下降! (根因触发点: vz从-5→+1)
            (12.0, 0.0, 1.0, 0.0, 15),
            # 恢复爬升
            (12.0, 0.0, -5.0, 0.0, 133),
        ],
        "expected_bugs": [
            "MPC_JERK_MAX: 19.8 m/s³ (限制10.0)",
            "MPC_ACC_DOWN_MAX: 9.27 m/s² (限制4.5)",
            "MPC_ACC_UP_MAX: -7.03 m/s² (限制-5.5)",
            "ACT_SAT_MAX: 0.3s",
        ],
    },
    "jerk_vz_gentle": {
        "desc": "RC2验证: 平滑vz过渡(线性插值)而非突变",
        "frame": "1780404069 (smoothed)",
        "interval": 0.02,
        "seq": [
            (12.0, 0.0, -5.0, 0.0, 87),
            # 10步线性过渡 -5 → +1 (而非突变)
            (12.0, 0.0, -4.4, 0.0, 3),
            (12.0, 0.0, -3.8, 0.0, 3),
            (12.0, 0.0, -3.2, 0.0, 3),
            (12.0, 0.0, -2.6, 0.0, 3),
            (12.0, 0.0, -2.0, 0.0, 3),
            (12.0, 0.0, -1.4, 0.0, 3),
            (12.0, 0.0, -0.8, 0.0, 3),
            (12.0, 0.0, -0.2, 0.0, 3),
            (12.0, 0.0, 0.4, 0.0, 3),
            (12.0, 0.0, 1.0, 0.0, 15),
            (12.0, 0.0, -5.0, 0.0, 123),
        ],
        "expected_bugs": [
            "对比: 如果无JERK超限 → 突变是必要条件",
            "PX4的轨迹规划器无法处理离散setpoint跳变",
        ],
    },
    "rate_high_speed_maneuver": {
        "desc": "RC3: 高速机动PID过冲 (pitch rate超限)",
        "frame": "1780403771",
        "interval": 0.02,
        "seq": [
            # 全速前进+侧移+爬升
            (12.0, -5.48, -4.05, 0.0, 80),
            # 转向最大对角速度
            (12.0, 12.0, 0.0, 0.0, 28),
            # 急刹+反向 (根因触发点)
            (-3.23, 12.0, 0.0, 0.0, 8),
            # 恢复全速
            (12.0, 12.0, 0.0, 0.0, 134),
        ],
        "expected_bugs": [
            "MC_PITCHRATE_MAX: 237 deg/s (限制225)",
            "MPC_ACC_DOWN_MAX: 8.81 m/s²",
            "Tilt: ~50°",
        ],
    },
    "rate_zero_input": {
        "desc": "RC3验证: 零输入是否触发角速率超限 (悬停振荡)",
        "frame": "1780313081 (exp1)",
        "interval": 0.02,
        "seq": [
            (0.0, 0.0, 0.0, 0.0, 250),
        ],
        "expected_bugs": [
            "如果触发PITCHRATE → 说明控制器状态初始化问题",
            "如果无bug → 需要先有高速飞行建立残余状态",
        ],
    },
    "baseline_hover": {
        "desc": "基准: 稳定悬停 (无bug预期)",
        "frame": None,
        "interval": 0.02,
        "seq": [
            (0.0, 0.0, 0.0, 0.0, 250),
        ],
        "expected_bugs": [],
    },
}


class PX4RosReplayer(Node):
    """通过ROS2 DDS控制PX4 OFFBOARD模式，发送TrajectorySetpoint序列"""

    def __init__(self, interval=0.02):
        super().__init__("px4_ros_replayer")
        self.interval = interval
        self.ts = 0

        # Subscribers
        self.sub_timesync = self.create_subscription(
            Timesync, "Timesync_PubSubTopic", self._timesync_cb, 10
        )

        # Publishers
        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode, "/OffboardControlMode_PubSubTopic", 10
        )
        self.pub_vehicle_cmd = self.create_publisher(
            VehicleCommand, "/VehicleCommand_PubSubTopic", 10
        )
        self.pub_trajectory = self.create_publisher(
            TrajectorySetpoint, "/TrajectorySetpoint_PubSubTopic", 10
        )

    def _timesync_cb(self, msg):
        self.ts = msg.timestamp

    def _spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.01)

    def _publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.ts
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.pub_offboard_mode.publish(msg)

    def _publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.timestamp = self.ts
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_vehicle_cmd.publish(msg)

    def _send_trajectory(self, vx, vy, vz, yawspeed):
        """发送velocity模式TrajectorySetpoint (x/y/z=NaN表示不用位置控制)"""
        msg = TrajectorySetpoint()
        msg.timestamp = self.ts
        msg.x = float('nan')
        msg.y = float('nan')
        msg.z = float('nan')
        msg.yaw = float('nan')
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.vz = float(vz)
        msg.yawspeed = float(yawspeed)
        msg.acceleration = [0.0, 0.0, 0.0]
        msg.jerk = [0.0, 0.0, 0.0]
        msg.thrust = [0.0, 0.0, 0.0]
        self.pub_trajectory.publish(msg)

    def prepare_flight(self):
        """OFFBOARD起飞流程: 发送setpoint流 → 设置模式 → ARM → 等待爬升"""
        print("[*] 建立setpoint流 (OFFBOARD模式需要先接收setpoint)...")
        takeoff_sp = TrajectorySetpoint()
        takeoff_sp.x = 0.0
        takeoff_sp.y = 0.0
        takeoff_sp.z = -8.0  # NED: 8m above ground

        # 发送50条位置setpoint (5s) 让PX4接受OFFBOARD切换
        for i in range(50):
            self._spin_once()
            msg = OffboardControlMode()
            msg.timestamp = self.ts
            msg.position = True
            msg.velocity = False
            msg.acceleration = False
            msg.attitude = False
            msg.body_rate = False
            self.pub_offboard_mode.publish(msg)
            takeoff_sp.timestamp = self.ts
            self.pub_trajectory.publish(takeoff_sp)
            time.sleep(0.1)

        # 切换到OFFBOARD模式
        print("[*] 切换OFFBOARD模式...")
        self._spin_once()
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0, param2=6.0  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        )
        time.sleep(0.5)

        # ARM
        print("[*] 解锁...")
        self._spin_once()
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0
        )
        time.sleep(1)

        # 等待爬升到目标高度 (继续发送位置setpoint)
        print("[*] 爬升中 (等待到达8m)...")
        for i in range(80):
            self._spin_once()
            msg = OffboardControlMode()
            msg.timestamp = self.ts
            msg.position = True
            msg.velocity = False
            msg.acceleration = False
            msg.attitude = False
            msg.body_rate = False
            self.pub_offboard_mode.publish(msg)
            takeoff_sp.timestamp = self.ts
            self.pub_trajectory.publish(takeoff_sp)
            time.sleep(0.1)

        print("[+] 起飞完成，切换到velocity模式开始回放")
        time.sleep(0.5)

    def replay_sequence(self, seq, interval=None, desc=""):
        """回放 (vx, vy, vz, yawspeed, count) 序列"""
        if interval is None:
            interval = self.interval
        if desc:
            print(f"\n{'='*60}")
            print(f"  回放: {desc}")
            print(f"{'='*60}")

        total_msgs = sum(s[4] for s in seq)
        total_time = total_msgs * interval
        print(f"  总消息数: {total_msgs}, 预计时间: {total_time:.1f}s")
        print()

        sent = 0
        for i, (vx, vy, vz, yawspeed, count) in enumerate(seq):
            # 标记极端值
            extreme = ""
            if abs(vx) > 50 or abs(vy) > 50 or abs(vz) > 10:
                extreme = " ⚠ EXTREME"
            print(f"  段{i+1:2d}: vx={vx:8.2f} vy={vy:7.2f} "
                  f"vz={vz:6.2f} yaw={yawspeed:6.2f} × {count}{extreme}")

            for _ in range(count):
                self._spin_once()
                self._publish_offboard_control_mode()
                self._send_trajectory(vx, vy, vz, yawspeed)
                sent += 1
                time.sleep(interval)

        print(f"\n[+] 回放完成: {sent}/{total_msgs} 条, {sent*interval:.1f}s")

    def hold_and_land(self):
        """回放后悬停观察，然后降落"""
        print("[*] 悬停观察 3s...")
        for _ in range(int(3.0 / self.interval)):
            self._spin_once()
            self._publish_offboard_control_mode()
            self._send_trajectory(0.0, 0.0, 0.0, 0.0)
            time.sleep(self.interval)

        # 缓慢下降
        print("[*] 缓降中...")
        for _ in range(int(5.0 / self.interval)):
            self._spin_once()
            self._publish_offboard_control_mode()
            self._send_trajectory(0.0, 0.0, 0.5, 0.0)  # vz=+0.5 = 缓慢下降(NED)
            time.sleep(self.interval)

        # DISARM
        self._spin_once()
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0
        )
        print("[+] 已降落并锁定")


def load_sequence_from_log(experiment_dir, iteration_ts):
    """从实验日志的queue/目录加载pickle格式的输入序列"""
    queue_dir = os.path.join(experiment_dir, "queue")
    if not os.path.isdir(queue_dir):
        raise FileNotFoundError(f"Queue目录不存在: {queue_dir}")

    # 找到匹配的queue文件
    prefix = f"msg-{iteration_ts}-"
    matching = sorted(f for f in os.listdir(queue_dir) if f.startswith(prefix))

    if not matching:
        raise FileNotFoundError(
            f"未找到匹配的queue文件 (prefix={prefix})\n"
            f"  尝试模糊匹配..."
        )

    print(f"[*] 从日志加载: {len(matching)} 条消息")

    # 读取pickle文件
    raw_msgs = []
    for qf in matching:
        with open(os.path.join(queue_dir, qf), 'rb') as f:
            d = pickle.load(f)
        raw_msgs.append(d)

    # 转换为 (vx, vy, vz, yawspeed, count) 格式 (合并相同的连续命令)
    seq = []
    prev = None
    count = 0
    for d in raw_msgs:
        cur = (
            round(d.get('vx', 0.0), 4),
            round(d.get('vy', 0.0), 4),
            round(d.get('vz', 0.0), 4),
            round(d.get('yawspeed', 0.0), 4),
        )
        if cur == prev:
            count += 1
        else:
            if prev is not None:
                seq.append((*prev, count))
            prev = cur
            count = 1
    if prev is not None:
        seq.append((*prev, count))

    # 显示统计
    vx_vals = [d.get('vx', 0) for d in raw_msgs]
    vy_vals = [d.get('vy', 0) for d in raw_msgs]
    vz_vals = [d.get('vz', 0) for d in raw_msgs]
    print(f"  vx: [{min(vx_vals):.2f}, {max(vx_vals):.2f}]")
    print(f"  vy: [{min(vy_vals):.2f}, {max(vy_vals):.2f}]")
    print(f"  vz: [{min(vz_vals):.2f}, {max(vz_vals):.2f}]")
    print(f"  Phases: {len(seq)}")

    # 显示对应的error
    err_file = os.path.join(experiment_dir, "errors", f"error-{iteration_ts}")
    if os.path.exists(err_file):
        print(f"\n[Bug报告]:")
        content = open(err_file).read().strip()
        if len(content) > 500:
            print(f"  {content[:500]}...")
        else:
            print(f"  {content}")
    print()

    return seq


def parse_custom_seq(seq_str):
    """解析自定义序列: "vx,vy,vz,yaw:count; vx,vy,vz,yaw:count; ..." """
    segments = []
    for part in seq_str.split(";"):
        part = part.strip()
        if not part:
            continue
        vals, count = part.rsplit(":", 1)
        vx, vy, vz, yaw = [float(v) for v in vals.split(",")]
        segments.append((vx, vy, vz, yaw, int(count)))
    return segments


def main():
    parser = argparse.ArgumentParser(
        description="PX4 ROS2 OFFBOARD Bug回放工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--case", choices=list(CASES.keys()),
                        help="选择预定义的bug case")
    parser.add_argument("--seq", type=str,
                        help='自定义序列 "vx,vy,vz,yaw:count; ..."')
    parser.add_argument("--from-log", type=str,
                        help="从实验日志目录加载 (需配合--iter)")
    parser.add_argument("--iter", type=str,
                        help="迭代timestamp (配合--from-log)")
    parser.add_argument("--list", action="store_true",
                        help="列出所有可用case")
    parser.add_argument("--interval", type=float, default=None,
                        help="发送间隔秒 (默认: case自带或0.02)")
    parser.add_argument("--no-land", action="store_true",
                        help="回放后不自动降落")
    parser.add_argument("--no-takeoff", action="store_true",
                        help="跳过起飞(假设已在空中)")
    args = parser.parse_args()

    if args.list:
        print("\n可用 Bug Cases (ROS2 OFFBOARD velocity 模式):")
        print("-" * 70)
        for name, info in CASES.items():
            total = sum(s[4] for s in info["seq"])
            t = total * info["interval"]
            print(f"\n  {name}")
            print(f"    {info['desc']}")
            print(f"    消息数: {total}, 时长: {t:.1f}s")
            if info["expected_bugs"]:
                for bug in info["expected_bugs"]:
                    print(f"    → {bug}")
        print()
        return

    if not args.case and not args.seq and not args.from_log:
        parser.error("必须指定 --case, --seq, 或 --from-log + --iter")

    # 解析序列
    if args.case:
        case = CASES[args.case]
        seq = case["seq"]
        interval = args.interval or case["interval"]
        desc = case["desc"]
        print(f"\n[Case] {args.case}: {desc}")
        if case["expected_bugs"]:
            print(f"[预期Bug]")
            for b in case["expected_bugs"]:
                print(f"  → {b}")
        if case["frame"]:
            print(f"[原始迭代] {case['frame']}")
    elif args.from_log:
        if not args.iter:
            parser.error("--from-log 需要配合 --iter 指定迭代timestamp")
        seq = load_sequence_from_log(args.from_log, args.iter)
        interval = args.interval or 0.02
        desc = f"日志回放: {args.iter}"
    else:
        seq = parse_custom_seq(args.seq)
        interval = args.interval or 0.02
        desc = "自定义序列"

    # 初始化ROS2
    rclpy.init(args=None)
    replayer = PX4RosReplayer(interval=interval)

    try:
        if not args.no_takeoff:
            replayer.prepare_flight()
        replayer.replay_sequence(seq, interval=interval, desc=desc)
        if not args.no_land:
            replayer.hold_and_land()
        else:
            print("[*] --no-land: 飞机仍在空中")
    except KeyboardInterrupt:
        print("\n[!] 用户中断")
    except Exception as e:
        print(f"\n[!] 错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        replayer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
