#!/usr/bin/env python3
"""
PX4 Bug回放工具 - 在PX4 SITL + Gazebo中重现fuzzer发现的bug

使用方法:
  1. 启动带GUI的PX4容器:
     docker run -it --rm --name px4_replay \
       -e DISPLAY="$DISPLAY" \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v /home/ymc/RoboFuzz/src:/robofuzz/src \
       -v /home/ymc/RoboFuzz/logs_px4:/robofuzz/src/logs \
       px4_fuzz bash

  2. 终端1 - 启动PX4 SITL + Gazebo:
     source /ros_entrypoint.sh
     PX4_SITL_WORLD=church /robofuzz/targets/PX4-Autopilot/Tools/sitl_run.sh \
       /robofuzz/targets/PX4-Autopilot/build/px4_sitl_rtps/bin/px4 \
       none gazebo iris none /robofuzz/targets/PX4-Autopilot \
       /robofuzz/targets/PX4-Autopilot/build/px4_sitl_rtps

  3. 终端2 - 启动micrortps_agent:
     source /ros_entrypoint.sh
     micrortps_agent -t UDP

  4. 终端3 - 运行回放:
     source /ros_entrypoint.sh
     cd /robofuzz/src
     python3 replay_px4_bug.py --case ekf_interleave
     python3 replay_px4_bug.py --case jerk_direction
     python3 replay_px4_bug.py --case wv_yrate_max
     python3 replay_px4_bug.py --case descent_overshoot
     python3 replay_px4_bug.py --list

  自定义序列:
     python3 replay_px4_bug.py --seq "0,0,0.5,0:12; 0,0,0.5,0.623:13; 0,0,0.5,0:75"
     格式: "x,y,z,r:count; x,y,z,r:count; ..."
"""

import sys
import os
import time
import argparse
import subprocess as sp

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pymavlink import mavutil


# ============================================================
# Bug Case 定义
# 格式: (x, y, z, r, count) — ManualControlSetpoint 的 4 个轴 + 重复次数
#   x: pitch/前后 [-1,1], y: roll/左右 [-1,1]
#   z: throttle [0,1] (0.5=悬停), r: yaw [-1,1]
# ============================================================
CASES = {
    "ekf_interleave": {
        "desc": "EKF Multi-Instance数据交错 (Bug1+2: 位置/姿态跳变)",
        "frame": "1779691499.821895",
        "seq": [
            (0.000, 0.000, 0.500, 0.000, 12),
            (0.000, 0.000, 0.500, 0.623, 13),
            (0.000, 0.000, 0.500, 0.000, 75),
        ],
        "expected_bugs": [
            "Velocity-position inconsistency (max ~74m)",
            "Attitude-angular velocity inconsistency (max 180deg)",
        ],
    },
    "jerk_direction": {
        "desc": "多轴方向突变触发Jerk超限 (Bug3: MPC_JERK_MAX)",
        "frame": "1779691539.491137",
        "seq": [
            (0.000, 0.000, 0.500, 0.000, 12),
            (0.000, 0.000, 0.500, 0.623, 13),
            (0.000, 0.000, 0.500, 0.000, 7),
            (0.112, -0.695, 0.268, 0.000, 25),
            (0.000, 0.000, 0.500, 0.000, 43),
        ],
        "expected_bugs": [
            "MPC_JERK_MAX violated: >250 m/s^3",
        ],
    },
    "wv_yrate_max": {
        "desc": "持续满偏触发Yaw Rate超限 (Bug4: WV_YRATE_MAX)",
        "frame": "1779691656.4440792",
        "seq": [
            (1.000, -1.000, 0.500, 1.000, 100),
        ],
        "expected_bugs": [
            "WV_YRATE_MAX violated: yaw rate 144 deg/s > 45 deg/s",
            "同时触发EKF instance切换",
        ],
    },
    "descent_overshoot": {
        "desc": "油门骤降触发下降速度超限 (Bug5+6)",
        "frame": "1779695052.2885876",
        "seq": [
            (-1.000, -1.000, 0.500, 0.000, 29),
            (-1.000, -1.000, 0.500, 0.723, 4),
            (-1.000, -0.775, 0.448, 0.723, 2),
            (-1.000, 0.948, 0.443, -0.723, 5),
            (-0.677, -0.775, 0.999, -0.379, 6),
            (-0.717, 0.162, 0.035, -0.213, 2),
            (-0.717, -1.000, 0.035, 0.000, 5),
            (-0.717, -1.000, 0.246, 0.000, 4),
            (0.112, -1.000, 0.246, 0.000, 6),
        ],
        "expected_bugs": [
            "MPC_Z_VEL_MAX_DN violated: >1.0 m/s",
            "MPC_ACC_DOWN_MAX violated: >3.0 m/s^2",
        ],
    },
    "baseline": {
        "desc": "正常悬停基准 (无bug, 用于对比)",
        "frame": None,
        "seq": [
            (0.000, 0.000, 0.500, 0.000, 100),
        ],
        "expected_bugs": [],
    },
}


class PX4BugReplayer:
    """通过MAVLink连接PX4 SITL，执行arm+takeoff后发送manual_control序列"""

    def __init__(self, connection_str="udpin:127.0.0.1:14550", rate_hz=10):
        self.connection_str = connection_str
        self.rate_hz = rate_hz
        self.interval = 1.0 / rate_hz
        self.master = None

    def connect(self):
        print(f"[*] 连接 MAVLink: {self.connection_str}")
        self.master = mavutil.mavlink_connection(self.connection_str)
        self.master.wait_heartbeat(timeout=30)
        if self.master.target_system == 0:
            raise RuntimeError("MAVLink heartbeat 超时，检查PX4是否已启动")
        print(f"[+] 已连接 (sysid={self.master.target_system})")

    def reset_drone(self):
        """重置无人机状态：DISARM → 重置Gazebo模型位姿 → 等待稳定"""
        m = self.master
        print("[*] 重置无人机状态...")

        # 1. 强制 DISARM (force=21196)
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 21196, 0, 0, 0, 0, 0
        )
        time.sleep(1)

        # 2. 重置 Gazebo 模型位姿
        try:
            sp.run(
                ["gz", "model", "-m", "iris",
                 "-x", "0", "-y", "0", "-z", "0.2",
                 "-R", "0", "-P", "0", "-Y", "0"],
                timeout=5, capture_output=True
            )
            print("[+] Gazebo 模型位姿已重置")
        except (sp.TimeoutExpired, FileNotFoundError) as e:
            print(f"[!] gz model 重置失败: {e}，尝试 MAVLink reboot")
            m.mav.command_long_send(
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                1, 0, 0, 0, 0, 0, 0
            )
            time.sleep(8)
            # 重新连接
            self.master.close()
            self.connect()
            return

        # 3. 等待 PX4 识别到 landed 状态
        time.sleep(3)
        print("[+] 无人机已重置，可以重新起飞")

    def arm_and_takeoff(self, alt=5.0):
        """设置POSCTL模式 → 解锁 → 油门悬停等待起飞"""
        m = self.master
        # 设置 POSCTL 模式 (main_mode=3, sub_mode=0)
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            209, 3, 0, 0, 0, 0, 0  # base_mode=209(armed+custom), main=3
        )
        time.sleep(1)

        # ARM
        print("[*] 解锁...")
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0
        )
        ack = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
        if not ack or ack.result != 0:
            raise RuntimeError(f"ARM 失败: {ack}")
        print("[+] 已解锁")

        # 发送悬停油门让飞机起飞
        print(f"[*] 起飞中 (目标高度 ~{alt}m)...")
        for _ in range(int(5 * self.rate_hz)):  # 5秒爬升
            self._send_rc(0, 0, 0.7, 0)  # z=0.7 上升
            time.sleep(self.interval)
        # 稳定悬停2秒
        for _ in range(int(2 * self.rate_hz)):
            self._send_rc(0, 0, 0.5, 0)
            time.sleep(self.interval)
        print("[+] 起飞完成，开始回放序列")

    def _send_rc(self, x, y, z, r):
        """发送 MANUAL_CONTROL 消息"""
        self.master.mav.manual_control_send(
            self.master.target_system,
            int(x * 1000),   # pitch [-1000, 1000]
            int(y * 1000),   # roll
            int(z * 1000),   # throttle [0, 1000]
            int(r * 1000),   # yaw
            0                # buttons
        )

    def replay_sequence(self, seq, desc=""):
        """回放一组 (x, y, z, r, count) 序列"""
        if desc:
            print(f"\n{'='*60}")
            print(f"  回放: {desc}")
            print(f"{'='*60}")

        total_msgs = sum(s[4] for s in seq)
        sent = 0
        for i, (x, y, z, r, count) in enumerate(seq):
            print(f"  段{i+1}: x={x:.3f} y={y:.3f} z={z:.3f} r={r:.3f} × {count}")
            for _ in range(count):
                self._send_rc(x, y, z, r)
                sent += 1
                time.sleep(self.interval)
        print(f"\n[+] 回放完成: 共发送 {sent}/{total_msgs} 条消息")

    def hold_and_land(self):
        """回放后悬停观察，然后降落"""
        print("[*] 悬停观察 3 秒...")
        for _ in range(int(3 * self.rate_hz)):
            self._send_rc(0, 0, 0.5, 0)
            time.sleep(self.interval)
        print("[*] 降落中...")
        m = self.master
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(5)
        print("[+] 降落指令已发送")

    def close(self):
        if self.master:
            self.master.close()


def parse_custom_seq(seq_str):
    """解析自定义序列字符串: "x,y,z,r:count; x,y,z,r:count; ..." """
    segments = []
    for part in seq_str.split(";"):
        part = part.strip()
        if not part:
            continue
        vals, count = part.rsplit(":", 1)
        x, y, z, r = [float(v) for v in vals.split(",")]
        segments.append((x, y, z, r, int(count)))
    return segments


def main():
    parser = argparse.ArgumentParser(
        description="PX4 Bug回放工具 - 重现fuzzer发现的bug",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--case", choices=list(CASES.keys()),
                        help="选择预定义的bug case")
    parser.add_argument("--seq", type=str,
                        help='自定义序列 "x,y,z,r:count; ..."')
    parser.add_argument("--list", action="store_true",
                        help="列出所有可用case")
    parser.add_argument("--conn", default="udpin:127.0.0.1:14550",
                        help="MAVLink连接字符串 (默认: udpin:127.0.0.1:14550)")
    parser.add_argument("--rate", type=int, default=10,
                        help="发送频率Hz (默认: 10)")
    parser.add_argument("--no-land", action="store_true",
                        help="回放后不自动降落")
    parser.add_argument("--reset", action="store_true", default=True,
                        help="回放前重置无人机状态 (默认开启)")
    parser.add_argument("--no-reset", action="store_true",
                        help="跳过回放前的重置")
    args = parser.parse_args()

    if args.list:
        print("\n可用 Bug Cases:")
        print("-" * 60)
        for name, info in CASES.items():
            bugs = ", ".join(info["expected_bugs"]) if info["expected_bugs"] else "无"
            print(f"  {name:20s} — {info['desc']}")
            print(f"  {'':20s}   预期触发: {bugs}")
        print()
        return

    if not args.case and not args.seq:
        parser.error("必须指定 --case 或 --seq")

    # 解析序列
    if args.case:
        case = CASES[args.case]
        seq = case["seq"]
        desc = case["desc"]
        print(f"\n[Case] {args.case}: {desc}")
        if case["expected_bugs"]:
            print(f"[预期Bug] {', '.join(case['expected_bugs'])}")
        if case["frame"]:
            print(f"[原始Frame] {case['frame']}")
    else:
        seq = parse_custom_seq(args.seq)
        desc = "自定义序列"

    # 执行回放
    replayer = PX4BugReplayer(connection_str=args.conn, rate_hz=args.rate)
    try:
        replayer.connect()
        if not args.no_reset:
            replayer.reset_drone()
        replayer.arm_and_takeoff()
        replayer.replay_sequence(seq, desc)
        if not args.no_land:
            replayer.hold_and_land()
        else:
            print("[*] --no-land: 跳过降落，飞机仍在空中")
    except KeyboardInterrupt:
        print("\n[!] 用户中断")
    except Exception as e:
        print(f"\n[!] 错误: {e}")
        sys.exit(1)
    finally:
        replayer.close()


if __name__ == "__main__":
    main()

