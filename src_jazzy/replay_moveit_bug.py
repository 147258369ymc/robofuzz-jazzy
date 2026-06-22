#!/usr/bin/env python3
"""
MoveIt2 Panda Bug 回放工具 (replay_moveit_bug.py)

与 PX4 回放 (replay_px4_bug.py / replay_px4_ros_bug.py) 的根本区别：
  PX4 的 bug 表现为可见的飞行失控（翻转/坠机），靠 Gazebo 肉眼即可确认。
  MoveIt 的 bug 是 **录制状态数据里的语义违例**（TOPP-RA 规划速度超限、
  规划下发位置越限、声称成功却没到位），靠 oracle 读取 4 个状态话题
  (/joint_states, /panda_arm_controller/state, /move_action/_action/status,
   /motion_plan_request) 计算得出，肉眼在 RViz 里通常 **看不出来**。

因此本工具提供两种回放模式：

  1) offline（离线复检，确定性，推荐先用）：
     直接读取该 bug 当时录下的 rosbag，重新跑一遍 oracle。
     一定能 100% 复现分析报告里的那条 error —— 用来核对"分析说的对不对"。

  2) live（在线重放，可在 RViz 观察）：
     向正在运行的 MoveIt 栈按原始顺序重新发送同一组目标位姿，
     录制状态话题到新 rosbag，再跑 oracle。
     可在 RViz 里看到机械臂的实际运动，但 bug 是否复现取决于规划非确定性。

用法见文件末尾或 replay_moveit_guide.md。
"""

import argparse
import glob
import os
import pickle
import signal
import subprocess as sp
import sys
import time


# ----------------------------------------------------------------------------
# 预定义 case：来自 run 20260618-150444 中已用 rosbag 确认的真实 bug。
# 每个 case 是一串目标位姿 (x, y, z, ox, oy, oz, ow)，按发送顺序排列。
# 这些序列从实验 queue/ 里的 pickle 精确提取，原样回放即可重走当时的规划。
# ----------------------------------------------------------------------------
CASES = {
    # --- 真实 bug：TOPP-RA 规划速度超限（joint1 达 2.2185+ > 2.175）---
    "velocity_joint1": {
        "desc": "TOPP-RA 规划下发 joint1 速度超限 (>2.175 rad/s)。真实 bug。",
        "frame": "1781797041.2787309",
        "poses": [
            (0.5308, 0.5176, 0.9000, 0.0, 0.0, 0.0, 1.0),
            (0.1714, 0.0520, 0.9000, 0.0, 0.0, 0.0, 1.0),
            (0.0203, 0.6000, 0.3036, 0.0, 0.0, 0.0, 1.0),
            (0.4377, 0.0272, 0.2681, 0.0, 0.0, 0.0, 1.0),
            (0.6000, 0.3120, 0.5021, 0.0, 0.0, 0.0, 1.0),
            (0.6000, 0.4387, 0.9000, 0.0, 0.0, 0.0, 1.0),
            (-0.2230, -0.3008, 0.4829, 0.0, 0.0, 0.0, 1.0),
            (0.5323, 0.6000, 0.9000, 0.046, -0.322, -0.351, 0.878),
        ],
    },
    # --- 真实 bug：TOPP-RA 规划速度超限（joint3）---
    "velocity_joint3": {
        "desc": "TOPP-RA 规划下发 joint3 速度超限 (>2.175 rad/s)。真实 bug。",
        "frame": "1781798548.397772",
        "poses": [
            (-0.3919, 0.1086, 0.4126, 0.0, 0.0, 0.0, 1.0),
            (-0.4120, 0.3737, 0.4803, 0.0, 0.0, 0.0, 1.0),
            (0.1720, -0.0807, 0.6223, 0.0, 0.0, 0.0, 1.0),
            (0.6000, 0.0000, 0.5000, 0.0, 0.0, 0.0, 1.0),
            (0.7500, 0.0000, 0.5000, 0.0, 0.0, 0.0, 1.0),
            (-0.5080, 0.3852, 0.7268, 0.0, 0.0, 0.0, 1.0),
            (0.7500, 0.0000, 0.5000, 0.0, 0.0, 0.0, 1.0),
        ],
    },
    # --- 真实 bug：规划器对 joint4 下发越限位置 (-0.0423 越出上限 -0.0698) ---
    "position_joint4": {
        "desc": "规划器对 joint4 下发超出关节角上限的目标位置。真实 bug。",
        "frame": "1781841389.0141182",
        "poses": [
            (0.2968, 0.2449, 0.4064, -0.022, 0.112, 0.165, 0.980),
            (0.2207, 0.0201, 0.4907, 0.0, 0.0, 0.0, 1.0),
            (0.2755, -0.2108, 0.5855, 0.0, 0.0, 0.0, 1.0),
            (0.4444, -0.3774, 0.5840, 0.037, -0.067, 0.008, 0.997),
            (0.6000, -0.4290, 0.4879, 0.480, 0.356, -0.094, 0.796),
            (0.6000, -0.3498, 0.4054, 0.0, 0.0, 0.0, 1.0),
        ],
    },
    # --- 真实 bug：声称成功但末端偏差 9.3mm > 1mm (ISO9283)。索引对齐正确。---
    "endpoint_real": {
        "desc": "目标 SUCCESS 但末端实际偏差 9.3mm (>1mm)。真实小偏差。",
        "frame": "1781821798.6098142",
        "poses": [
            (0.6000, 0.0000, 0.5000, 0.0, 0.0, 0.0, 1.0),
            (-0.5257, -0.0002, 0.5129, 0.0, 0.0, 0.0, 1.0),
            (0.0592, 0.6000, 0.5789, 0.0, 0.0, 0.0, 1.0),
            (-0.5257, -0.0002, 0.5129, 0.0, 0.0, 0.0, 1.0),
            (0.5860, 0.0220, 0.4994, -0.201, 0.0, 0.019, 0.980),
            (-0.5257, -0.0002, 0.5129, 0.0, 0.0, 0.0, 1.0),
            (0.0227, -0.6000, 0.4722, 0.0, 0.0, 0.0, 1.0),
            (-0.4859, -0.1728, 0.7124, 0.0, 0.0, 0.0, 1.0),
        ],
    },
    # --- 对照组：单个可达目标，正常应无 bug ---
    "baseline": {
        "desc": "单个可达目标，正常规划，应无任何 error（对照组）。",
        "frame": None,
        "poses": [
            (0.4000, 0.0000, 0.5000, 0.0, 0.0, 0.0, 1.0),
            (0.3000, 0.2000, 0.6000, 0.0, 0.0, 0.0, 1.0),
        ],
    },
}


# ----------------------------------------------------------------------------
# 工具函数
# ----------------------------------------------------------------------------
def default_logdir():
    """优先用容器内挂载路径，其次本机仓库路径。"""
    for d in ("/robofuzz/src/logs", "/home/ymc/RoboFuzz/logs_moveit"):
        if os.path.isdir(d):
            return d
    return "logs"


def load_poses_from_log(logdir, frame):
    """从实验 queue/ 目录按 frame 读取该次迭代的完整目标位姿序列。"""
    pat = os.path.join(logdir, "queue", "msg-{}-*".format(frame))
    files = sorted(glob.glob(pat))
    if not files:
        raise FileNotFoundError(
            "在 {} 下找不到 frame={} 的 queue 文件".format(logdir, frame)
        )
    poses = []
    for f in files:
        d = pickle.load(open(f, "rb"))
        p = d["position"]
        o = d["orientation"]
        poses.append((p["x"], p["y"], p["z"],
                      o["x"], o["y"], o["z"], o["w"]))
    return poses


def parse_seq(seq_str):
    """解析自定义序列字符串 'x,y,z,ox,oy,oz,ow; ...'。
    省略 orientation 时默认 (0,0,0,1)。"""
    poses = []
    for chunk in seq_str.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        vals = [float(v) for v in chunk.split(",")]
        if len(vals) == 3:
            x, y, z = vals
            ox, oy, oz, ow = 0.0, 0.0, 0.0, 1.0
        elif len(vals) == 7:
            x, y, z, ox, oy, oz, ow = vals
        else:
            raise ValueError(
                "每段应为 'x,y,z' 或 'x,y,z,ox,oy,oz,ow'，得到: " + chunk
            )
        poses.append((x, y, z, ox, oy, oz, ow))
    return poses


def print_poses(poses):
    """打印位姿序列，对越界 / 极端目标加标记。"""
    print("  目标序列 ({} 个目标):".format(len(poses)))
    for i, (x, y, z, ox, oy, oz, ow) in enumerate(poses):
        # workspace 约 0.855m 球；超出 reliable reach 标记
        reach = (x * x + y * y + z * z) ** 0.5
        tag = ""
        if reach > 0.855:
            tag = " ⚠ 超工作空间"
        elif reach > 0.75:
            tag = " ⚠ 边界/可能不可达"
        ori = "" if (ox, oy, oz, ow) == (0.0, 0.0, 0.0, 1.0) else " +ori"
        print("   目标 {:>2}: x={:+.4f} y={:+.4f} z={:+.4f}  reach={:.3f}{}{}"
              .format(i, x, y, z, reach, ori, tag))


# ----------------------------------------------------------------------------
# 模式 1：离线复检 —— 在该 bug 的原始 rosbag 上重跑 oracle（确定性）
# ----------------------------------------------------------------------------
def run_offline(logdir, frame):
    """复刻 fuzzer 的 oracle 调用路径，对已录制的 rosbag 重新检测。"""
    # 延迟导入：这些依赖 ROS2 环境，offline 也需要（要反序列化消息）
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from rosbag_parser import RosbagParser
    import checker

    bag_glob = os.path.join(logdir, "rosbags", frame, "states-*.bag",
                            "*.db3")
    bags = glob.glob(bag_glob)
    if not bags:
        print("[-] 找不到 rosbag: {}".format(bag_glob))
        return 1
    db3 = bags[0]
    print("[offline] 读取 rosbag: {}".format(db3))

    parser = RosbagParser(db3)
    state_dict = parser.process_all_messages()
    print("[offline] 录到的话题: {}".format(
        {k: len(v) for k, v in state_dict.items()}))

    # 构造一个最小 config（oracle 只判 config.test_moveit 分支，不读其它字段）
    class _Cfg:
        test_moveit = True
        rospkg = "moveit2"
        tb3_sitl = tb3_hitl = px4_sitl = False
        test_rosidl = test_cli = test_rcl = False
    cfg = _Cfg()

    # oracle 的端点检查需要 reach 不到 msg_list（它用 MPR）；msg_list 给空即可
    errs = checker.run_checks(cfg, [], state_dict, [])
    print("\n[offline] ===== oracle 复检结果 =====")
    if errs:
        print("[offline] 复现 {} 条 error：".format(len(errs)))
        for e in errs:
            print("   - {}".format(e))
    else:
        print("[offline] 未复现任何 error（可能是录制截断类误报，或 oracle 已改）")
    return 0


# ----------------------------------------------------------------------------
# 模式 2：在线重放 —— 向 live MoveIt 栈重新发送目标，录制并复检
# ----------------------------------------------------------------------------
def _make_min_config(logdir, frame):
    """构造执行所需的最小 config（复用 fuzzer 的 executor / harness）。"""

    class _Cfg:
        test_moveit = True
        rospkg = "moveit2"
        rosnode = None
        exec_cmd = None
        persistent = False
        replay = True  # 不再往 queue 写 pickle
        tb3_sitl = tb3_hitl = px4_sitl = False
        px4_ros = use_mavlink = exp_pgfuzz = False
        test_rosidl = test_cli = test_rcl = False
        use_ulg = False
        watchlist = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "watchlist", "moveit2.json")
        seqlen = 10
        # 录制输出目录：单独放，避免污染实验目录
        _replay_root = os.path.join(logdir, "replay")
        rosbag_dir = os.path.join(_replay_root, "rosbags")
        queue_dir = os.path.join(_replay_root, "queue")
        error_dir = os.path.join(_replay_root, "errors")

    cfg = _Cfg()
    for d in (cfg.rosbag_dir, cfg.queue_dir, cfg.error_dir):
        os.makedirs(d, exist_ok=True)
    return cfg


def run_live(logdir, poses, rate, keep_running):
    """重新发送目标位姿到 live MoveIt，录制状态，再跑 oracle。

    依赖：终端里已经 `ros2 launch moveit2_tutorials move_group.launch.py`
    把 move_group 栈拉起来（与 fuzzer 的 run_moveit_harness 一致）。
    """
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import harness
    from geometry_msgs.msg import Pose
    from rosbag_parser import RosbagParser
    import checker

    cfg = _make_min_config(logdir, None)

    frame = "replay-{}".format(int(time.time()))
    bag_dir = os.path.join(cfg.rosbag_dir, frame)
    os.makedirs(bag_dir, exist_ok=True)

    # 录制 4 个 watched 话题（与实验一致）
    with open(cfg.watchlist) as f:
        import json
        topics = list(json.load(f).keys())
    rec_path = os.path.join(bag_dir, "states-0.bag")
    rec_cmd = ["ros2", "bag", "record", "-o", rec_path,
               "--include-hidden-topics"] + topics
    print("[live] 开始录制: {}".format(" ".join(rec_cmd)))
    rec = sp.Popen(rec_cmd, stdout=sp.DEVNULL, stderr=sp.DEVNULL,
                   start_new_session=True)
    time.sleep(2.0)  # 等录制就绪

    period = 1.0 / rate if rate > 0 else 0.0
    try:
        for i, (x, y, z, ox, oy, oz, ow) in enumerate(poses):
            msg = Pose()
            msg.position.x = float(x)
            msg.position.y = float(y)
            msg.position.z = float(z)
            msg.orientation.x = float(ox)
            msg.orientation.y = float(oy)
            msg.orientation.z = float(oz)
            msg.orientation.w = float(ow)
            print("[live] 发送目标 {}/{}: "
                  "({:+.3f},{:+.3f},{:+.3f})".format(i + 1, len(poses),
                                                     x, y, z))
            harness.moveit_send_command(msg)  # 阻塞至该目标完成/超时
            if period:
                time.sleep(period)
    finally:
        # 停止录制
        try:
            os.killpg(os.getpgid(rec.pid), signal.SIGINT)
            rec.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(rec.pid), signal.SIGKILL)
            except Exception:
                pass

    # 跑 oracle
    bags = glob.glob(os.path.join(bag_dir, "states-0.bag", "*.db3"))
    if not bags:
        print("[-] 录制失败，无 db3 文件")
        return 1
    parser = RosbagParser(bags[0])
    state_dict = parser.process_all_messages()
    print("[live] 录到的话题: {}".format(
        {k: len(v) for k, v in state_dict.items()}))

    errs = checker.run_checks(cfg, [], state_dict, [])
    print("\n[live] ===== oracle 检测结果 =====")
    if errs:
        print("[live] 检出 {} 条 error：".format(len(errs)))
        for e in errs:
            print("   - {}".format(e))
    else:
        print("[live] 未检出 error（规划非确定性，未必每次复现）")

    if not keep_running:
        # moveit_send_command 已对每个目标自行清理其 launch 进程组；
        # 这里再扫一遍残留的 move_group_interface_tutorial 防止泄漏。
        sp.run(["pkill", "-9", "-f", "move_group_interface_tutorial"],
               stdout=sp.DEVNULL, stderr=sp.DEVNULL)
    print("[live] rosbag 保存在: {}".format(bag_dir))
    return 0


# ----------------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="MoveIt2 Panda bug 回放工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--list", action="store_true", help="列出预定义 case")
    ap.add_argument("--case", help="回放预定义 case 名")
    ap.add_argument("--seq", help="自定义序列 'x,y,z[,ox,oy,oz,ow]; ...'")
    ap.add_argument("--from-log", dest="from_log",
                    help="实验日志目录（默认自动探测）")
    ap.add_argument("--frame", help="--from-log 时指定 bug 的 frame 时间戳")
    ap.add_argument("--mode", choices=["offline", "live"], default="offline",
                    help="offline=在原 rosbag 上复检(默认,确定性); "
                         "live=向 live MoveIt 重发目标并观察")
    ap.add_argument("--rate", type=float, default=0.0,
                    help="live 模式发送间隔频率(Hz)，0=逐目标阻塞(默认)")
    ap.add_argument("--keep-running", action="store_true",
                    help="live 模式结束后不清理 move_group 栈")
    args = ap.parse_args()

    if args.list:
        print("可用预定义 case：\n")
        for name, c in CASES.items():
            fr = c["frame"] or "(无, 对照组)"
            print("  {:<18} {}".format(name, c["desc"]))
            print("  {:<18} frame={}  goals={}\n"
                  .format("", fr, len(c["poses"])))
        print("用法示例：")
        print("  python3 replay_moveit_bug.py --case velocity_joint1")
        print("  python3 replay_moveit_bug.py --case velocity_joint1 "
              "--mode live")
        print("  python3 replay_moveit_bug.py --from-log "
              "../logs_moveit/20260618-150444 --frame 1781797041.2787309")
        return 0

    logdir = args.from_log or default_logdir()

    # 决定回放哪个序列 + 是否有原始 rosbag 可离线复检
    poses = None
    frame = None
    if args.case:
        if args.case not in CASES:
            print("[-] 未知 case: {}（用 --list 查看）".format(args.case))
            return 1
        poses = CASES[args.case]["poses"]
        frame = CASES[args.case]["frame"]
    elif args.seq:
        poses = parse_seq(args.seq)
    elif args.frame:
        frame = args.frame
        poses = load_poses_from_log(logdir, frame)
    else:
        print("[-] 需指定 --case / --seq / --frame 之一（或 --list）")
        return 1

    print("=" * 64)
    print("回放模式: {}".format(args.mode))
    if frame:
        print("bug frame: {}".format(frame))
    print_poses(poses)
    print("=" * 64)

    if args.mode == "offline":
        if not frame:
            print("[-] offline 模式需要原始 rosbag，请用 --case(真实bug) "
                  "或 --frame 指定一个实验中存在的 frame；")
            print("    自定义 --seq 没有历史 rosbag，请改用 --mode live")
            return 1
        return run_offline(logdir, frame)
    else:
        return run_live(logdir, poses, args.rate, args.keep_running)


if __name__ == "__main__":
    sys.exit(main())
