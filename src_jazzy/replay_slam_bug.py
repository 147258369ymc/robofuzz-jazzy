#!/usr/bin/env python3
"""
slam_toolbox bug 回放工具 - 脱离 RoboFuzz,独立重现 SLAM 漂移 bug 并配合 RViz 录屏。

与 px4/tb3 的回放不同:SLAM bug 是"定位漂移"——机器人物理上动得正常,是它
估计的位姿偏离了真值。所以本脚本在发布 /cmd_vel 驱动机器人的同时,订阅
slam_toolbox 的估计位姿 (/pose) 和 Gazebo 真值 (/sim_ground_truth_pose),
实时打印两者的偏离,并在结束时给出该轮的漂移汇总。配合 RViz 里估计轨迹与
真值轨迹的"分叉"可视化,即可录制出有说服力的复现视频。

三个具名 case 的输入序列取自 1000 轮长跑 (slam_long1000_v6) 的实测触发轮次,
数值与 docs/issues/slam_toolbox_forms/ 的 issue 文档、
docs/reports/2026-08-03-slam-toolbox-v6-1000round-bug-analysis.md 完全一致。

前置:先在带 GUI 的容器里起好 SLAM 栈(Gazebo + slam_toolbox + RViz),
例如按 README 的 "slam_toolbox mapping with Gazebo and RViz" 分节启动
run_target.sh slam_toolbox_jazzy(TURTLEBOT4_HEADLESS=0, SLAM_RVIZ=1)。
等机器人、/scan、/pose、RViz 就绪后,在同一容器另开终端运行本脚本。

用法:
    source /opt/ros/jazzy/setup.bash
    python3 /work/src_jazzy/replay_slam_bug.py --case B_translation

    # 列出所有 case
    python3 replay_slam_bug.py --list

    # 自定义序列 (lin,ang;lin,ang;...)
    python3 replay_slam_bug.py --seq "-0.35,0;-0.35,0;-0.35,0"

选项:
    --case NAME     预定义 case (见 --list)
    --interval N    指令间隔秒 (默认 1.0,与 fuzzer campaign 一致)
    --settle N      序列结束后继续观测的秒数 (默认 5.0)
    --seq STR       自定义序列
    --list          列出 case
"""

import argparse
import math
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSProfile,
        QoSReliabilityPolicy,
        QoSHistoryPolicy,
    )
    from geometry_msgs.msg import TwistStamped
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
except ImportError:
    print("ERROR: ROS 2 not sourced. Run: source /opt/ros/jazzy/setup.bash")
    sys.exit(1)


ESTIMATE_TOPIC = "/pose"                       # PoseWithCovarianceStamped
GROUND_TRUTH_TOPIC = "/sim_ground_truth_pose"  # nav_msgs/Odometry
CMD_TOPIC = "/cmd_vel"                          # geometry_msgs/TwistStamped


def _run(value, count):
    return [value] * count


# ============================================================
# 预定义 case —— 序列取自 slam_long1000_v6 实测触发轮次
# ============================================================
CASES = {
    "A_rotation": {
        "desc": "旋转致航向漂移 (YAW 0.04998 rad ~2.9deg, +138% over clean baseline). "
                "弧线急转: (+0.2975,+0.6)x20 -> (-0.2975,-0.6)x20 [round 1785221178]",
        "watch": "yaw",
        "seq": _run((0.2975, 0.6), 20) + _run((-0.2975, -0.6), 20),
    },
    "B_translation": {
        "desc": "纯直线平移致相对位姿漂移 (RPE 0.04283 m, +150% over clean baseline). "
                "纯平移无旋转: (-0.35,0)x40 [round 1785332386]",
        "watch": "rpe",
        "seq": _run((-0.35, 0.0), 40),
    },
    "C_arc": {
        "desc": "中等弧线致边界漂移 (RPE 0.02315 m, +0.6% over threshold, 弱信号). "
                "多段变curvature弧线 [round 1785318265]",
        "watch": "rpe",
        "seq": (_run((-0.3189, 1.1386), 8) + _run((-0.2229, -0.8584), 8)
                + _run((0.2018, -1.0739), 8) + _run((-0.2302, -1.1674), 8)
                + _run((0.3142, 0.8469), 8)),
    },
    "baseline": {
        "desc": "温和前进 (无漂移基准,用于对照录屏: 两条轨迹应当基本重合)",
        "watch": "both",
        "seq": _run((0.15, 0.0), 40),
    },
}


def _yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class SlamBugReplayer(Node):
    def __init__(self):
        super().__init__("slam_bug_replayer")
        self.pub = self.create_publisher(TwistStamped, CMD_TOPIC, 10)
        # /pose 通常 reliable;真值 (gz odom 插件) 是 best_effort。
        self.create_subscription(
            PoseWithCovarianceStamped, ESTIMATE_TOPIC, self._on_est, 10
        )
        best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            Odometry, GROUND_TRUTH_TOPIC, self._on_truth, best_effort
        )
        self.est = []    # (t, x, y, yaw)
        self.truth = []  # (t, x, y, yaw)
        time.sleep(0.5)

    def _on_est(self, msg):
        p = msg.pose.pose
        self.est.append(
            (time.time(), p.position.x, p.position.y,
             _yaw_from_quat(p.orientation))
        )

    def _on_truth(self, msg):
        p = msg.pose.pose
        self.truth.append(
            (time.time(), p.position.x, p.position.y,
             _yaw_from_quat(p.orientation))
        )

    def _make_twist(self, lin_x, ang_z):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = float(lin_x)
        m.twist.angular.z = float(ang_z)
        return m

    def send_stop(self):
        self.pub.publish(self._make_twist(0.0, 0.0))

    def replay(self, seq, interval, settle):
        print(f"\n  发布 {len(seq)} 条指令 (interval={interval}s)...")
        for i, (lin_x, ang_z) in enumerate(seq):
            self.pub.publish(self._make_twist(lin_x, ang_z))
            n_est = len(self.est)
            print(f"  [{i+1:>2}/{len(seq)}] cmd lin={lin_x:+.4f} "
                  f"ang={ang_z:+.4f}  |  /pose 累计={n_est}", end="")
            self._print_live_drift()
            end = time.time() + interval
            while time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
        print("  [STOP] 发送零速度")
        self.send_stop()
        print(f"  观测 {settle}s...")
        end = time.time() + settle
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _print_live_drift(self):
        """打印当前估计与真值航向的瞬时偏差(粗略,实时观感用)。"""
        if not self.est or not self.truth:
            print("  (等待 /pose 或真值...)")
            return
        _, ex, ey, eyaw = self.est[-1]
        _, tx, ty, tyaw = self.truth[-1]
        dyaw = abs((eyaw - tyaw + math.pi) % (2 * math.pi) - math.pi)
        print(f"  瞬时|Δyaw|={dyaw:.4f}rad")

    def summarize(self, watch="both"):
        """结束时用 oracle 自身的口径计算 RPE/ATE/YAW,与 issue 数值同源。

        watch: 'rpe' / 'yaw' / 'both' —— 决定重点高亮哪个指标(全部都算,
        只是按 case 的失效指标标注)。复用 oracles/slam.py 的函数,不自己重算。
        """
        print(f"\n{'='*60}")
        print(f"  漂移汇总:  /pose 样本={len(self.est)}  真值样本={len(self.truth)}")

        try:
            import oracles.slam as S
        except ImportError:
            print("  (无法 import oracles.slam;确认 PYTHONPATH 含 src_jazzy)")
            print(f"{'='*60}")
            return

        # self.est / self.truth 已是 (t, x, y, yaw),正是 oracle 内部轨迹形状,
        # 可直接喂 _match_nearest。
        est_track = sorted(self.est, key=lambda r: r[0])
        truth_track = sorted(self.truth, key=lambda r: r[0])
        pairs, matched_times = S._match_nearest(est_track, truth_track)
        print(f"  匹配对数={len(pairs)}  (oracle 门槛 SLAM_MIN_SAMPLES={S.SLAM_MIN_SAMPLES})")
        if len(pairs) < S.SLAM_MIN_SAMPLES:
            print("  匹配样本不足,oracle 口径下不足以判定。")
            print("  录屏建议:加长 --interval,或改用 --play-bag 回放原始 bug rosbag。")
            print(f"{'='*60}")
            return

        window = S._window_for_times(matched_times, len(pairs))
        rpe = S._max_rpe(pairs, window)
        ate = S._ate_rmse(pairs)
        yaw = S._yaw_drift_spread(pairs)

        def mark(val, lim):
            return f"{val:.5f}  (阈值 {lim}) -> {'超限 FIRES' if val > lim else 'ok'}"

        hi_rpe = "  >>> " if watch in ("rpe", "both") else "      "
        hi_yaw = "  >>> " if watch in ("yaw", "both") else "      "
        print(f"{hi_rpe}RPE (window={window}): {mark(rpe, S.SLAM_RPE_DRIFT_ERROR)}")
        print(f"      ATE:               {mark(ate, S.SLAM_ATE_RMSE_ERROR)}")
        print(f"{hi_yaw}YAW drift:         {mark(yaw, S.SLAM_YAW_DRIFT_ERROR)} "
              f"({math.degrees(yaw):.2f} deg)")
        print(f"  口径与 oracles/slam.py 一致 —— 与 issue/report 数值同源。")
        print(f"{'='*60}")


def parse_seq(s):
    seq = []
    for pair in s.strip().split(";"):
        parts = pair.strip().split(",")
        if len(parts) != 2:
            print(f"ERROR: bad pair '{pair}', expected 'lin,ang'")
            sys.exit(1)
        seq.append((float(parts[0]), float(parts[1])))
    return seq


def _resolve_bag_dir(path):
    """Accept either the bag directory (containing metadata.yaml) or a parent;
    return the directory `ros2 bag play` should be pointed at."""
    import os
    if os.path.isfile(os.path.join(path, "metadata.yaml")):
        return path
    # allow pointing at the round-timestamp dir that holds states-0.bag/
    inner = os.path.join(path, "states-0.bag")
    if os.path.isfile(os.path.join(inner, "metadata.yaml")):
        return inner
    return path


def play_bag_mode(bag_path, watch):
    """Replay an original bug rosbag into RViz and summarize with oracle math.

    This is the strongest evidence: it replays the actually-recorded /pose vs
    /sim_ground_truth_pose from the campaign, so the drift shown is real data,
    not a re-triggered approximation. We remap the recorded /cmd_vel to a dummy
    topic so playback does not fight a live controller (the sim need not even be
    running — RViz can visualize the recorded /tf, /scan, /map, /pose, truth).
    """
    import glob
    import os
    import subprocess

    bag_dir = _resolve_bag_dir(bag_path)
    if not os.path.isfile(os.path.join(bag_dir, "metadata.yaml")):
        print(f"ERROR: no metadata.yaml under {bag_path!r} (need a ros2 bag dir)")
        sys.exit(1)

    print(f"\n{'='*60}")
    print(f"  slam_toolbox bug 回放 (原始 rosbag 实测数据)")
    print(f"  bag: {bag_dir}")
    print(f"{'='*60}")
    print("  在 RViz 里对比重播的 /pose(估计) 与 /sim_ground_truth_pose(真值);")
    print("  两条轨迹的分叉就是当时真实发生的 SLAM 漂移。")
    print("  正在 ros2 bag play(remap /cmd_vel 到 dummy,避免干扰实时控制器)...")

    # 回放供 RViz 可视化。数字汇总不依赖这个实时流(实时订阅回放会因时序/丢包
    # 导致 window 退化、数值失真);汇总改为回放后直接离线读 bag 文件,口径与
    # oracle 检测时完全一致(见 _summarize_from_bag)。
    play = subprocess.run(
        ["ros2", "bag", "play", bag_dir,
         "--remap", "/cmd_vel:=/replay_cmd_vel_ignored"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    # 权威汇总:直接从 bag 的 .db3 读整段 /pose + 真值,用 oracle 口径计算。
    dbs = glob.glob(os.path.join(bag_dir, "*.db3"))
    if not dbs:
        print("  (未找到 .db3,无法离线汇总)")
    else:
        _summarize_from_bag(dbs[0], watch)
    print("\nDone.")


def _summarize_from_bag(db_path, watch):
    """离线读 bag 的 /pose + /sim_ground_truth_pose,用 oracles/slam.py 的口径
    计算 RPE/ATE/YAW —— 与 campaign 检测、离线重算完全同源(不受回放时序影响)。"""
    import sqlite3
    try:
        import oracles.slam as S
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        print(f"  (无法 import 依赖: {exc};确认已 source ROS 且 PYTHONPATH 含 src_jazzy)")
        return

    conn = sqlite3.connect(db_path)
    topics = {r[1]: (r[0], r[2])
              for r in conn.execute("select id,name,type from topics")}
    loaded = {}
    for name in (ESTIMATE_TOPIC, GROUND_TRUTH_TOPIC):
        if name not in topics:
            loaded[name] = []
            continue
        tid, typ = topics[name]
        msg_cls = get_message(typ)
        loaded[name] = [
            (ts, deserialize_message(bytes(data), msg_cls))
            for ts, data in conn.execute(
                "select timestamp,data from messages where topic_id=? "
                "order by timestamp", (tid,))
        ]
    conn.close()

    est = S._pose_track(loaded.get(ESTIMATE_TOPIC, []))
    tru = S._pose_track(loaded.get(GROUND_TRUTH_TOPIC, []))
    print(f"\n{'='*60}")
    print(f"  漂移汇总 (离线读 bag,oracle 口径):  "
          f"/pose={len(est)}  真值={len(tru)}")
    if not est or not tru:
        print("  bag 内 /pose 或真值为空,无法汇总。")
        print(f"{'='*60}")
        return
    pairs, matched_times = S._match_nearest(est, tru)
    print(f"  匹配对数={len(pairs)}  (门槛 SLAM_MIN_SAMPLES={S.SLAM_MIN_SAMPLES})")
    if len(pairs) < S.SLAM_MIN_SAMPLES:
        print("  匹配样本不足,不足以判定。")
        print(f"{'='*60}")
        return

    window = S._window_for_times(matched_times, len(pairs))
    rpe_metrics = S._rpe_metrics(pairs, window)
    rpe = rpe_metrics["translation_max"]
    ate = S._ate_rmse(pairs)
    # 与 check() 一致: yaw 取 spread 与 rpe 旋转分量的较大者
    yaw = max(S._yaw_drift_spread(pairs), rpe_metrics["rotation_max"])

    def mark(val, lim):
        return f"{val:.5f}  (阈值 {lim}) -> {'超限 FIRES' if val > lim else 'ok'}"

    hi_rpe = "  >>> " if watch in ("rpe", "both") else "      "
    hi_yaw = "  >>> " if watch in ("yaw", "both") else "      "
    print(f"{hi_rpe}RPE (window={window}): {mark(rpe, S.SLAM_RPE_DRIFT_ERROR)}")
    print(f"      ATE:               {mark(ate, S.SLAM_ATE_RMSE_ERROR)}")
    print(f"{hi_yaw}YAW drift:         {mark(yaw, S.SLAM_YAW_DRIFT_ERROR)} "
          f"({math.degrees(yaw):.2f} deg)")
    print(f"  口径与 oracles/slam.py 的 check() 一致 —— 与 issue/report 数值同源。")
    print(f"{'='*60}")


def main():
    ap = argparse.ArgumentParser(description="Replay a slam_toolbox drift bug")
    ap.add_argument("--case", default="B_translation",
                    help=f"case: {', '.join(CASES)}")
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--settle", type=float, default=5.0)
    ap.add_argument("--seq", default=None, help="custom 'lin,ang;lin,ang;...'")
    ap.add_argument("--play-bag", default=None,
                    help="replay an original bug rosbag dir instead of "
                         "re-issuing commands (strongest evidence)")
    ap.add_argument("--watch", default=None, choices=["rpe", "yaw", "both"],
                    help="metric to highlight in summary (default: from case)")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    if args.list:
        print("\n可用 case:\n")
        for name, info in CASES.items():
            print(f"  {name:15s} [{info['watch']}] {info['desc']}\n")
        print("  --play-bag <dir>  回放原始 bug rosbag(实测数据,最强证据)\n")
        return

    # --play-bag: replay recorded data (does not need a case/seq)
    if args.play_bag:
        play_bag_mode(args.play_bag, args.watch or "both")
        return

    if args.seq:
        seq, desc, watch = parse_seq(args.seq), f"custom: {args.seq}", "both"
    elif args.case in CASES:
        seq = CASES[args.case]["seq"]
        desc = CASES[args.case]["desc"]
        watch = CASES[args.case]["watch"]
    else:
        print(f"ERROR: unknown case '{args.case}'. Use --list.")
        sys.exit(1)
    if args.watch:
        watch = args.watch

    rclpy.init()
    node = SlamBugReplayer()
    print(f"\n{'='*60}")
    print(f"  slam_toolbox bug 回放")
    print(f"  case: {args.case if not args.seq else 'custom'}  (watch={watch})")
    print(f"  {desc}")
    print(f"{'='*60}")
    print("  提示: 在 RViz 里对比 /pose(估计) 与 /sim_ground_truth_pose(真值),")
    print("        观察两条轨迹是否分叉 —— 分叉即 SLAM 漂移。")
    try:
        node.replay(seq, args.interval, args.settle)
        node.summarize(watch=watch)
    except KeyboardInterrupt:
        print("\n中断,发送停止...")
        node.send_stop()
    finally:
        node.send_stop()
        node.destroy_node()
        rclpy.shutdown()
    print("\nDone.")


if __name__ == "__main__":
    main()
