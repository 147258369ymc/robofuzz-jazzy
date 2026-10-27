#!/usr/bin/env python3
"""Nav2 1000-round FALSE_FAILURE 真伪判定 — reads each violating round's ARCHIVED
bag (rosbags/<frame>/states-0.bag) for the ACTUAL sent goal (post-snap, from
/robofuzz/nav2_goal), the ground-truth trajectory, and the goal_event, then
judges真伪 with the inflation-clearance rule.

Runs inside the container (needs rosbag2_py + rclpy deserialization).
"""
import os, glob, math, sys
sys.path.insert(0, "/work/src_jazzy")
import nav2_map

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, String
from nav_msgs.msg import Odometry

RUN = os.environ.get("NAV2_RUN",
    "/work/logs_jazzy/nav2_long1000_20260813_193229/20260813-113232")
INFLATION, ROBOT_R = 0.45, 0.22
CLEARANCE_MIN = INFLATION + ROBOT_R  # 0.67 m

def read_bag(bag_path):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag_path, storage_id="sqlite3"),
           ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    goal = None; result = None; event = None; truth = []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == "/robofuzz/nav2_goal":
            m = deserialize_message(data, PoseStamped)
            goal = (m.pose.position.x, m.pose.position.y)
        elif topic == "/robofuzz/nav2_result":
            result = deserialize_message(data, Int32).data
        elif topic == "/robofuzz/nav2_goal_event":
            event = deserialize_message(data, String).data
        elif topic == "/sim_ground_truth_pose":
            m = deserialize_message(data, Odometry)
            truth.append((m.pose.pose.position.x, m.pose.pose.position.y))
    return goal, result, event, truth

def clearance(m, gx, gy, max_r=1.0):
    gc, gr = m.world_to_cell(gx, gy)
    if not m._in_bounds(gc, gr):
        return max_r
    steps = int(max_r / m.resolution); best = max_r
    for dr in range(-steps, steps+1):
        for dc in range(-steps, steps+1):
            c, r = gc+dc, gr+dr
            if m._in_bounds(c, r) and not m.is_free(c, r):
                best = min(best, math.hypot(dc, dr)*m.resolution)
    return best

def main():
    m = nav2_map.load_occupancy_map("/work/maps/slam_arena.yaml")
    bags = sorted(glob.glob(RUN + "/rosbags/*/states-0.bag"))
    print("=== %d 违规轮 bag ===\n" % len(bags))
    hdr = "%-16s %7s %7s %6s %6s %7s %8s  %s" % (
        "frame","goal_x","goal_y","net_d","clear","result","snapped","verdict")
    print(hdr); print("-"*95)
    real=fp=other=0
    for b in bags:
        frame = b.split("/rosbags/")[1].split("/")[0]
        try:
            goal, result, event, truth = read_bag(b)
        except Exception as e:
            print("%-16s bag读取失败: %s" % (frame[:16], e)); continue
        if goal is None or not truth:
            print("%-16s 缺 goal/truth" % frame[:16]); continue
        gx, gy = goal
        net_d = math.hypot(truth[-1][0]-truth[0][0], truth[-1][1]-truth[0][1])
        clr = clearance(m, gx, gy)
        snapped = "yes" if (event and "snapped" in event) else "no"
        # 真伪: ABORTED(6) + 开阔区(clr>=0.67) + 真实运动 => 真bug; 膨胀区 => 误报
        if result == 6 and clr >= CLEARANCE_MIN and net_d >= 0.10:
            verdict = "REAL-BUG"; real += 1
        elif result == 6 and clr < CLEARANCE_MIN:
            verdict = "FALSE-POS(in-inflation clr<%.2f)" % CLEARANCE_MIN; fp += 1
        else:
            verdict = "OTHER(result=%s)" % result; other += 1
        print("%-16s %7.2f %7.2f %6.2f %6.2f %7s %8s  %s" % (
            frame[:16], gx, gy, net_d, clr, result, snapped, verdict))
    print("\n=== 判定: %d 真bug / %d 误报(膨胀区) / %d 其他 ===" % (real, fp, other))
    print("膨胀阈值 = inflation(0.45)+robot(0.22) = %.2fm" % CLEARANCE_MIN)

if __name__ == "__main__":
    main()
