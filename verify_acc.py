#!/usr/bin/env python3
"""Independent verification: read raw desired.accelerations straight from the
rosbag (NOT via the oracle) and recompute ratios against Panda joint limits.

This proves the acc-limit violations are genuine planner output, not an oracle
calculation artifact.
"""
import sys
import sqlite3

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

PANDA_ACC = {
    "panda_joint1": 3.75, "panda_joint2": 1.875, "panda_joint3": 2.5,
    "panda_joint4": 3.125, "panda_joint5": 3.75, "panda_joint6": 5.0,
    "panda_joint7": 5.0,
}

bag = sys.argv[1]
con = sqlite3.connect(bag)
cur = con.cursor()

# topic table
topics = {}
for tid, name, typ in cur.execute("SELECT id,name,type FROM topics"):
    topics[tid] = (name, typ)

# find controller_state topic
cs_tid = None
cs_type = None
for tid, (name, typ) in topics.items():
    if "controller_state" in name or "controller/state" in name:
        cs_tid = tid
        cs_type = typ
        print(f"controller_state topic: {name}  type={typ}")
        break

if cs_tid is None:
    print("topics in bag:")
    for tid, (name, typ) in topics.items():
        print(f"  {name}  ({typ})")
    sys.exit("no controller_state topic found")

MsgType = get_message(cs_type)

worst = {}          # joint -> (max_abs_acc, ratio)
total_samples = 0
violation_samples = 0
max_overall = (None, 0.0, 0.0)  # joint, acc, ratio

rows = cur.execute(
    "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (cs_tid,)
).fetchall()

for (blob,) in rows:
    msg = deserialize_message(bytes(blob), MsgType)
    names = list(getattr(msg, "joint_names", []))
    desired = getattr(msg, "desired", None) or getattr(msg, "reference", None)
    if desired is None:
        continue
    accs = list(getattr(desired, "accelerations", []) or [])
    if not accs:
        continue
    total_samples += 1
    for i, nm in enumerate(names):
        if nm not in PANDA_ACC or i >= len(accs):
            continue
        a = abs(accs[i])
        lim = PANDA_ACC[nm]
        ratio = a / lim
        if nm not in worst or a > worst[nm][0]:
            worst[nm] = (a, ratio)
        if ratio > 1.02:
            violation_samples += 1
        if ratio > max_overall[2]:
            max_overall = (nm, a, ratio)

print(f"\n=== RAW rosbag verification: {bag.split('/')[-3]} ===")
print(f"controller_state samples with desired.accelerations: {total_samples}")
print(f"samples exceeding 1.02x limit: {violation_samples}")
print(f"\nper-joint worst |desired acceleration| (from RAW bag):")
for nm in sorted(worst):
    a, r = worst[nm]
    flag = "  <-- VIOLATION" if r > 1.02 else ""
    print(f"  {nm:14s} {a:8.4f} rad/s^2  (limit {PANDA_ACC[nm]:.3f}, ratio {r:.3f}){flag}")
print(f"\nWORST overall: {max_overall[0]} = {max_overall[1]:.4f} rad/s^2 "
      f"(ratio {max_overall[2]:.3f})")
