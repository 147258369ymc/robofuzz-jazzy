"""Verify the aborted-last-MPR false-positive hypothesis for endpoint
deviation errors in run 20260618-150444.

For each deviation rosbag, recompute the oracle's endpoint check and ALSO
compute the deviation against the goal that the arm actually rested at
(the best-matching MPR), to prove whether the reported deviation is a
false positive caused by comparing against an aborted/earlier MPR goal.
"""
import os
import sys
import glob
import math

import kinpy
from rosbag_parser import RosbagParser

RUN = "/robofuzz/src/logs/20260618-150444"
URDF = os.environ.get("PANDA_URDF", "")

DEV_FILES = [
    "1781841903.0983608", "1781821798.6098142", "1781907145.5874925",
    "1781809164.6583545", "1781899299.888733", "1781899347.6912837",
    "1781919436.5579739", "1781888870.508211", "1781810117.372118",
    "1781886129.010948", "1781886074.9529471", "1781808178.072124",
    "1781885978.905871", "1781808130.658542", "1781815134.3129492",
    "1781810960.9687629", "1781808276.7060928", "1781808227.988831",
    "1781824742.4836748", "1781824687.5246549", "1781824800.8453097",
    "1781866357.0346437", "1781853396.1479654", "1781858422.8862042",
]


def find_urdf():
    if URDF and os.path.exists(URDF):
        return URDF
    cands = glob.glob("/robofuzz/**/panda*.urdf", recursive=True)
    cands += glob.glob("/robofuzz/**/panda.urdf", recursive=True)
    cands += glob.glob("/opt/ros/**/panda.urdf", recursive=True)
    return cands[0] if cands else None


def fk_ee(chain, arm_positions):
    jam = dict(arm_positions)
    jam.setdefault("panda_finger_joint1", 0.0)
    jam.setdefault("panda_finger_joint2", 0.0)
    fk = chain.forward_kinematics(jam)
    return fk["panda_hand"].pos


def main():
    urdf = find_urdf()
    if not urdf:
        print("URDF not found; set PANDA_URDF")
        sys.exit(1)
    with open(urdf) as f:
        chain = kinpy.build_chain_from_urdf(f.read())

    print(f"{'frame':>20} {'nMPR':>4} {'nStat':>5} {'oracle_dev':>10} "
          f"{'best_dev':>9} {'best_mpr_idx':>12} verdict   statuses")
    for frame in DEV_FILES:
        bags = glob.glob(f"{RUN}/rosbags/{frame}/states-*.bag/*.db3")
        if not bags:
            print(f"{frame:>20}  NO BAG")
            continue
        parser = RosbagParser(bags[0])
        msgs = parser.process_all_messages()

        status_list = msgs.get("/move_action/_action/status", [])
        mprs = msgs.get("/motion_plan_request", [])
        cont = msgs.get("/panda_arm_controller/state", [])
        if not status_list or not mprs or not cont:
            print(f"{frame:>20}  MISSING TOPICS "
                  f"stat={len(status_list)} mpr={len(mprs)} cont={len(cont)}")
            continue

        # last action status terminal + statuses
        _, last_status_msg = status_list[-1]
        statuses = [gs.status for gs in last_status_msg.status_list]
        nStat = len(statuses)
        last_terminal = statuses[-1] if statuses else None

        # all MPR goals
        mpr_goals = []
        for _, mpr in mprs:
            try:
                gp = mpr.goal_constraints[0].position_constraints[0] \
                    .constraint_region.primitive_poses[0].position
                mpr_goals.append((gp.x, gp.y, gp.z))
            except (IndexError, AttributeError):
                mpr_goals.append(None)
        nMPR = len(mpr_goals)
        last_mpr_goal = mpr_goals[-1] if mpr_goals else None

        # controller actual pose at/just before last status ts
        last_status_ts = status_list[-1][0]
        chosen = None
        for (ts, cs) in cont:
            if ts <= last_status_ts:
                chosen = cs
            else:
                break
        if chosen is None:
            chosen = cont[-1][1]
        arm_pos = {
            name: chosen.actual.positions[i]
            for i, name in enumerate(chosen.joint_names)
            if i < len(chosen.actual.positions)
        }

        ee = fk_ee(chain, arm_pos)

        def dist(g):
            return math.sqrt((g[0]-ee[0])**2 + (g[1]-ee[1])**2
                             + (g[2]-ee[2])**2)

        oracle_dev = dist(last_mpr_goal) if last_mpr_goal else None

        # best-matching MPR goal (what the arm actually rested at)
        best_dev = None
        best_idx = None
        for i, g in enumerate(mpr_goals):
            if g is None:
                continue
            d = dist(g)
            if best_dev is None or d < best_dev:
                best_dev = d
                best_idx = i

        # verdict: FP if oracle compared against non-best MPR AND a different
        # MPR is a much better match (arm actually reached a different goal)
        verdict = "REAL"
        if best_dev is not None and oracle_dev is not None:
            if best_idx != (nMPR - 1) and best_dev < 0.05 < oracle_dev:
                verdict = "FALSE_POS"
            elif nStat != nMPR and best_dev < 0.05:
                verdict = "FALSE_POS"

        od = f"{oracle_dev:.4f}" if oracle_dev is not None else "None"
        bd = f"{best_dev:.4f}" if best_dev is not None else "None"
        print(f"{frame:>20} {nMPR:>4} {nStat:>5} {od:>10} {bd:>9} "
              f"{str(best_idx):>12} {verdict:<9} {statuses}")


if __name__ == "__main__":
    main()
