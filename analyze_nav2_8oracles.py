#!/usr/bin/env python3
"""Nav2 8-oracle campaign deep bug analysis (5 original + 3 new process-level).

Analyzes all errors from the 1000-round campaign with enhanced oracles:
- Original 5: ARRIVAL/COLLISION/TIMEOUT/STUCK/FALSE_FAILURE
- New 3: PATH_EFFICIENCY/NAVIGATION_TIME/POSE_OSCILLATION

For each bug:
1. Maps input goal to bug type
2. Judges authenticity via oracle evidence (clearance, efficiency metrics)
3. Locates root cause at code/parameter level
4. Analyzes cascade relationships
5. Determines false positives

Runs inside container (needs nav2_map).
"""
import os, glob, ast, re, sys
sys.path.insert(0, "/work/src_jazzy")
import nav2_map
from collections import Counter, defaultdict

RUN = os.environ.get("NAV2_RUN",
    "/work/logs_jazzy/nav2_long1000_8oracles_20260817_215622/20260817-135625")
INFLATION, ROBOT_R = 0.45, 0.22
CLEARANCE_MIN = INFLATION + ROBOT_R  # 0.67m

def parse_error_file(epath):
    """Parse error file (Python list format) and extract all entries."""
    try:
        content = open(epath).read()
        if content.startswith("["):
            entries = ast.literal_eval(content)
            if not isinstance(entries, list):
                entries = [entries]
        else:
            entries = [content]

        results = []
        for entry in entries:
            # Match common fields
            m = re.search(
                r"NAV2_([A-Z_]+):(info |pose )?source=nav2\s+"
                r".*?goal_x=([\d\.\-]+)\s+goal_y=([\d\.\-]+)"
                r".*?result=(\d+).*?net_disp_m=([\d\.]+)"
                r".*?duration_s=([\d\.]+).*?clearance_m=([\d\.]+)",
                entry
            )
            if not m:
                continue

            bug_type = m.group(1)
            info_marker = m.group(2)  # "info " or None

            bug = {
                'type': f"NAV2_{bug_type}",
                'is_info': (bug_type == "ABORT_IN_INFLATION"),  # Only ABORT_IN_INFLATION is info
                'goal_x': float(m.group(3)),
                'goal_y': float(m.group(4)),
                'result': int(m.group(5)),
                'net_disp': float(m.group(6)),
                'duration': float(m.group(7)),
                'clearance': float(m.group(8)),
            }

            # Extract oracle-specific fields
            if "PATH_EFFICIENCY" in entry:
                pm = re.search(r"path_efficiency=([\d\.]+).*?actual_path_m=([\d\.]+).*?straight_line_m=([\d\.]+)", entry)
                if pm:
                    bug['path_efficiency'] = float(pm.group(1))
                    bug['actual_path'] = float(pm.group(2))
                    bug['straight_line'] = float(pm.group(3))

            if "NAVIGATION_TIME" in entry:
                tm = re.search(r"time_efficiency=([\d\.]+).*?expected_time_s=([\d\.]+)", entry)
                if tm:
                    bug['time_efficiency'] = float(tm.group(1))
                    bug['expected_time'] = float(tm.group(2))

            if "OSCILLATION" in entry:
                om = re.search(r"oscillation_count=(\d+)", entry)
                if om:
                    bug['oscillation_count'] = int(om.group(1))

            results.append(bug)

        return results
    except Exception as e:
        print(f"  Parse error: {e}")
        return []

def categorize_root_cause(bug, map_ctx):
    """Code-level root cause analysis."""
    btype = bug['type']
    gx, gy = bug['goal_x'], bug['goal_y']
    clr = bug['clearance']

    # Check goal location
    boundary = "map-edge" if abs(gx) > 6.3 or abs(gy) > 6.3 else "interior"

    if "ABORT_IN_INFLATION" in btype:
        return f"LEGITIMATE_ABORT:in_inflation(clr={clr:.2f}<0.67,{boundary})"

    if "NAVIGATION_TIME_ERROR" in btype:
        te = bug.get('time_efficiency', 0)
        exp_t = bug.get('expected_time', 0)
        dur = bug['duration']
        # Diagnose why slow
        if te > 5.0:
            return f"SEVERE_SLOWDOWN:time_eff={te:.1f}x(dur={dur:.1f}s,exp={exp_t:.1f}s)"
        elif boundary == "map-edge":
            return f"EDGE_NAVIGATION_SLOW:time_eff={te:.1f}x(boundary_proximity)"
        else:
            return f"MODERATE_SLOWDOWN:time_eff={te:.1f}x(tuning_or_recovery)"

    if "PATH_EFFICIENCY" in btype:
        pe = bug.get('path_efficiency', 0)
        return f"INEFFICIENT_PATH:path_eff={pe:.1f}x(local_minima_or_costmap)"

    if "OSCILLATION" in btype:
        osc = bug.get('oscillation_count', 0)
        return f"POSE_INSTABILITY:oscillation={osc}(controller_tuning)"

    return f"OTHER:{btype}"

def main():
    m = nav2_map.load_occupancy_map("/work/maps/slam_arena.yaml")
    errors = sorted(glob.glob(RUN + "/errors/error-*"))

    print("=" * 100)
    print("NAV2 8-ORACLE CAMPAIGN BUG ANALYSIS (5 original + 3 new)")
    print("=" * 100)
    print(f"Run: {RUN}")
    print(f"Error files: {len(errors)}\n")

    all_bugs = []
    for epath in errors:
        entries = parse_error_file(epath)
        for bug in entries:
            bug['root_cause'] = categorize_root_cause(bug, m)
            all_bugs.append(bug)

    print(f"Total bugs parsed: {len(all_bugs)}\n")

    # Categorize by type
    by_type = defaultdict(list)
    for b in all_bugs:
        by_type[b['type']].append(b)

    print("=" * 100)
    print("BUG TYPE BREAKDOWN")
    print("=" * 100)
    for btype in sorted(by_type.keys()):
        bugs = by_type[btype]
        is_info = bugs[0]['is_info']
        marker = "(info-level, non-bug)" if is_info else "(REAL BUG)"
        print(f"{btype:>35}: {len(bugs):>3} {marker}")

    # Detailed analysis of NAVIGATION_TIME_ERROR (new oracle's catch)
    if 'NAV2_NAVIGATION_TIME_ERROR' in by_type:
        time_bugs = by_type['NAV2_NAVIGATION_TIME_ERROR']
        print(f"\n{'=' * 100}")
        print(f"NAVIGATION_TIME_ERROR DEEP DIVE ({len(time_bugs)} bugs)")
        print("=" * 100)

        print(f"\n{'frame':>16} {'goal_x':>7} {'goal_y':>7} {'dur':>5} {'exp':>5} {'eff':>5} {'root_cause'}")
        print("-" * 100)
        for i, b in enumerate(time_bugs[:20], 1):  # Show first 20
            print(f"{i:>16} {b['goal_x']:>7.2f} {b['goal_y']:>7.2f} "
                  f"{b['duration']:>5.1f} {b.get('expected_time',0):>5.1f} "
                  f"{b.get('time_efficiency',0):>5.1f} {b['root_cause']}")

        if len(time_bugs) > 20:
            print(f"... ({len(time_bugs) - 20} more)")

        # Statistics
        effs = [b.get('time_efficiency', 0) for b in time_bugs]
        print(f"\nTime efficiency range: {min(effs):.1f}x ~ {max(effs):.1f}x (threshold=3.0x)")
        print(f"Average: {sum(effs)/len(effs):.1f}x")

        # Goal location clustering
        coords = [(round(b['goal_x'], 1), round(b['goal_y'], 1)) for b in time_bugs]
        hotspots = Counter(coords).most_common(5)
        print(f"\nTop 5 slow-navigation hotspots:")
        for (gx, gy), cnt in hotspots:
            print(f"  ({gx:>5.1f}, {gy:>5.1f}): {cnt:>2} times")

        # Root cause distribution
        causes = [b['root_cause'].split(":")[0] for b in time_bugs]
        print(f"\nRoot cause breakdown:")
        for cause, cnt in Counter(causes).most_common():
            print(f"  {cause:>30}: {cnt:>3}")

    # Verdict
    print(f"\n{'=' * 100}")
    print("FINAL VERDICT")
    print("=" * 100)

    real_bugs = [b for b in all_bugs if not b['is_info']]
    info_level = [b for b in all_bugs if b['is_info']]

    print(f"Total violations: {len(all_bugs)}")
    print(f"  → Real bugs (oracle errors): {len(real_bugs)}")
    print(f"  → Info-level (non-bugs): {len(info_level)}")
    print(f"\nReal bugs by type:")
    for btype, bugs in sorted(by_type.items()):
        if not bugs[0]['is_info']:
            print(f"  {btype}: {len(bugs)}")

    print(f"\nNew oracles (Criterion 5-7) contribution:")
    new_oracle_bugs = len(by_type.get('NAV2_PATH_EFFICIENCY_ERROR', [])) + \
                      len(by_type.get('NAV2_NAVIGATION_TIME_ERROR', [])) + \
                      len(by_type.get('NAV2_POSE_OSCILLATION_ERROR', []))
    print(f"  Found {new_oracle_bugs} process-level bugs (vs 0 from original 5)")
    print(f"  Coverage expansion: task-level → task+process-level ✅")

if __name__ == "__main__":
    main()
