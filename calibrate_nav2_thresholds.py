#!/usr/bin/env python3
"""Nav2 Oracle 阈值标定工具

从干净 campaign 的 bag 数据中提取 arrival_xy_error / arrival_yaw_error 分布，
计算合理的阈值（clean_max × safety_factor），避免误报。

用法:
    python3 calibrate_nav2_thresholds.py <campaign_dir> [--safety-factor 1.3]

输出:
    - 推荐的 NAV2_ARRIVAL_XY_TOL / NAV2_ARRIVAL_YAW_TOL
    - arrival error 分布统计（min/max/mean/p95/p99）
"""

import argparse
import glob
import json
import sys
from pathlib import Path
import numpy as np

def collect_arrival_errors(campaign_dir):
    """从 campaign 的 metadata 文件中提取 arrival errors."""
    meta_files = list(Path(campaign_dir).rglob("metadata/*.json"))
    if not meta_files:
        meta_files = list(Path(campaign_dir).rglob("metadata.json"))

    xy_errors = []
    yaw_errors = []

    for mf in meta_files:
        try:
            with open(mf) as f:
                data = json.load(f)
            fb = data.get('feedback', {})

            # 只统计 oracle_valid=1 且 result=SUCCEEDED 的成功到达
            if fb.get('nav2_oracle_valid', 0) != 1.0:
                continue
            if fb.get('nav2_result_code', 0) != 4:  # 4=SUCCEEDED
                continue

            xy_err = fb.get('nav2_arrival_xy_error')
            yaw_err = fb.get('nav2_arrival_yaw_error')

            if xy_err is not None and xy_err >= 0:
                xy_errors.append(xy_err)
            if yaw_err is not None and yaw_err >= 0:
                yaw_errors.append(yaw_err)
        except Exception as e:
            print(f"[warn] 无法解析 {mf}: {e}", file=sys.stderr)
            continue

    return np.array(xy_errors), np.array(yaw_errors)


def compute_stats(errors, name):
    """计算误差分布统计."""
    if len(errors) == 0:
        print(f"[warn] {name}: 无数据")
        return None

    stats = {
        'name': name,
        'count': len(errors),
        'min': np.min(errors),
        'max': np.max(errors),
        'mean': np.mean(errors),
        'median': np.median(errors),
        'std': np.std(errors),
        'p95': np.percentile(errors, 95),
        'p99': np.percentile(errors, 99),
    }
    return stats


def print_stats(stats):
    """打印统计结果."""
    if stats is None:
        return

    print(f"\n{stats['name']} 分布 (n={stats['count']}):")
    print(f"  min:    {stats['min']:.6f}")
    print(f"  max:    {stats['max']:.6f}")
    print(f"  mean:   {stats['mean']:.6f}")
    print(f"  median: {stats['median']:.6f}")
    print(f"  std:    {stats['std']:.6f}")
    print(f"  p95:    {stats['p95']:.6f}")
    print(f"  p99:    {stats['p99']:.6f}")


def recommend_threshold(stats, safety_factor, current_threshold):
    """基于 clean max 和 safety factor 推荐阈值."""
    if stats is None or stats['max'] == 0:
        return current_threshold

    # 使用 p99 而非 max（max 可能是异常值）
    clean_upper = stats['p99']
    recommended = clean_upper * safety_factor

    # 不低于当前阈值的 80%（避免过度收紧）
    recommended = max(recommended, current_threshold * 0.8)

    return recommended


def main():
    parser = argparse.ArgumentParser(
        description="Nav2 oracle 阈值标定工具（从干净 campaign 数据）"
    )
    parser.add_argument(
        'campaign_dir',
        help='干净 campaign 的输出目录（包含 rosbags/metadata）'
    )
    parser.add_argument(
        '--safety-factor',
        type=float,
        default=1.35,
        help='安全系数（推荐阈值 = clean_p99 × safety_factor），默认 1.35'
    )
    parser.add_argument(
        '--current-xy-tol',
        type=float,
        default=0.35,
        help='当前 NAV2_ARRIVAL_XY_TOL（m），默认 0.35'
    )
    parser.add_argument(
        '--current-yaw-tol',
        type=float,
        default=0.35,
        help='当前 NAV2_ARRIVAL_YAW_TOL（rad），默认 0.35'
    )

    args = parser.parse_args()

    print(f"=== Nav2 Oracle 阈值标定 ===")
    print(f"Campaign: {args.campaign_dir}")
    print(f"Safety factor: {args.safety_factor}")

    # 收集数据
    xy_errors, yaw_errors = collect_arrival_errors(args.campaign_dir)

    if len(xy_errors) == 0 and len(yaw_errors) == 0:
        print("\n[error] 未找到有效的 arrival error 数据")
        print("  确认 campaign_dir 包含 metadata 文件")
        print("  确认有 oracle_valid=1 且 result_code=4 的样本")
        sys.exit(1)

    # 统计
    xy_stats = compute_stats(xy_errors, "xy_error (m)")
    yaw_stats = compute_stats(yaw_errors, "yaw_error (rad)")

    print_stats(xy_stats)
    print_stats(yaw_stats)

    # 推荐阈值
    print(f"\n=== 推荐阈值（safety_factor={args.safety_factor}）===")

    if xy_stats:
        rec_xy = recommend_threshold(xy_stats, args.safety_factor, args.current_xy_tol)
        print(f"NAV2_ARRIVAL_XY_TOL:")
        print(f"  当前: {args.current_xy_tol:.3f} m")
        print(f"  推荐: {rec_xy:.3f} m  (p99={xy_stats['p99']:.4f} × {args.safety_factor})")
        if rec_xy < args.current_xy_tol:
            print(f"  → 可收紧阈值（当前过于宽松，减少误报）")
        elif rec_xy > args.current_xy_tol * 1.1:
            print(f"  → 需放宽阈值（当前过紧，可能产生误报）")
        else:
            print(f"  → 当前阈值合理")

    if yaw_stats:
        rec_yaw = recommend_threshold(yaw_stats, args.safety_factor, args.current_yaw_tol)
        print(f"\nNAV2_ARRIVAL_YAW_TOL:")
        print(f"  当前: {args.current_yaw_tol:.3f} rad")
        print(f"  推荐: {rec_yaw:.3f} rad  (p99={yaw_stats['p99']:.4f} × {args.safety_factor})")
        if rec_yaw < args.current_yaw_tol:
            print(f"  → 可收紧阈值")
        elif rec_yaw > args.current_yaw_tol * 1.1:
            print(f"  → 需放宽阈值")
        else:
            print(f"  → 当前阈值合理")

    print("\n=== 应用推荐阈值 ===")
    print("编辑 src_jazzy/oracles/nav2.py:")
    if xy_stats:
        print(f"  NAV2_ARRIVAL_XY_TOL = {rec_xy:.3f}  # m")
    if yaw_stats:
        print(f"  NAV2_ARRIVAL_YAW_TOL = {rec_yaw:.3f}  # rad")
    print("\n然后重新运行模糊测试验证新阈值的误报率。")


if __name__ == '__main__':
    main()
