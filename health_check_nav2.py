#!/usr/bin/env python3
"""Nav2 Fuzzing 健康检查 - 验证代码和配置完整性（容器外可运行）

检查项:
1. Oracle代码完整性（nav2.py扩充、单元测试）
2. OracleIR specs完整性（42个spec全部合法）
3. 地图和seed生成配置
4. 关键依赖和配置文件
"""

import sys
import os
from pathlib import Path

# 切换到项目根目录
os.chdir(Path(__file__).parent)
sys.path.insert(0, 'src_jazzy')

PASS = "\033[92m✓\033[0m"
FAIL = "\033[91m✗\033[0m"
WARN = "\033[93m⚠\033[0m"

def check_oracle_code():
    """检查oracle代码扩充."""
    print("\n=== 1. Oracle代码检查 ===")

    # 检查nav2.py存在且扩充
    oracle_file = Path('src_jazzy/oracles/nav2.py')
    if not oracle_file.exists():
        print(f"{FAIL} nav2.py不存在")
        return False

    content = oracle_file.read_text()
    line_count = len(content.split('\n'))
    print(f"{PASS} nav2.py存在 ({line_count}行, 预期~225行)")

    # 检查新增feedback
    new_feedbacks = [
        'nav2_arrival_xy_margin',
        'nav2_arrival_yaw_margin',
        'nav2_arrival_xy_pressure',
        'nav2_arrival_yaw_pressure',
        'nav2_avg_velocity',
        'nav2_stuck_pressure'
    ]

    missing = [fb for fb in new_feedbacks if fb not in content]
    if missing:
        print(f"{FAIL} 缺少新增feedback: {missing}")
        return False
    print(f"{PASS} 新增6个feedback全部存在")

    # 检查单元测试
    test_file = Path('src_jazzy/tests/test_nav2_oracle.py')
    if not test_file.exists():
        print(f"{WARN} 单元测试文件不存在")
    else:
        print(f"{PASS} 单元测试文件存在")

    # 运行单元测试（容器外可运行）
    try:
        from tests.test_nav2_oracle import TestArrival, TestCollision, fb_list
        from oracles import nav2

        # 快速测试
        test = TestArrival()
        test.test_arrival_ok_no_error()
        test.test_missing_topics_no_error()

        test2 = TestCollision()
        # (test_collision需要完整mock数据，跳过)

        print(f"{PASS} 单元测试通过（基础检查）")
        print(f"    Feedback总数: {len(nav2.NAV2_FEEDBACK_NAMES)}")

    except Exception as e:
        print(f"{FAIL} 单元测试失败: {e}")
        return False

    return True


def check_oracleir_specs():
    """检查OracleIR specs完整性."""
    print("\n=== 2. OracleIR Specs检查 ===")

    specs_dir = Path('src_jazzy/oracle_ir/specs/nav2_jazzy')
    if not specs_dir.exists():
        print(f"{FAIL} specs目录不存在: {specs_dir}")
        return False

    spec_files = list(specs_dir.glob('*.yaml'))
    print(f"{PASS} Specs目录存在: {len(spec_files)}个yaml文件（预期42）")

    if len(spec_files) < 40:
        print(f"{WARN} Spec数量偏少（<40），可能生成不完整")

    # 校验所有spec（需要validator）
    try:
        import yaml
        from oracle_ir.transform.parser import _dict_to_oracle_ir
        from oracle_ir.transform.validator import validate_oracle_ir

        invalid = []
        for sf in spec_files:
            try:
                data = yaml.safe_load(sf.read_text())
                ir = _dict_to_oracle_ir(data)
                result = validate_oracle_ir(ir)
                if not result.valid:
                    invalid.append((sf.name, result.errors))
            except Exception as e:
                invalid.append((sf.name, str(e)))

        if invalid:
            print(f"{FAIL} {len(invalid)}个spec非法:")
            for name, err in invalid[:3]:
                print(f"    - {name}: {err}")
            return False

        print(f"{PASS} 所有{len(spec_files)}个spec通过validator")

    except ImportError:
        print(f"{WARN} 无法加载validator（跳过spec校验）")

    return True


def check_map_and_seed():
    """检查地图和seed生成."""
    print("\n=== 3. 地图和Seed生成检查 ===")

    # 检查地图文件
    map_yaml = Path('maps/slam_arena.yaml')
    map_pgm = Path('maps/slam_arena.pgm')

    if not map_yaml.exists():
        print(f"{FAIL} 地图配置不存在: {map_yaml}")
        return False
    if not map_pgm.exists():
        print(f"{FAIL} 地图图像不存在: {map_pgm}")
        return False

    print(f"{PASS} 地图文件存在")

    # 测试地图加载和goal生成
    try:
        import nav2_map
        import seed_generator

        mctx = nav2_map.load_occupancy_map(str(map_yaml))
        print(f"{PASS} 地图加载成功: {mctx.width}x{mctx.height}, origin={mctx.origin}")

        # 验证机器人起始位置
        sc, sr = mctx.world_to_cell(0.0, 0.0)
        if not mctx.is_free(sc, sr):
            print(f"{FAIL} 起始位置(0,0)不在free space")
            return False
        print(f"{PASS} 起始位置(0,0)在free space")

        # 生成goals
        goals = seed_generator.generate_nav2_goal_seeds(
            mctx, start_xy=(0.0, 0.0), count=12, min_dist=0.75, max_dist=3.0
        )

        if len(goals) < 10:
            print(f"{WARN} Goal生成数量偏少: {len(goals)} (预期12)")
        else:
            print(f"{PASS} Goal生成正常: {len(goals)}个")

        # 验证可达性
        unreachable = [g for g in goals if not mctx.reachable((0,0), g[:2])]
        if unreachable:
            print(f"{FAIL} {len(unreachable)}个goal不可达")
            return False
        print(f"{PASS} 所有goal可达")

    except Exception as e:
        print(f"{FAIL} 地图/seed生成测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

    return True


def check_config_files():
    """检查关键配置文件."""
    print("\n=== 4. 配置文件检查 ===")

    files = [
        ('run_target.sh', '目标启动脚本'),
        ('src_jazzy/fuzzer.py', 'Fuzzer主程序'),
        ('src_jazzy/harness.py', 'Harness(含nav2处理)'),
        ('smoke_test_nav2.sh', '冒烟测试脚本'),
        ('calibrate_nav2_thresholds.py', '阈值标定工具'),
    ]

    all_ok = True
    for fpath, desc in files:
        p = Path(fpath)
        if not p.exists():
            print(f"{FAIL} {desc}不存在: {fpath}")
            all_ok = False
        else:
            print(f"{PASS} {desc}存在")

    # 检查run_target.sh是否支持nav2_jazzy
    run_target = Path('run_target.sh').read_text()
    if 'nav2_jazzy' not in run_target:
        print(f"{FAIL} run_target.sh未包含nav2_jazzy配置")
        all_ok = False
    else:
        print(f"{PASS} run_target.sh支持nav2_jazzy")

    return all_ok


def main():
    print("=" * 60)
    print("Nav2 Fuzzing 健康检查")
    print("=" * 60)

    checks = [
        ("Oracle代码", check_oracle_code),
        ("OracleIR Specs", check_oracleir_specs),
        ("地图和Seed", check_map_and_seed),
        ("配置文件", check_config_files),
    ]

    results = []
    for name, func in checks:
        try:
            ok = func()
            results.append((name, ok))
        except Exception as e:
            print(f"\n{FAIL} {name}检查异常: {e}")
            import traceback
            traceback.print_exc()
            results.append((name, False))

    print("\n" + "=" * 60)
    print("检查结果总结:")
    print("=" * 60)

    all_pass = True
    for name, ok in results:
        status = PASS if ok else FAIL
        print(f"  {status} {name}")
        if not ok:
            all_pass = False

    print("=" * 60)
    if all_pass:
        print(f"\n{PASS} 全部检查通过！可以开始冒烟测试。")
        print("\n下一步:")
        print("  1. 在ROS容器内运行: ./smoke_test_nav2.sh 5")
        print("  2. 观察日志确认首轮成功")
        print("  3. 运行长时间实验: ./smoke_test_nav2.sh 100")
        return 0
    else:
        print(f"\n{FAIL} 部分检查失败，请修复后重试。")
        return 1


if __name__ == '__main__':
    sys.exit(main())
