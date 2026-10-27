#!/usr/bin/env bash
# Nav2 Smoke Test - 优化版，用于长时间实验验证
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 配置
MAXLOOP="${1:-5}"  # 默认 5 轮，可传参覆盖
OUTPUT_DIR="${2:-/tmp/nav2_smoke_$(date +%Y%m%d_%H%M%S)}"
LOG_FILE="$OUTPUT_DIR/smoke_test.log"

mkdir -p "$OUTPUT_DIR"

echo "========================================" | tee -a "$LOG_FILE"
echo "Nav2 Smoke Test - $(date)" | tee -a "$LOG_FILE"
echo "  Rounds: $MAXLOOP" | tee -a "$LOG_FILE"
echo "  Output: $OUTPUT_DIR" | tee -a "$LOG_FILE"
echo "========================================" | tee -a "$LOG_FILE"

# 优化 1: 首轮冷启动用更长超时（600s），后续 300s
export NAV2_ACTION_TIMEOUT_SEC=600
echo "[smoke] 首轮超时: ${NAV2_ACTION_TIMEOUT_SEC}s (冷启动)" | tee -a "$LOG_FILE"

# 优化 2: headless 模式（faster）
export TURTLEBOT4_HEADLESS=1
export TURTLEBOT4_WORLD=slam_arena
export TURTLEBOT4_MODEL=standard

# 优化 3: 详细日志级别
LOG_LEVEL="INFO"

# 运行首轮（冷启动）
echo "[smoke] ===== Round 1 (cold start) =====" | tee -a "$LOG_FILE"
timeout 900 python3 -u src_jazzy/fuzzer.py \
  --target-profile nav2_jazzy \
  --schedule single \
  --maxloop 1 \
  --output-dir "$OUTPUT_DIR/round1" \
  --log-level "$LOG_LEVEL" 2>&1 | tee -a "$LOG_FILE" || {
    echo "[smoke] ✗ Round 1 failed or timed out" | tee -a "$LOG_FILE"
    exit 1
}

# 检查首轮结果
if [ -d "$OUTPUT_DIR/round1/rosbags" ]; then
    bag_count=$(find "$OUTPUT_DIR/round1/rosbags" -name "*.db3" 2>/dev/null | wc -l)
    echo "[smoke] Round 1 bags: $bag_count" | tee -a "$LOG_FILE"
fi

if [ -d "$OUTPUT_DIR/round1/errors" ]; then
    error_count=$(find "$OUTPUT_DIR/round1/errors" -type f 2>/dev/null | wc -l)
    echo "[smoke] Round 1 errors: $error_count" | tee -a "$LOG_FILE"
    if [ "$error_count" -gt 0 ]; then
        echo "[smoke] Error samples:" | tee -a "$LOG_FILE"
        find "$OUTPUT_DIR/round1/errors" -type f 2>/dev/null | head -3 | while read f; do
            echo "  - $(basename $f)" | tee -a "$LOG_FILE"
            head -5 "$f" | sed 's/^/    /' | tee -a "$LOG_FILE"
        done
    fi
fi

# 优化 4: 后续轮次用正常超时
if [ "$MAXLOOP" -gt 1 ]; then
    export NAV2_ACTION_TIMEOUT_SEC=300
    echo "[smoke] 后续轮次超时: ${NAV2_ACTION_TIMEOUT_SEC}s" | tee -a "$LOG_FILE"

    remaining=$((MAXLOOP - 1))
    echo "[smoke] ===== Rounds 2-${MAXLOOP} (warm) =====" | tee -a "$LOG_FILE"
    timeout $((remaining * 400)) python3 -u src_jazzy/fuzzer.py \
      --target-profile nav2_jazzy \
      --schedule single \
      --maxloop "$remaining" \
      --output-dir "$OUTPUT_DIR/round2plus" \
      --log-level "$LOG_LEVEL" 2>&1 | tee -a "$LOG_FILE" || {
        echo "[smoke] ✗ Warm rounds failed or timed out" | tee -a "$LOG_FILE"
        # 非致命，继续分析
    }

    # 统计后续轮次
    if [ -d "$OUTPUT_DIR/round2plus/rosbags" ]; then
        bag_count=$(find "$OUTPUT_DIR/round2plus/rosbags" -name "*.db3" 2>/dev/null | wc -l)
        echo "[smoke] Rounds 2+ bags: $bag_count" | tee -a "$LOG_FILE"
    fi

    if [ -d "$OUTPUT_DIR/round2plus/errors" ]; then
        error_count=$(find "$OUTPUT_DIR/round2plus/errors" -type f 2>/dev/null | wc -l)
        echo "[smoke] Rounds 2+ errors: $error_count" | tee -a "$LOG_FILE"
    fi
fi

# 优化 5: 分析 feedback 数据（验证新增 feedback 工作）
echo "[smoke] ===== Feedback Analysis =====" | tee -a "$LOG_FILE"
python3 <<'PYEOF' 2>&1 | tee -a "$LOG_FILE"
import sys, os, glob, json
sys.path.insert(0, 'src_jazzy')

# 找所有 metadata 文件（含 feedback）
meta_files = []
for pattern in ['*/round*/metadata/*.json', '*/round*/metadata.json']:
    meta_files.extend(glob.glob(pattern))

if not meta_files:
    print("[smoke] No metadata files found")
    sys.exit(0)

print(f"[smoke] Found {len(meta_files)} metadata files")

# 统计 feedback 出现频率
feedback_stats = {}
for mf in meta_files:
    try:
        with open(mf) as f:
            data = json.load(f)
        fb = data.get('feedback', {})
        for name, value in fb.items():
            if name not in feedback_stats:
                feedback_stats[name] = {'count': 0, 'sum': 0.0, 'max': float('-inf'), 'min': float('inf')}
            feedback_stats[name]['count'] += 1
            if isinstance(value, (int, float)):
                feedback_stats[name]['sum'] += value
                feedback_stats[name]['max'] = max(feedback_stats[name]['max'], value)
                feedback_stats[name]['min'] = min(feedback_stats[name]['min'], value)
    except Exception as e:
        pass

# 重点关注新增的 6 个 feedback
new_feedbacks = [
    'nav2_arrival_xy_margin',
    'nav2_arrival_yaw_margin',
    'nav2_arrival_xy_pressure',
    'nav2_arrival_yaw_pressure',
    'nav2_avg_velocity',
    'nav2_stuck_pressure'
]

print("\n[smoke] 新增 Feedback 统计:")
for name in new_feedbacks:
    if name in feedback_stats:
        s = feedback_stats[name]
        avg = s['sum'] / s['count'] if s['count'] > 0 else 0
        print(f"  {name}:")
        print(f"    count={s['count']}, avg={avg:.4f}, min={s['min']:.4f}, max={s['max']:.4f}")
    else:
        print(f"  {name}: NOT FOUND (可能 oracle 未触发)")

# 检查是否有错误被捕获
print("\n[smoke] Oracle 错误统计:")
for pattern in ['*/round*/errors/*']:
    err_files = glob.glob(pattern)
    if err_files:
        print(f"  发现 {len(err_files)} 个错误样本")
        for ef in err_files[:3]:
            print(f"    - {os.path.basename(ef)}")
    else:
        print("  无错误样本（全部通过 oracle 或 oracle 未触发）")
PYEOF

echo "[smoke] ========================================" | tee -a "$LOG_FILE"
echo "[smoke] Smoke test 完成: $OUTPUT_DIR" | tee -a "$LOG_FILE"
echo "[smoke] 日志: $LOG_FILE" | tee -a "$LOG_FILE"
echo "[smoke] ========================================" | tee -a "$LOG_FILE"
