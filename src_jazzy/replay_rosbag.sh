#!/bin/bash
# ============================================================
# Rosbag回放脚本 - 回放fuzzer录制的传感器数据
#
# 用法:
#   ./replay_rosbag.sh <exec_timestamp>
#
# 示例:
#   ./replay_rosbag.sh 1779245432.6389372    # [0.22, -2.84]x5 coupling case
#   ./replay_rosbag.sh 1779244258.704316     # [-0.22, 0]x5 backward case
#   ./replay_rosbag.sh 1779247773.447483     # angular zigzag case
#
# 注意: rosbag回放只重放录制的topic数据(/odom, /imu, /scan, /cmd_vel)
#       不会驱动Gazebo中的机器人运动。要看Gazebo动画请用 replay_case.py
# ============================================================

LOGS_DIR="/robofuzz/src/logs/20260520-022554"

if [ -z "$1" ]; then
    echo "Usage: $0 <exec_timestamp>"
    echo ""
    echo "Available timestamps with errors (first 20):"
    ls "$LOGS_DIR/errors/" | head -20 | sed 's/error-/  /'
    echo "  ..."
    echo ""
    echo "Typical cases:"
    echo "  1779245432.6389372  - [0.22, -2.84]x5 coupling bug"
    echo "  1779244258.704316   - [-0.22, 0]x5 backward case"
    echo "  1779247773.447483   - angular zigzag"
    echo "  1779247568.9032757  - linear reversal + lateral"
    echo "  1779253276.0757842  - extreme mutation"
    exit 1
fi

TIMESTAMP=$1
BAG_DIR="$LOGS_DIR/rosbags/$TIMESTAMP/states-0.bag"

if [ ! -d "$BAG_DIR" ]; then
    echo "ERROR: Rosbag not found at $BAG_DIR"
    echo "Available rosbags:"
    ls "$LOGS_DIR/rosbags/" | grep "$TIMESTAMP" || echo "  (none matching)"
    exit 1
fi

echo "============================================================"
echo "Replaying rosbag for execution: $TIMESTAMP"
echo "Bag path: $BAG_DIR"
echo "============================================================"
echo ""
echo "Topics in bag:"
ros2 bag info "$BAG_DIR" 2>/dev/null | grep -A20 "Topic information"
echo ""
echo "Starting playback (Ctrl+C to stop)..."
echo ""

if ros2 bag play -h 2>&1 | grep -q -- '--clock'; then
    ros2 bag play "$BAG_DIR" --clock
else
    echo "Note: this ROS 2 version does not support 'ros2 bag play --clock'; playing without --clock."
    ros2 bag play "$BAG_DIR"
fi
