#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="${1:-}"

usage() {
  cat <<'USAGE'
Usage: ./run_target.sh <target-profile>

Target profiles:
  turtlebot4_jazzy
  px4_v117_jazzy
  moveit2_jazzy

Environment overrides:
  TURTLEBOT4_MODEL=standard|lite
  PX4_ROOT=/robofuzz/targets/PX4-Autopilot
  START_UXRCE_AGENT=1
  MOVEIT_LAUNCH_PKG=moveit2_tutorials
  MOVEIT_LAUNCH_FILE=move_group.launch.py
USAGE
}

source_ros() {
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash
  else
    echo "ERROR: /opt/ros/jazzy/setup.bash not found. Run inside robofuzz-jazzy image." >&2
    exit 1
  fi
}

run_turtlebot4() {
  local model="${TURTLEBOT4_MODEL:-standard}"
  exec ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py "model:=${model}"
}

run_px4() {
  local px4_root="${PX4_ROOT:-/robofuzz/targets/PX4-Autopilot}"

  if [[ ! -d "${px4_root}" ]]; then
    echo "ERROR: PX4_ROOT does not exist: ${px4_root}" >&2
    exit 1
  fi

  if [[ "${START_UXRCE_AGENT:-1}" == "1" ]]; then
    if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
      echo "ERROR: MicroXRCEAgent not found in PATH." >&2
      exit 1
    fi
    MicroXRCEAgent udp4 -p 8888 &
    local agent_pid="$!"
    trap 'kill "${agent_pid}" >/dev/null 2>&1 || true' EXIT
  fi

  cd "${px4_root}"
  exec env HEADLESS=1 make px4_sitl gz_x500
}

run_moveit2() {
  local launch_pkg="${MOVEIT_LAUNCH_PKG:-moveit2_tutorials}"
  local launch_file="${MOVEIT_LAUNCH_FILE:-move_group.launch.py}"

  if [[ -f "${ROOT_DIR}/moveit_ws/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "${ROOT_DIR}/moveit_ws/install/setup.bash"
  fi

  exec ros2 launch "${launch_pkg}" "${launch_file}"
}

if [[ -z "${TARGET}" || "${TARGET}" == "-h" || "${TARGET}" == "--help" ]]; then
  usage
  exit 0
fi

source_ros

case "${TARGET}" in
  turtlebot4_jazzy)
    run_turtlebot4
    ;;
  px4_v117_jazzy)
    run_px4
    ;;
  moveit2_jazzy)
    run_moveit2
    ;;
  *)
    echo "ERROR: unknown target profile: ${TARGET}" >&2
    usage >&2
    exit 2
    ;;
esac
