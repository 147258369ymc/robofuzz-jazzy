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
  TURTLEBOT4_HEADLESS=1|0
  TURTLEBOT4_MODEL=standard|lite
  TURTLEBOT4_GAZEBO=ignition|classic
  TURTLEBOT4_GZ_ARGS="-s -r"
  TURTLEBOT4_USE_XVFB=1|0
  TURTLEBOT4_SPLIT_LAUNCH=1|0
  TURTLEBOT4_WORLD=warehouse|simple
  TURTLEBOT4_GUI_WORLD=simple
  QT_QPA_PLATFORM=offscreen
  PX4_ROOT=/robofuzz/targets/PX4-Autopilot
  START_UXRCE_AGENT=1
  PX4_UXRCE_PORT=8888
  MOVEIT_LAUNCH_PKG=moveit_resources_panda_moveit_config
  MOVEIT_LAUNCH_FILE=demo.launch.py
  MOVEIT_HEADLESS=1|0
  MOVEIT_WITH_RVIZ=1|0
  MOVEIT_RVIZ_CONFIG=/work/src_jazzy/rviz/moveit_fuzz.rviz
USAGE
}

source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck source=/dev/null
  source "${setup_file}"
  local rc="$?"
  set -u
  return "${rc}"
}

source_ros() {
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    source_setup /opt/ros/jazzy/setup.bash
  else
    echo "ERROR: /opt/ros/jazzy/setup.bash not found. Run inside robofuzz-jazzy image." >&2
    exit 1
  fi

  if [[ -f /robofuzz/px4_ros2_ws/install/setup.bash ]]; then
    source_setup /robofuzz/px4_ros2_ws/install/setup.bash
  fi
}

pkg_share() {
  local pkg="$1"
  local prefix
  prefix="$(ros2 pkg prefix "${pkg}")"
  printf '%s/share/%s\n' "${prefix}" "${pkg}"
}

install_custom_tb4_world() {
  local world="$1"
  local tb4_gz_share="$2"
  local src="/work/worlds/${world}.sdf"
  local dst="${tb4_gz_share}/worlds/${world}.sdf"

  if [[ "${world}" == */* || ! -f "${src}" ]]; then
    return
  fi

  if [[ ! -f "${dst}" ]] || ! cmp -s "${src}" "${dst}"; then
    cp -f "${src}" "${dst}"
  fi
}

run_turtlebot4() {
  local model="${TURTLEBOT4_MODEL:-standard}"
  local gazebo="${TURTLEBOT4_GAZEBO:-ignition}"
  local headless="${TURTLEBOT4_HEADLESS:-1}"
  local split_launch="${TURTLEBOT4_SPLIT_LAUNCH:-1}"
  local default_world="warehouse"
  if [[ "${headless}" != "1" ]]; then
    default_world="${TURTLEBOT4_GUI_WORLD:-simple}"
  fi
  local world="${TURTLEBOT4_WORLD:-${default_world}}"
  local tb4_gz_share
  local create_gz_share
  local ros_share="/opt/ros/${ROS_DISTRO}/share"

  tb4_gz_share="$(pkg_share turtlebot4_gz_bringup)"
  create_gz_share="$(pkg_share irobot_create_gz_bringup)"
  install_custom_tb4_world "${world}" "${tb4_gz_share}"
  # Add /work/worlds to both GUI and headless resource paths so the lightweight
  # RoboFuzz worlds can be selected by world name (e.g. TURTLEBOT4_WORLD=simple).
  export GZ_SIM_RESOURCE_PATH="/work/worlds:${tb4_gz_share}/worlds:${create_gz_share}/worlds:${ros_share}:${GZ_SIM_RESOURCE_PATH:-}"

  # Support custom world file paths (e.g., /work/worlds/empty.sdf)
  # If world contains '/', treat as full path; otherwise append .sdf
  local world_arg
  if [[ "${world}" == */* ]]; then
    world_arg="${world}"
  elif [[ -f "/work/worlds/${world}.sdf" ]]; then
    world_arg="/work/worlds/${world}.sdf"
  else
    world_arg="${world}.sdf"
  fi

  # GUI mode needs gz_args without -s (server-only); headless uses -s -r
  local gz_args
  if [[ "${headless}" == "1" ]]; then
    gz_args="${TURTLEBOT4_GZ_ARGS:--s -r}"
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
  else
    gz_args="${TURTLEBOT4_GZ_ARGS:--r}"
    # GUI mode: unset offscreen to allow X11 display
    unset QT_QPA_PLATFORM
  fi

  if [[ "${headless}" == "1" || "${split_launch}" == "1" ]]; then
    local sim_pid=""
    local clock_pid=""
    local use_xvfb="${TURTLEBOT4_USE_XVFB:-${headless}}"

    export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

    if [[ "${use_xvfb}" == "1" ]] && command -v xvfb-run >/dev/null 2>&1; then
      xvfb-run -a -s "-screen 0 1280x1024x24" \
        ros2 launch ros_gz_sim gz_sim.launch.py "gz_args:=${gz_args} ${world_arg}" &
    else
      ros2 launch ros_gz_sim gz_sim.launch.py "gz_args:=${gz_args} ${world_arg}" &
    fi
    sim_pid="$!"
    ros2 run ros_gz_bridge parameter_bridge \
      "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
      --ros-args -r __node:=clock_bridge &
    clock_pid="$!"
    trap 'kill "${clock_pid}" "${sim_pid}" >/dev/null 2>&1 || true' EXIT
    sleep 5

    python3 "${ROOT_DIR}/src_jazzy/launch/turtlebot4_fuzz_spawn.launch.py" \
      "model:=${model}" \
      "world:=${world}" \
      rviz:=false
    return
  fi

  # Legacy GUI mode: use the upstream complete TurtleBot4 launch path only
  # when explicitly requested. RoboFuzz defaults to split no-dock launch
  # because the upstream path starts docked and may trigger autonomous motion.
  ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py \
    "model:=${model}" \
    "gazebo:=${gazebo}" \
    "world:=${world}" \
    rviz:=false \
    "gz_args:=${gz_args}"
}

run_px4() {
  local px4_root="${PX4_ROOT:-/robofuzz/targets/PX4-Autopilot}"
  local agent_pid=""
  local uxrce_port="${PX4_UXRCE_PORT:-8888}"

  if [[ ! -d "${px4_root}" ]]; then
    echo "ERROR: PX4_ROOT does not exist: ${px4_root}" >&2
    exit 1
  fi

  if [[ "${START_UXRCE_AGENT:-1}" == "1" ]]; then
    if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
      echo "ERROR: MicroXRCEAgent not found in PATH." >&2
      exit 1
    fi
    MicroXRCEAgent udp4 -p "${uxrce_port}" &
    agent_pid="$!"
    trap "kill ${agent_pid} >/dev/null 2>&1 || true" EXIT
  fi

  cd "${px4_root}"
  env HEADLESS=1 make px4_sitl_default gz_x500
}

run_moveit2() {
  local launch_pkg="${MOVEIT_LAUNCH_PKG:-moveit_resources_panda_moveit_config}"
  local launch_file="${MOVEIT_LAUNCH_FILE:-demo.launch.py}"
  local with_rviz="${MOVEIT_WITH_RVIZ:-1}"
  local headless_launch="${ROOT_DIR}/src_jazzy/launch/moveit2_panda_headless.launch.py"
  local default_rviz_config="${ROOT_DIR}/src_jazzy/rviz/moveit_fuzz.rviz"
  local rviz_config="${MOVEIT_RVIZ_CONFIG:-}"
  local launch_args=("ros2_control_hardware_type:=mock_components")

  if [[ -f "${ROOT_DIR}/moveit_ws/install/setup.bash" ]]; then
    source_setup "${ROOT_DIR}/moveit_ws/install/setup.bash"
  fi

  if [[ "${MOVEIT_HEADLESS:-0}" == "1" || -z "${DISPLAY:-}" ]]; then
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
  fi

  if [[ "${with_rviz}" == "0" ]]; then
    exec python3 "${headless_launch}" "${launch_args[@]}"
  fi

  if [[ -z "${rviz_config}" && -f "${default_rviz_config}" ]]; then
    rviz_config="${default_rviz_config}"
  fi
  if [[ -n "${rviz_config}" ]]; then
    launch_args+=("rviz_config:=${rviz_config}")
  fi

  exec ros2 launch "${launch_pkg}" "${launch_file}" "${launch_args[@]}"
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
