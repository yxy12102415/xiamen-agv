#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

enable_nvidia_offload() {
  if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __VK_LAYER_NV_optimus=NVIDIA_only
    export GAZEBO_GPU_HINT="nvidia-prime-offload"
    echo "[sim.sh] NVIDIA PRIME render offload enabled for Gazebo/RViz children."
  fi
}

enable_nvidia_offload

WORLD_PATH="${SCRIPT_DIR}/install/agv_gazebo/share/agv_gazebo/worlds/xiamen_empty.sdf"
MODE="${1:-}"

stop_workspace_processes() {
  local signal="${1:-TERM}"
  pkill -"${signal}" -f "${SCRIPT_DIR}/install/.*/lib/.*" 2>/dev/null || true
  pkill -"${signal}" -f "ros2 launch agv_gazebo agv_gazebo_sim.launch.py" 2>/dev/null || true
  pkill -"${signal}" -f "parameter_bridge.*sensing/camera" 2>/dev/null || true
  pkill -"${signal}" -f "parameter_bridge.*/world/xiamen_empty" 2>/dev/null || true
  pkill -"${signal}" -f "parameter_bridge.*sensing/lidar" 2>/dev/null || true
  pkill -"${signal}" -f "rviz2.*${SCRIPT_DIR}/install/agv_gazebo" 2>/dev/null || true
  pkill -"${signal}" -f "static_transform_publisher.*base_link" 2>/dev/null || true
  pkill -"${signal}" -f "lidar_point_marker_publisher" 2>/dev/null || true
  pkill -"${signal}" -f "pointcloud_frame_republisher_node" 2>/dev/null || true
  pkill -"${signal}" -f "ffmpeg .*udp://127\\.0\\.0\\.1:15[12]0[0-5]" 2>/dev/null || true
  pkill -"${signal}" -f "ign gazebo.*${WORLD_PATH}" 2>/dev/null || true
}

stop_simulation() {
  stop_workspace_processes TERM
  sleep 1
  stop_workspace_processes KILL
  pkill -KILL -f "^ign gazebo server" 2>/dev/null || true
  pkill -KILL -f "^ign gazebo gui" 2>/dev/null || true
}

if [[ "${MODE}" == "stop" ]]; then
  stop_simulation
  echo "[sim.sh] Humble simulation stopped."
  exit 0
fi

# A force-closed terminal can leave launch children orphaned.  Those children
# duplicate camera / control publishers on the next run and can halve the
# simulation rate, so clear only processes belonging to this workspace first.
stop_workspace_processes TERM
sleep 1

if pgrep -f "ign gazebo.*${WORLD_PATH}" >/dev/null; then
  pkill -9 -f "ign gazebo.*${WORLD_PATH}" || true
  sleep 1
fi
if pgrep -f "^ign gazebo server" >/dev/null; then
  pkill -9 -f "^ign gazebo server" || true
  sleep 1
fi
stop_workspace_processes KILL
sleep 0.5

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash" 2>/tmp/agv_sim_setup_warnings.log

case "${MODE}" in
  map|scene)
    shift
    export GZ_SIM_RESOURCE_PATH="${SCRIPT_DIR}/install/agv_gazebo/share/agv_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"
    export IGN_GAZEBO_RESOURCE_PATH="${SCRIPT_DIR}/install/agv_gazebo/share/agv_gazebo/models:${IGN_GAZEBO_RESOURCE_PATH:-}"
    exec ign gazebo -r -v 4 "${WORLD_PATH}" "$@"
    ;;
  no-control)
    shift
    exec ros2 launch agv_gazebo agv_gazebo_sim.launch.py \
      use_autoware_map:=false \
      use_rviz:=false \
      enable_centerline_trajectory:=true \
      enable_mpc:=false \
      "$@"
    ;;
  rviz|full)
    shift
    exec ros2 launch agv_gazebo agv_gazebo_sim.launch.py \
      use_autoware_map:=false \
      use_rviz:=true \
      enable_centerline_trajectory:=true \
      enable_mpc:=false \
      "$@"
    ;;
  control|full-control)
    shift
    exec ros2 launch agv_gazebo agv_gazebo_sim.launch.py \
      use_autoware_map:=false \
      use_rviz:=true \
      enable_centerline_trajectory:=true \
      enable_mpc:=true \
      "$@"
    ;;
  remote|teleop)
    shift
    REMOTE_LAUNCH_PID=""
    REMOTE_VIEWER_PID=""
    REMOTE_USE_GAZEBO_GUI="${AGV_REMOTE_USE_GAZEBO_GUI:-false}"
    REMOTE_USE_RVIZ="${AGV_REMOTE_USE_RVIZ:-false}"
    export AGV_SENSOR_PROFILE="${AGV_REMOTE_SENSOR_PROFILE:-remote}"

    cleanup_remote() {
      trap - EXIT INT TERM
      if [[ -n "${REMOTE_VIEWER_PID}" ]] && kill -0 "${REMOTE_VIEWER_PID}" 2>/dev/null; then
        kill -INT "${REMOTE_VIEWER_PID}" 2>/dev/null || true
      fi
      if [[ -n "${REMOTE_LAUNCH_PID}" ]] && kill -0 "${REMOTE_LAUNCH_PID}" 2>/dev/null; then
        kill -INT "${REMOTE_LAUNCH_PID}" 2>/dev/null || true
      fi
      sleep 1
      stop_workspace_processes TERM
      sleep 1
      stop_workspace_processes KILL
      pkill -KILL -f "^ign gazebo server" 2>/dev/null || true
      pkill -KILL -f "^ign gazebo gui" 2>/dev/null || true
      wait "${REMOTE_VIEWER_PID}" 2>/dev/null || true
      wait "${REMOTE_LAUNCH_PID}" 2>/dev/null || true
    }

    on_remote_signal() {
      cleanup_remote
      exit 130
    }

    trap cleanup_remote EXIT
    trap on_remote_signal INT TERM

    ros2 launch agv_gazebo agv_gazebo_sim.launch.py \
      use_autoware_map:=false \
      use_gazebo_gui:="${REMOTE_USE_GAZEBO_GUI}" \
      use_rviz:="${REMOTE_USE_RVIZ}" \
      enable_centerline_trajectory:=false \
      enable_mpc:=false \
      enable_remote_drive:=true \
      control_authority:=REMOTE \
      enable_camera_topics:=false \
      "$@" &
    REMOTE_LAUNCH_PID=$!

    if [[ "${AGV_REMOTE_CAMERA_VIEWER:-0}" == "1" ]] && [[ -n "${DISPLAY:-}" ]]; then
      ros2 run agv_gazebo remote_drive_camera_viewer &
      REMOTE_VIEWER_PID=$!
    fi

    if [[ "${AGV_REMOTE_TERMINAL_CONTROL:-0}" == "1" ]] || [[ -z "${DISPLAY:-}" ]]; then
      echo "[sim.sh] Remote mode started with terminal controls."
      echo "[sim.sh] Remote sensor profile: ${AGV_SENSOR_PROFILE}; direct Gazebo->H.264, 50 ms one-way."
      ros2 run agv_remote_drive remote_drive_keyboard
    else
      echo "[sim.sh] Remote mode started with GTA-style WASD controls."
      echo "[sim.sh] Remote sensor profile: ${AGV_SENSOR_PROFILE}; direct Gazebo->H.264, 50 ms one-way."
      ros2 run agv_remote_drive remote_drive_wasd
    fi
    cleanup_remote
    ;;
  *)
    exec ros2 launch agv_gazebo agv_gazebo_sim.launch.py \
      use_autoware_map:=false \
      use_rviz:=true \
      enable_centerline_trajectory:=true \
      enable_mpc:=false \
      "$@"
    ;;
esac
