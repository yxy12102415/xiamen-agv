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
    echo "[autosim.sh] NVIDIA PRIME render offload enabled for Gazebo/RViz children."
  fi
}

enable_nvidia_offload

source /opt/ros/humble/setup.bash
source /data/projects/autoware/install/setup.bash 2>/tmp/autoware_sim_setup_warnings.log
source "${SCRIPT_DIR}/install/setup.bash" 2>/tmp/agv_autosim_setup_warnings.log

LAUNCH_PID=""

cleanup() {
  local status=$?
  trap - INT TERM EXIT

  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -KILL "${LAUNCH_PID}" 2>/dev/null || true
  fi

  pkill -TERM -f "ign gazebo.*xiamen_empty.sdf" 2>/dev/null || true
  pkill -TERM -f "rviz2.*stage4_lidar.rviz" 2>/dev/null || true
  pkill -TERM -f "parameter_bridge.*sensing/lidar" 2>/dev/null || true
  pkill -TERM -f "pointcloud_frame_republisher_node" 2>/dev/null || true

  exit "${status}"
}

trap cleanup INT TERM EXIT

ros2 launch agv_bringup autoware_gazebo_sim.launch.py "$@" &
LAUNCH_PID=$!
wait "${LAUNCH_PID}"
