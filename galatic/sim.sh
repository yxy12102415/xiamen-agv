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
if pgrep -f "ign gazebo.*${WORLD_PATH}" >/dev/null; then
  pkill -9 -f "ign gazebo.*${WORLD_PATH}" || true
  sleep 1
fi
if pgrep -f "^ign gazebo server" >/dev/null; then
  pkill -9 -f "^ign gazebo server" || true
  sleep 1
fi
pkill -TERM -f "lidar_point_marker_publisher" 2>/dev/null || true
pkill -TERM -f "pointcloud_frame_republisher_node" 2>/dev/null || true
pkill -TERM -f "parameter_bridge.*sensing/lidar" 2>/dev/null || true
sleep 0.5

source /opt/ros/galactic/setup.bash
source "${SCRIPT_DIR}/install/setup.bash" 2>/tmp/agv_sim_setup_warnings.log

MODE="${1:-}"
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
  *)
    exec ros2 launch agv_gazebo agv_gazebo_sim.launch.py \
      use_autoware_map:=false \
      use_rviz:=true \
      enable_centerline_trajectory:=true \
      enable_mpc:=false \
      "$@"
    ;;
esac
