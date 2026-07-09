#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

if [ ! -f /opt/ros/galactic/setup.bash ]; then
  exec "${SCRIPT_DIR}/docker.sh" sim "$@"
fi

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

BUILD_BASE="${BUILD_BASE:-${SCRIPT_DIR}/build_galactic}"
INSTALL_BASE="${INSTALL_BASE:-${SCRIPT_DIR}/install_galactic}"
LOG_BASE="${LOG_BASE:-${SCRIPT_DIR}/log_galactic}"
SETUP_FILE="${INSTALL_BASE}/setup.bash"
WORLD_PATH="${INSTALL_BASE}/agv_gazebo/share/agv_gazebo/worlds/xiamen_empty.sdf"
LITE_WORLD_PATH="${INSTALL_BASE}/agv_gazebo/share/agv_gazebo/worlds/xiamen_empty_galactic_lite.sdf"

source /opt/ros/galactic/setup.bash

if [ ! -f "${SETUP_FILE}" ]; then
  echo "[sim.sh] Galactic workspace is not built yet; running colcon build..."
  colcon --log-base "${LOG_BASE}" build \
    --symlink-install \
    --build-base "${BUILD_BASE}" \
    --install-base "${INSTALL_BASE}"
fi

if [ ! -f "${SETUP_FILE}" ]; then
  echo "[sim.sh] Build finished but setup file was not found: ${SETUP_FILE}" >&2
  exit 1
fi

if pgrep -f "ign gazebo.*xiamen_empty" >/dev/null; then
  pkill -9 -f "ign gazebo.*xiamen_empty" || true
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

source "${SETUP_FILE}" 2>/tmp/agv_sim_setup_warnings.log

MODE="${1:-}"
case "${MODE}" in
  map|scene|full-map)
    shift
    export GZ_SIM_RESOURCE_PATH="${INSTALL_BASE}/agv_gazebo/share/agv_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"
    export IGN_GAZEBO_RESOURCE_PATH="${INSTALL_BASE}/agv_gazebo/share/agv_gazebo/models:${IGN_GAZEBO_RESOURCE_PATH:-}"
    exec ign gazebo -r -v 4 "${WORLD_PATH}" "$@"
    ;;
  light|map-rviz)
    shift
    export AGV_GALACTIC_LITE_WORLD=1
    exec ros2 launch agv_gazebo agv_gazebo_sim.launch.py \
      use_autoware_map:=false \
      use_standalone_map:=true \
      use_gazebo_gui:=true \
      use_rviz:=true \
      enable_lidar_topics:=true \
      enable_dynamic_obstacles:=true \
      enable_centerline_trajectory:=true \
      enable_mpc:=false \
      enable_vehicle_interface:=false \
      "$@"
    ;;
  lite|fast)
    shift
    export AGV_GALACTIC_LITE_WORLD=1
    exec ros2 launch agv_gazebo agv_gazebo_sim.launch.py \
      use_autoware_map:=false \
      use_standalone_map:=true \
      use_gazebo_gui:=true \
      use_rviz:=true \
      enable_lidar_topics:=true \
      enable_dynamic_obstacles:=true \
      enable_centerline_trajectory:=true \
      enable_mpc:=false \
      enable_vehicle_interface:=false \
      "$@"
    ;;
  lite-map|fast-map)
    shift
    export GZ_SIM_RESOURCE_PATH="${INSTALL_BASE}/agv_gazebo/share/agv_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"
    export IGN_GAZEBO_RESOURCE_PATH="${INSTALL_BASE}/agv_gazebo/share/agv_gazebo/models:${IGN_GAZEBO_RESOURCE_PATH:-}"
    exec ign gazebo -r -v 2 "${LITE_WORLD_PATH}" "$@"
    ;;
  no-control)
    shift
    unset AGV_GALACTIC_LITE_WORLD
    exec ros2 launch agv_gazebo agv_gazebo_sim.launch.py \
      use_autoware_map:=false \
      use_standalone_map:=true \
      use_gazebo_gui:=true \
      use_rviz:=true \
      enable_lidar_topics:=true \
      enable_dynamic_obstacles:=true \
      enable_centerline_trajectory:=true \
      enable_mpc:=false \
      enable_vehicle_interface:=false \
      "$@"
    ;;
  full|full-rviz)
    shift
    unset AGV_GALACTIC_LITE_WORLD
    exec ros2 launch agv_gazebo agv_gazebo_sim.launch.py \
      use_autoware_map:=false \
      use_rviz:=true \
      enable_lidar_topics:=true \
      enable_dynamic_obstacles:=true \
      enable_centerline_trajectory:=true \
      enable_mpc:=false \
      "$@"
    ;;
  rviz)
    shift
    export AGV_GALACTIC_LITE_WORLD=1
    exec ros2 launch agv_gazebo agv_gazebo_sim.launch.py \
      use_autoware_map:=false \
      use_standalone_map:=false \
      use_gazebo_gui:=false \
      use_rviz:=true \
      enable_lidar_topics:=false \
      enable_dynamic_obstacles:=false \
      enable_centerline_trajectory:=false \
      enable_mpc:=false \
      enable_vehicle_interface:=false \
      "$@"
    ;;
  control)
    shift
    unset AGV_GALACTIC_LITE_WORLD
    exec ros2 launch agv_gazebo agv_gazebo_sim.launch.py \
      use_autoware_map:=false \
      use_standalone_map:=true \
      use_gazebo_gui:=true \
      use_rviz:=true \
      enable_lidar_topics:=true \
      enable_dynamic_obstacles:=true \
      enable_centerline_trajectory:=true \
      enable_mpc:=true \
      "$@"
    ;;
  full-control)
    shift
    unset AGV_GALACTIC_LITE_WORLD
    exec ros2 launch agv_gazebo agv_gazebo_sim.launch.py \
      use_autoware_map:=false \
      use_standalone_map:=true \
      use_gazebo_gui:=true \
      use_rviz:=true \
      enable_lidar_topics:=true \
      enable_dynamic_obstacles:=true \
      enable_centerline_trajectory:=true \
      enable_mpc:=true \
      "$@"
    ;;
  *)
    unset AGV_GALACTIC_LITE_WORLD
    exec ros2 launch agv_gazebo agv_gazebo_sim.launch.py \
      use_autoware_map:=false \
      use_standalone_map:=true \
      use_gazebo_gui:=true \
      use_rviz:=true \
      enable_lidar_topics:=true \
      enable_dynamic_obstacles:=true \
      enable_centerline_trajectory:=true \
      enable_mpc:=false \
      enable_vehicle_interface:=false \
      "$@"
    ;;
esac
