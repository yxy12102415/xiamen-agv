#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE_NAME="${XIAMEN_AGV_GALACTIC_IMAGE:-agv-sim-galactic:latest}"
CONTAINER_NAME="${XIAMEN_AGV_GALACTIC_CONTAINER:-xiamen-agv-galactic}"
CONTAINER_PROJECT_DIR="${XIAMEN_AGV_CONTAINER_PROJECT_DIR:-/workspace/xiamen-agv-sim}"
CONTAINER_WS_DIR="${CONTAINER_PROJECT_DIR}/galatic"

usage() {
  cat <<EOF
Usage:
  $0 shell              Enter the Galactic container
  $0 gui                Enter the Galactic container with X11 enabled
  $0 sim [args...]      Run ./sim.sh inside the Galactic container
  $0 build [args...]    Build the galatic workspace inside the container
  $0 root               Enter the container as root
  $0 clean              Remove the named container if it exists

Environment:
  XIAMEN_AGV_GALACTIC_IMAGE      Docker image. Default: agv-sim-galactic:latest
  XIAMEN_AGV_GALACTIC_CONTAINER  Container name. Default: xiamen-agv-galactic
  XIAMEN_AGV_DOCKER_ARGS         Extra docker run args

Examples:
  ./docker.sh shell
  ./docker.sh sim
  ./docker.sh sim control
  ./docker.sh build --packages-select agv_gazebo agv_vehicle_model
EOF
}

rerun_with_docker_group_if_needed() {
  if docker info >/dev/null 2>&1; then
    return
  fi

  if getent group docker | awk -F: -v user="${USER}" '
    $1 == "docker" {
      split($4, members, ",")
      for (i in members) {
        if (members[i] == user) {
          found = 1
        }
      }
    }
    END { exit found ? 0 : 1 }
  '; then
    exec sg docker -c "$(printf "%q " "$0" "$@")"
  fi
}

require_docker() {
  if ! command -v docker >/dev/null 2>&1; then
    echo "docker command not found. Install Docker first." >&2
    exit 127
  fi

  rerun_with_docker_group_if_needed "$@"

  if ! docker info >/dev/null 2>&1; then
    echo "Cannot access the Docker daemon. Try: newgrp docker" >&2
    exit 1
  fi

  if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
    cat >&2 <<EOF
Docker image not found: ${IMAGE_NAME}

This host currently expects the existing Galactic image:
  agv-sim-galactic:latest

Available images:
EOF
    docker images >&2
    exit 1
  fi
}

docker_args() {
  local mode="${1:-shell}"
  local display="${DISPLAY:-}"
  local xauthority="${XAUTHORITY:-${HOME}/.Xauthority}"

  local args=(
    --rm
    --name "${CONTAINER_NAME}"
    --net host
    --ipc host
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
    -e COLCON_BUILD_BASE="${CONTAINER_WS_DIR}/build_galactic"
    -e COLCON_INSTALL_BASE="${CONTAINER_WS_DIR}/install_galactic"
    -e COLCON_LOG_BASE="${CONTAINER_WS_DIR}/log_galactic"
    -v "${PROJECT_DIR}:${CONTAINER_PROJECT_DIR}:rw"
    -w "${CONTAINER_WS_DIR}"
  )

  if [ -t 0 ]; then
    args+=(-it)
  else
    args+=(-i)
  fi

  if [ "${mode}" = "gui" ] || [ "${mode}" = "sim" ]; then
    if [ -n "${display}" ]; then
      xhost +local:docker >/dev/null 2>&1 || true
      args+=(
        -e DISPLAY="${display}"
        -e QT_X11_NO_MITSHM=1
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw
      )
      if [ -f "${xauthority}" ]; then
        args+=(-e XAUTHORITY=/tmp/.docker.xauth -v "${xauthority}:/tmp/.docker.xauth:ro")
      fi
    fi
  fi

  if [ -n "${XIAMEN_AGV_DOCKER_ARGS:-}" ]; then
    # shellcheck disable=SC2206
    args+=(${XIAMEN_AGV_DOCKER_ARGS})
  fi

  printf '%s\n' "${args[@]}"
}

run_container() {
  local mode="${1:-shell}"
  local user="${2:-agv}"
  shift 2 || true

  mapfile -t args < <(docker_args "${mode}")
  docker run "${args[@]}" --user "${user}" "${IMAGE_NAME}" "$@"
}

case "${1:-shell}" in
  shell)
    require_docker "$@"
    run_container shell agv /bin/bash
    ;;
  gui)
    require_docker "$@"
    run_container gui agv /bin/bash
    ;;
  sim)
    shift || true
    require_docker sim "$@"
    run_container sim agv /bin/bash -lc './sim.sh "$@"' bash "$@"
    ;;
  build)
    shift || true
    require_docker build "$@"
    run_container shell agv /bin/bash -lc \
      'source /opt/ros/galactic/setup.bash && colcon --log-base log_galactic build --symlink-install --build-base build_galactic --install-base install_galactic "$@"' \
      bash "$@"
    ;;
  root)
    require_docker "$@"
    run_container shell root /bin/bash
    ;;
  clean)
    docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
