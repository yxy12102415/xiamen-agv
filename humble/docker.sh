#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE_NAME="${XIAMEN_AGV_HUMBLE_IMAGE:-agv-sim-humble:latest}"
DEV_IMAGE_NAME="${XIAMEN_AGV_HUMBLE_DEV_IMAGE:-agv-sim-humble-dev:latest}"
BASE_IMAGE="${XIAMEN_AGV_HUMBLE_BASE_IMAGE:-osrf/ros:humble-simulation}"
CONTAINER_NAME="${XIAMEN_AGV_HUMBLE_CONTAINER:-xiamen-agv-humble}"
DEV_CONTAINER_NAME="${XIAMEN_AGV_HUMBLE_DEV_CONTAINER:-xiamen-agv-humble-dev}"
CONTAINER_PROJECT_DIR="${XIAMEN_AGV_CONTAINER_PROJECT_DIR:-/workspace/xiamen-agv-sim}"
CONTAINER_WS_DIR="${CONTAINER_PROJECT_DIR}/humble"

usage() {
  cat <<EOF
Usage:
  $0 build-image        Build the runtime Humble Docker image
  $0 build-image-dev    Build the developer Humble Docker image
  $0 shell              Enter the Humble container
  $0 gui                Enter the Humble container with X11 enabled
  $0 sim [args...]      Run ./sim.sh inside the Humble container
  $0 build [args...]    Build the Humble workspace inside the container
  $0 gpu-check          Check NVIDIA Docker GPU access
  $0 root               Enter the container as root
  $0 clean              Remove the named container if it exists

Environment:
  XIAMEN_AGV_HUMBLE_IMAGE      Docker image. Default: agv-sim-humble:latest
  XIAMEN_AGV_HUMBLE_DEV_IMAGE  Developer image. Default: agv-sim-humble-dev:latest
  XIAMEN_AGV_HUMBLE_BASE_IMAGE Base ROS image. Default: osrf/ros:humble-simulation
  XIAMEN_AGV_HUMBLE_CONTAINER  Container name. Default: xiamen-agv-humble
  XIAMEN_AGV_HUMBLE_DEV_CONTAINER Developer container name. Default: xiamen-agv-humble-dev
  XIAMEN_AGV_DOCKER_ARGS       Extra docker run args

Examples:
  ./docker.sh build-image
  ./docker.sh build-image-dev
  ./docker.sh shell
  ./docker.sh sim
  ./docker.sh sim remote
  ./docker.sh build --packages-select agv_gazebo agv_vehicle_model
  ./docker.sh gpu-check
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

require_docker_daemon() {
  if ! command -v docker >/dev/null 2>&1; then
    echo "docker command not found. Install Docker first." >&2
    exit 127
  fi

  rerun_with_docker_group_if_needed "$@"

  if ! docker info >/dev/null 2>&1; then
    echo "Cannot access the Docker daemon. Try: newgrp docker" >&2
    exit 1
  fi
}

require_docker() {
  require_docker_daemon "$@"

  if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
    cat >&2 <<EOF
Docker image not found: ${IMAGE_NAME}

Build it locally:
  ./docker.sh build-image

Or load a shared image archive:
  docker load < agv-sim-humble_latest.tar.gz

Available images:
EOF
    docker images >&2
    exit 1
  fi
}

require_image() {
  local image_name="${1:-}"
  if [ -z "${image_name}" ]; then
    echo "Internal error: missing image name." >&2
    exit 1
  fi

  if ! docker image inspect "${image_name}" >/dev/null 2>&1; then
    cat >&2 <<EOF
Docker image not found: ${image_name}

Build it locally:
  ./docker.sh build-image

Or load a shared image archive:
  docker load < agv-sim-humble_latest.tar.gz

Available images:
EOF
    docker images >&2
    exit 1
  fi
}

nvidia_runtime_available() {
  docker info 2>/dev/null | grep -Eiq 'Runtimes:.*nvidia|nvidia.*Runtimes:'
}

print_gpu_help() {
  cat <<EOF
NVIDIA GPU is visible on the host, but Docker cannot pass it into containers yet.

Install and configure NVIDIA Container Toolkit on the host:

  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \\
    sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

  curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \\
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \\
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

  sudo apt-get update
  sudo apt-get install -y nvidia-container-toolkit
  sudo nvidia-ctk runtime configure --runtime=docker
  sudo systemctl restart docker

Then verify:

  ./docker.sh gpu-check
EOF
}

gpu_check() {
  require_docker "$@"

  echo "[gpu-check] Host NVIDIA:"
  if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
  else
    echo "  nvidia-smi not found on host"
  fi

  echo "[gpu-check] Docker runtimes:"
  docker info 2>/dev/null | sed -n '/Runtimes:/,/Default Runtime/p'

  if ! nvidia_runtime_available; then
    echo
    print_gpu_help
    exit 1
  fi

  echo "[gpu-check] Testing --gpus all with the Humble image:"
  docker run --rm --gpus all "${IMAGE_NAME}" nvidia-smi
}

docker_args() {
  local mode="${1:-shell}"
  local display="${DISPLAY:-}"
  local xauthority="${XAUTHORITY:-${HOME}/.Xauthority}"
  local use_nvidia=0

  local args=(
    --rm
    --name "${CONTAINER_NAME}"
    --net host
    --ipc host
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
    -v "${PROJECT_DIR}:${CONTAINER_PROJECT_DIR}:rw"
    -w "${CONTAINER_WS_DIR}"
  )

  if nvidia_runtime_available; then
    use_nvidia=1
    args+=(
      --gpus all
      -e NVIDIA_VISIBLE_DEVICES="${NVIDIA_VISIBLE_DEVICES:-all}"
      -e NVIDIA_DRIVER_CAPABILITIES="${NVIDIA_DRIVER_CAPABILITIES:-graphics,utility,compute}"
      -e __NV_PRIME_RENDER_OFFLOAD=1
      -e __GLX_VENDOR_LIBRARY_NAME=nvidia
      -e __VK_LAYER_NV_optimus=NVIDIA_only
    )
  elif [ "${XIAMEN_AGV_FORCE_NVIDIA:-0}" = "1" ]; then
    args+=(--gpus all)
  fi

  if [ -d /dev/dri ]; then
    args+=(--device /dev/dri)
    if getent group video >/dev/null; then
      args+=(--group-add "$(getent group video | cut -d: -f3)")
    fi
    if getent group render >/dev/null; then
      args+=(--group-add "$(getent group render | cut -d: -f3)")
    fi
    args+=(-e LIBGL_DRI3_DISABLE="${LIBGL_DRI3_DISABLE:-0}")
    if [ "${use_nvidia}" -eq 0 ]; then
      args+=(-e MESA_LOADER_DRIVER_OVERRIDE="${MESA_LOADER_DRIVER_OVERRIDE:-radeonsi}")
    fi
  fi

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

container_id() {
  docker ps -aq --filter "name=^/${CONTAINER_NAME}$" | head -n 1
}

container_is_running() {
  local id="${1:-}"
  [ -n "${id}" ] && docker inspect -f '{{.State.Running}}' "${id}" 2>/dev/null | grep -qx true
}

prepare_container_name() {
  local id
  id="$(container_id)"
  if [ -z "${id}" ]; then
    return
  fi

  if container_is_running "${id}"; then
    return
  fi

  docker rm "${id}" >/dev/null
}

exec_in_running_container() {
  local user="${1:-agv}"
  shift || true

  local id
  id="$(container_id)"
  if [ -z "${id}" ]; then
    return 1
  fi

  if ! container_is_running "${id}"; then
    return 1
  fi

  echo "[docker.sh] Reusing running container: ${CONTAINER_NAME}"
  local exec_args=(exec)
  if [ -t 0 ]; then
    exec_args+=(-it)
  else
    exec_args+=(-i)
  fi
  docker "${exec_args[@]}" --user "${user}" -w "${CONTAINER_WS_DIR}" "${id}" "$@"
}

run_container() {
  local mode="${1:-shell}"
  local user="${2:-agv}"
  shift 2 || true

  prepare_container_name
  mapfile -t args < <(docker_args "${mode}")
  docker run "${args[@]}" --user "${user}" "${IMAGE_NAME}" "$@"
}

case "${1:-shell}" in
  build-image)
    require_docker_daemon "$@"
    build_args=(
      --target runtime \
      --build-arg BASE_IMAGE="${BASE_IMAGE}" \
      -t "${IMAGE_NAME}" \
      -f "${SCRIPT_DIR}/Dockerfile" \
    )

    for proxy_var in HTTP_PROXY HTTPS_PROXY NO_PROXY http_proxy https_proxy no_proxy; do
      if [ -n "${!proxy_var:-}" ]; then
        build_args+=(--build-arg "${proxy_var}=${!proxy_var}")
      fi
    done

    if [ -n "${HTTP_PROXY:-${http_proxy:-}}" ] || [ -n "${HTTPS_PROXY:-${https_proxy:-}}" ]; then
      # A loopback proxy belongs to the host, so the build must share its network.
      build_args+=(--network host)
    fi

    docker build "${build_args[@]}" "${SCRIPT_DIR}"
    ;;
  build-image-dev)
    require_docker_daemon "$@"
    build_args=(
      --target builder \
      --build-arg BASE_IMAGE="${BASE_IMAGE}" \
      -t "${DEV_IMAGE_NAME}" \
      -f "${SCRIPT_DIR}/Dockerfile" \
    )

    for proxy_var in HTTP_PROXY HTTPS_PROXY NO_PROXY http_proxy https_proxy no_proxy; do
      if [ -n "${!proxy_var:-}" ]; then
        build_args+=(--build-arg "${proxy_var}=${!proxy_var}")
      fi
    done

    if [ -n "${HTTP_PROXY:-${http_proxy:-}}" ] || [ -n "${HTTPS_PROXY:-${https_proxy:-}}" ]; then
      build_args+=(--network host)
    fi

    docker build "${build_args[@]}" "${SCRIPT_DIR}"
    ;;
  shell)
    require_docker "$@"
    exec_in_running_container agv /bin/bash && exit 0
    run_container shell agv /bin/bash
    ;;
  gui)
    require_docker "$@"
    exec_in_running_container agv /bin/bash && exit 0
    run_container gui agv /bin/bash
    ;;
  sim)
    shift || true
    require_docker sim "$@"
    exec_in_running_container agv /bin/bash -lc './sim.sh "$@"' bash "$@" && exit 0
    run_container sim agv /bin/bash -lc './sim.sh "$@"' bash "$@"
    ;;
  build)
    shift || true
    IMAGE_NAME="${DEV_IMAGE_NAME}"
    CONTAINER_NAME="${DEV_CONTAINER_NAME}"
    require_image "${IMAGE_NAME}"
    exec_in_running_container agv /bin/bash -lc \
      'source /opt/ros/humble/setup.bash && colcon build --symlink-install "$@"' \
      bash "$@" && exit 0
    run_container shell agv /bin/bash -lc \
      'source /opt/ros/humble/setup.bash && colcon build --symlink-install "$@"' \
      bash "$@"
    ;;
  gpu-check)
    gpu_check "$@"
    ;;
  root)
    require_docker "$@"
    run_container shell root /bin/bash
    ;;
  clean)
    docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
    docker rm -f "${DEV_CONTAINER_NAME}" >/dev/null 2>&1 || true
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
