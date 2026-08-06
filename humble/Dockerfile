ARG BASE_IMAGE=osrf/ros:humble-simulation

FROM ${BASE_IMAGE} AS builder

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN printf 'Acquire::http::Proxy "false";\nAcquire::https::Proxy "false";\n' >/etc/apt/apt.conf.d/99no-proxy

RUN apt-get -o Acquire::Retries=5 -o Acquire::http::Timeout=60 update && apt-get -o Acquire::Retries=5 -o Acquire::http::Timeout=60 install -y --no-install-recommends --fix-missing \
    build-essential \
    cmake \
    curl \
    git \
    libeigen3-dev \
    libignition-gazebo6-dev \
    libignition-msgs8-dev \
    libignition-transport11-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-humble-ros-gz-interfaces \
    ros-humble-tf2-ros \
    sudo \
  && rm -rf /var/lib/apt/lists/*

ARG SOURCE_DIR=/tmp/xiamen-agv-sim/humble
ARG BUILD_DIR=/tmp/xiamen-agv-build/build
ARG INSTALL_DIR=/opt/xiamen-agv-sim/install

COPY . "${SOURCE_DIR}"

WORKDIR "${SOURCE_DIR}"

RUN source /opt/ros/humble/setup.bash && \
    colcon build --merge-install \
      --build-base "${BUILD_DIR}" \
      --install-base "${INSTALL_DIR}"

FROM ${BASE_IMAGE} AS runtime

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN printf 'Acquire::http::Proxy "false";\nAcquire::https::Proxy "false";\n' >/etc/apt/apt.conf.d/99no-proxy

RUN apt-get update && apt-get install -y --no-install-recommends --fix-missing \
    ffmpeg \
    libignition-gazebo6 \
    libignition-gazebo6-plugins \
    libignition-msgs8 \
    libignition-transport11 \
    python3-opencv \
    python3-pil \
    python3-pil.imagetk \
    python3-tk \
    ros-humble-cv-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-interfaces \
    ros-humble-ros-gz-sim \
    ros-humble-rviz2 \
    ros-humble-tf2-ros \
  && rm -rf /var/lib/apt/lists/*

ARG USERNAME=agv
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
  && useradd --uid "${USER_UID}" --gid "${USER_GID}" -m "${USERNAME}" -s /bin/bash \
  && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/90-agv \
  && chmod 0440 /etc/sudoers.d/90-agv

COPY --from=builder /opt/xiamen-agv-sim/install /opt/xiamen-agv-sim/install

RUN mkdir -p /workspace/xiamen-agv-sim \
  && chown -R "${USERNAME}:${USERNAME}" /opt/xiamen-agv-sim

USER ${USERNAME}
WORKDIR /workspace/xiamen-agv-sim/humble

RUN printf '%s\n' \
  'source /opt/ros/humble/setup.bash' \
  'if [ -f /opt/xiamen-agv-sim/install/setup.bash ]; then' \
  '  source /opt/xiamen-agv-sim/install/setup.bash' \
  'fi' >>/home/${USERNAME}/.bashrc

CMD ["/bin/bash"]
