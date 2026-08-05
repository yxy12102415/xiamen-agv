FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    curl \
    ffmpeg \
    git \
    libeigen3-dev \
    libignition-gazebo6-dev \
    libignition-msgs8-dev \
    libignition-transport11-dev \
    mesa-utils \
    python3-colcon-common-extensions \
    python3-matplotlib \
    python3-opencv \
    python3-pil \
    python3-pil.imagetk \
    python3-pip \
    python3-rosdep \
    python3-tk \
    ros-humble-cv-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ros-gz \
    ros-humble-tf2-ros \
    sudo \
    x11-apps \
  && rm -rf /var/lib/apt/lists/*

ARG USERNAME=agv
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
  && useradd --uid "${USER_UID}" --gid "${USER_GID}" -m "${USERNAME}" -s /bin/bash \
  && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/90-agv \
  && chmod 0440 /etc/sudoers.d/90-agv

USER ${USERNAME}
WORKDIR /workspace/xiamen-agv-sim/humble

RUN echo 'source /opt/ros/humble/setup.bash' >>/home/${USERNAME}/.bashrc

CMD ["/bin/bash"]
