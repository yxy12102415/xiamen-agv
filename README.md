# Xiamen AGV Simulation

ROS 2 AGV simulation workspace for the Xiamen port scene. The repository keeps
two ROS 2 workspaces:

- `humble/`: ROS 2 Humble workspace.
- `galatic/`: ROS 2 Galactic-compatible workspace, including Docker helper
  scripts for machines that do not have Galactic installed locally.

The project currently supports Gazebo/RViz simulation, lidar point clouds,
Lanelet2-style map visualization, trajectory tracking experiments, yard crane
motion, container-yard scene objects, and a real-vehicle interface skeleton for
future bench and shadow-mode testing.

## Quick Start With Docker

The Galactic workspace expects a Docker image named
`agv-sim-galactic:latest`. If you receive the image as a compressed archive,
load it first:

```bash
docker load < agv-sim-galactic_latest.tar.gz
cd xiamen-agv/galatic
./docker.sh gpu-check
./docker.sh build
./docker.sh sim control
```

If a previous container is still using the same name:

```bash
./docker.sh clean
./docker.sh sim control
```

GPU passthrough is recommended. Without it, Gazebo, RViz, and lidar point-cloud
display can be much slower.

## 中文快速说明

如果别人电脑上没有 ROS 2 Galactic，推荐使用 Docker。先加载镜像压缩包：

```bash
docker load < agv-sim-galactic_latest.tar.gz
cd xiamen-agv/galatic
./docker.sh gpu-check
./docker.sh build
./docker.sh sim control
```

如果提示容器名冲突：

```bash
./docker.sh clean
./docker.sh sim control
```

建议配置 NVIDIA Docker GPU 支持，否则 Gazebo、RViz 和激光点云显示会比较卡。

More details:

- [Galactic README](galatic/README.md)
- [Humble README](humble/README.md)
