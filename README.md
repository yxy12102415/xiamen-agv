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

这是厦门港 AGV 仿真项目，仓库中包含两个 ROS 2 工作空间：

- `humble/`：ROS 2 Humble 工作空间。
- `galatic/`：兼容 ROS 2 Galactic 的工作空间，并提供 Docker 辅助脚本，方便
  没有本机 Galactic 环境的电脑运行。

当前项目支持 Gazebo/RViz 仿真、激光点云、Lanelet2 风格地图可视化、轨迹跟踪实
验、堆场吊运动、集装箱堆场场景，以及用于后续台架测试和影子模式测试的实车接口
骨架。

## Quick Start With Docker / Docker 快速启动

The Galactic workspace expects a Docker image named
`agv-sim-galactic:latest`. If you receive the image as a compressed archive,
load it first:

Galactic 工作空间默认需要一个名为 `agv-sim-galactic:latest` 的 Docker 镜像。
如果你收到的是镜像压缩包，先加载镜像：

```bash
docker load < agv-sim-galactic_latest.tar.gz
cd xiamen-agv/galatic
./docker.sh gpu-check
./docker.sh build
./docker.sh sim control
```

If a previous container is still using the same name:

如果提示容器名冲突，先清理旧容器：

```bash
./docker.sh clean
./docker.sh sim control
```

GPU passthrough is recommended. Without it, Gazebo, RViz, and lidar point-cloud
display can be much slower.

建议配置 NVIDIA Docker GPU 支持，否则 Gazebo、RViz 和激光点云显示会比较卡。

## More Details / 更多说明

- [Galactic README](galatic/README.md)
- [Humble README](humble/README.md)
