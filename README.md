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

## Current Progress / 当前进度

The Humble workspace is the primary development target and is currently at the
**rich simulation / pre-real-vehicle integration** stage. It includes the full
port scene, six lidar feeds, six 120-degree cameras, an eight-wheel vehicle
model with steering axles 1/2 and per-wheel Ackermann tire forces, estimated
empty/loaded dynamics profiles, MPC tracking, chassis-like feedback, safety
supervision, and a six-camera remote-driving loopback.

The remote-driving path uses H.264 over UDP with a configurable delay, jitter,
loss, and bandwidth model. With the default `remote` sensor profile on the
development RTX 3070 system, all six decoded feeds have been verified at about
15 FPS with a configured 50 ms one-way delay. The Galactic workspace contains
the compatible simulation subset and Docker helpers; new remote-driving work is
currently Humble-only.

The repository is not a production real-vehicle package yet. Vendor CAN DBC or
Ethernet ICD integration, measured vehicle/load parameters, production sensor
calibration and timing, hardware E-stop/mode/watchdog validation, and real
5G/video-link integration remain outstanding.

### One-to-One Vehicle Replication / 一比一复刻状态

This is a functional digital prototype, not a one-to-one validated real-vehicle
replica. The known vehicle dimensions, mass, axle locations, eight-wheel layout,
first-two-axle steering, and sensor layout have been modeled. Vehicle/load
dynamics, tire and actuator behavior, sensor calibration, chassis protocol, and
remote network behavior still contain engineering assumptions or simulated data.
Real calibration files, vendor protocol definitions, and closed-course empty and
loaded test logs are required before one-to-one correlation can be claimed.

当前是可运行的数字样机，不是一比一验证后的实车复刻。已按已有资料实现车辆尺寸、
质量、轴位、八轮结构、前两轴转向和传感器布局；但车辆/负载动力学、轮胎和执行器
特性、传感器标定、底盘协议与远驾网络中仍有工程估计或仿真数据。拿到实车标定文件、
厂家协议和空载/带载闭场测试记录后，才能进行一比一参数对齐和回归验证。

Humble 是当前主要开发版本，项目处于**丰富仿真 / 实车预集成**阶段。现已包含完整
港区场景、六路激光雷达、六路 120 度摄像头、前两轴转向的八轮逐轮 Ackermann
轮胎力模型、空载和负载估计参数、MPC 跟踪、底盘反馈仿真、安全监督及六路远程
驾驶回环。

远驾链路采用 H.264/UDP，并可配置时延、抖动、丢包和带宽。开发机 RTX 3070 上，
默认 `remote` 传感器配置已验证六路解码画面约 15 FPS，配置的单向网络时延为
50 ms。Galactic 工作空间保留兼容仿真功能和 Docker 启动工具；新增远驾功能目前
只在 Humble 中维护。

当前仍不能直接作为实车量产部署包使用，尚需完成厂家 CAN DBC / 以太网 ICD、
实测车辆及负载参数、正式传感器标定与时间同步、硬件急停/模式/看门狗验证，以及
真实 5G 和视频链路接入。

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
