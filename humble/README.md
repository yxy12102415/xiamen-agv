# AGV Simulation Workspace

ROS 2 Humble workspace for the Xiamen AGV simulation. It contains the Gazebo
world, Lanelet2 map, RViz visualization helpers, a two-axis AGV vehicle model,
and launch files for running either standalone simulation or Autoware planning
and control against the same Gazebo scene.

## Contents

- `src/agv_gazebo`: Gazebo world, AGV model with four lidars, dynamic obstacles,
  Gazebo/ROS bridge, and RViz launch support.
- `src/agv_vehicle_model`: two-axis steering simulator node and keyboard teleop.
- `src/agv_map_visualizer`: Lanelet2, vehicle, and sampled lidar marker
  publishers for RViz.
- `src/agv_mpc_controller`: centerline trajectory publisher and lightweight
  sampling MPC tracker for the standalone simulation.
- `src/agv_safety_supervisor`: command safety gate for velocity, acceleration,
  braking, steering, and command timeout limits.
- `src/agv_vehicle_interface`: reference vehicle-interface bridge / shadow-mode
  node for moving toward real vehicle integration.
- `src/autoware_*_msgs`: local Autoware-style message definitions used by the
  simulator interface. This workspace is Autoware-style, not a full Autoware
  checkout.
- `src/agv_maps/map/xiamen-prod.osm`: Xiamen Lanelet2-style vector map.
- `src/agv_bringup`: Autoware + Gazebo integration launch files and route helper.

## 中文说明

这是厦门港 AGV 仿真的 ROS 2 Humble 工作空间，包含 Gazebo 场景、Lanelet2 风格
地图、RViz 可视化、双轴转向 AGV 车辆模型，以及独立仿真和 Autoware 联合仿真的
启动文件。

主要目录：

- `src/agv_gazebo`：Gazebo 世界、AGV 模型、激光雷达、动态障碍物、Gazebo/ROS
  桥接和 RViz 启动配置。
- `src/agv_vehicle_model`：双轴转向车辆仿真节点和键盘控制。
- `src/agv_map_visualizer`：Lanelet2 地图、车辆、采样点云和标记可视化。
- `src/agv_mpc_controller`：轨迹中心线发布和轻量采样 MPC 跟踪控制。
- `src/agv_safety_supervisor`：速度、加速度、制动、转向和指令超时安全限制。
- `src/agv_vehicle_interface`：面向实车联调的接口骨架和影子模式节点。
- `src/agv_bringup`：Autoware + Gazebo 集成启动文件和路线辅助工具。

当前进度：

- 已完成厦门港 Gazebo 场景、AGV 模型、六路激光点云桥接、RViz 可视化、独立轨迹
  跟踪、双轴车辆仿真、安全监督器、堆场吊和集装箱场景对象。
- 项目仍处于“丰富仿真 / 实车预集成”阶段，不能直接部署到真实 AGV。
- 实车部署前还需要完成底盘 CAN/以太网协议、急停和看门狗、传感器外参、定位地图
  原点、低速封闭场地测试，以及空载/负载车辆动力学参数标定。
- 已保持前两轴转向、后两轴固定；轴中心转角会转换为左右轮 Ackermann 转角，
  8 个轮位分别参与轮胎力计算，MPC 使用相同的八轮运动学几何。

常用命令：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
./sim.sh
./autosim.sh
```

`./sim.sh` 启动独立 Gazebo/RViz 仿真；`./autosim.sh` 在此基础上加载 Autoware
规划、控制、API 和系统模块。不要同时运行两个仿真入口，除非使用不同的
ROS/Gazebo domain 和 world。

## 远程驾驶网络仿真

远驾模式已经加入应用层网络仿真。控制指令、模拟 B&R 状态反馈和六路摄像头默认
都使用 `50 ms` 单向延迟，抖动和丢包默认为零。远驾控制台显示 UDP 链路状态、
反馈时延，以及每路相机的接收/显示帧率和估算画面年龄。

```bash
./sim.sh remote

# 自定义单向延迟、抖动和丢包率
./sim.sh remote \
  remote_network_delay_ms:=100.0 \
  remote_network_jitter_ms:=20.0 \
  remote_network_loss_percent:=2.0
```

视频默认由六个 Gazebo Transport/C++ 发送节点直接读取相机图像，使用五路 NVENC
和一路 `libx264` 编码，再由一个 UDP 网络损伤代理和六个 C++/FFmpeg 接收节点完成
Annex-B/UDP 传输与解码；每路默认 `1.5 Mbit/s`，总链路
上限 `100 Mbit/s`。当前显卡最多并发五路 NVENC，因此右后相机固定使用 `libx264`；
若 NVENC 启动失败，发送节点会自动切换到 `libx264`，避免相机画面直接中断。
前视视频冻结约 `1 s` 或
接收端健康诊断超过 `1.5 s` 时，REMOTE 仲裁器自动输出停车指令。正式
RTP/SRT/WebRTC、5G 切片和鉴权仍未接入。完整状态见
[远程驾驶 UDP 协议状态](docs/remote_driving_udp_protocol_status.md)。

`./sim.sh remote` 默认采用 `remote` 传感器配置：前视保持 `960x540`，其余相机
以 `640x360` 生成，四个 H16 的水平采样由 `1800` 降至 `900`；相机/LiDAR 频率、
LiDAR 垂直线数、量程、噪声、视场角、安装外参和场景模型均不改变。运行时只修改
`/tmp` 中的模型副本，源码中的完整传感器参数保持不变。

```bash
# 默认均衡档
./sim.sh remote

# 原始传感器参数，用于精度对照
AGV_REMOTE_SENSOR_PROFILE=full ./sim.sh remote

# 帧率优先：前视 800x450，其余 480x270，LiDAR 水平采样上限 600
AGV_REMOTE_SENSOR_PROFILE=fast ./sim.sh remote
```

## Remote-Driving Network Simulation

Remote mode now applies an application-level network model to command UDP,
mock B&R feedback UDP, and all six camera feeds. The default is `50 ms` one-way
delay, zero jitter, and zero packet loss. The console reports link state,
feedback latency, camera receive/render FPS, and estimated image age.

Each camera is read directly from Gazebo Transport by a C++ H.264 sender,
followed by one UDP impairment proxy and six C++/FFmpeg receivers. The default
path uses H.264 Annex-B over local UDP at `1.5 Mbit/s` per camera, with a
`100 Mbit/s` aggregate cap. Five feeds use NVENC and the right-rear feed uses
`libx264`; a failed NVENC process automatically falls back to `libx264`. A front
video freeze or receiver-health timeout stops REMOTE commands. RTP/SRT/WebRTC,
5G slicing, authentication, and encryption remain future integration work.

Remote mode defaults to the `remote` sensor profile. Use
`AGV_REMOTE_SENSOR_PROFILE=full` for original sensor fidelity or
`AGV_REMOTE_SENSOR_PROFILE=fast` for the highest frame-rate profile.

RViz 使用 `map` 作为固定坐标系。仿真器发布 `map -> base_link`，启动文件发布
从 `base_link` 到激光雷达、IMU、GNSS 等传感器坐标系的静态 TF。若点云位置漂移
或闪烁，优先检查 Gazebo SDF、启动参数和实车标定外参是否一致。

## 负载动力学工况

启动时可选择空载或三个负载工况：

    ./sim.sh control vehicle_load_profile:=payload_20t
    ./sim.sh control vehicle_load_profile:=payload_30_48t
    ./sim.sh control vehicle_load_profile:=rated_70t

四档分别为 0、20、30.48 和 70 吨载荷。质量、三维质心、偏航惯量、轴荷、
轮胎侧偏刚度、轮胎半径和牵引力限制会同步切换；MPC 预测同步使用相同的执行器
死区与转向回差。数值均为工程估计，不是实车标定结果，推导和待替换数据见
[动力学估计说明](docs/estimated_vehicle_dynamics.md)。

## Estimated Load Profiles

Use vehicle_load_profile:=empty|payload_20t|payload_30_48t|rated_70t with
sim.sh. The selected profile updates mass, 3D center of gravity, yaw inertia,
axle loads, cornering stiffness, loaded wheel radius and tractive-force limits.
These are engineering estimates, not measured vehicle data; see
[estimated vehicle dynamics](docs/estimated_vehicle_dynamics.md).

## Current Progress

The project is currently at a **rich simulation / pre-real-vehicle integration**
stage. It is useful for Gazebo/RViz scenario validation, perception topic
plumbing, trajectory tracking experiments, and control-interface dry runs, but it
is **not yet a complete real-vehicle deployment package**.

Implemented and working:

- Xiamen port Gazebo scene with Lanelet2-style map visualization.
- AGV visual/collision model with lidar, camera, GNSS, and IMU frames.
- Six lidar point clouds bridged into ROS and visualized in RViz.
- Standalone port-loop trajectory generation and sampling MPC tracking.
- Two-axis AGV vehicle simulator with Autoware-style control/status topics.
- Safety supervisor for command timeout, velocity, acceleration, braking, and
  steering limits.
- Dynamic scene objects:
  - one straight-moving traffic vehicle,
  - eight yard cranes aligned to the left-side lane groups,
  - yard cranes moving along x from 0 m to 280 m at 1.2 m/s,
  - three rows of container piles aligned to yard cranes,
  - hanging containers under quay cranes on the controller trajectory lane.
- Wheel TF, the dynamic tire model, and MPC prediction apply the same Ackermann
  inner/outer geometry to eight wheel stations; only axles 1 and 2 steer.
- Six-camera remote-driving loopback with 40 Hz command/feedback UDP, H.264
  video transport, configurable delay/jitter/loss/bandwidth, video-stale safety
  interlock, and one-window WASD control.
- Selectable `full`, `remote`, and `fast` runtime sensor profiles. The default
  `remote` profile has been verified at about 15 FPS on all six decoded feeds
  with an RTX 3070 and a configured 50 ms one-way delay.

Recent scene layout:

- Yard cranes `A01..A08` are aligned at y:
  `80, 110, 140, 168, 198, 226, 256, 286`.
- Yard crane initial x positions are intentionally shuffled:
  `248, 36, 181, 96, 267, 14, 223, 132`.
- Container piles use `container_block_3x6x2`, with 3 containers in x, 6 in y,
  and 2 layers in z.
- Container pile rows are at y `80, 110, 140`; x positions are
  `0, 45, 90, 135, 180, 225`.
- Quay hanging containers are at x `52, 148, 244, 340`, y `10.0013`, z `10.0`,
  with the long side along x.

### One-to-One Vehicle Replication / 一比一复刻状态

The simulator is a functional digital prototype, not a one-to-one validated
real-vehicle replica. Known dimensions, mass, axle locations, eight-wheel
layout, steering-axle selection, and sensor layout are represented. The
vehicle/load dynamics, tire and actuator characteristics, sensor calibration,
chassis protocol, and remote network still include engineering assumptions or
simulated data. Real calibration files, vendor protocol definitions, and empty
and loaded closed-course logs are required for one-to-one correlation.

当前仿真是可运行的数字样机，不是一比一验证后的实车复刻。已按已知资料实现车辆
尺寸、质量、轴位、八轮结构、前两轴转向及传感器布局；车辆/负载动力学、轮胎和
执行器特性、传感器标定、底盘协议与远驾网络仍包含工程估计或仿真数据。只有拿到
实车标定文件、厂家协议以及空载/带载闭场测试记录后，才能完成一比一参数对齐与回归
验证。

## Real Vehicle Readiness

This workspace is **not ready to deploy directly to a real AGV yet**. The main
remaining work is calibration, interface validation, and safety verification:

- Replace estimated vehicle dynamics parameters with measured real-vehicle data.
- Validate CAN / chassis protocol mapping against the real controller.
- Add hardware E-stop, remote/local mode, and watchdog tests around the safety
  supervisor.
- Verify GNSS/IMU/lidar/camera extrinsics against real calibration.
- Validate localization frame conventions and map origin against the production
  map.
- Run low-speed closed-course tests before enabling higher-speed MPC tracking.
- Add loaded-vehicle parameters or adaptive/gain-scheduled MPC before carrying
  containers on the real vehicle.
- Replace the local UDP/video loopback with the production authenticated
  5G/video transport and validate its end-to-end delay and fail-safe behavior.

### Real Vehicle Interface Status

English:

This workspace already contains a **real-vehicle interface skeleton**, but it is
not yet a validated production chassis interface.

- Interface node:
  `src/agv_vehicle_interface/agv_vehicle_interface/real_vehicle_interface_node.py`
- Real bringup launch:
  `src/agv_bringup/launch/real_bringup.launch.py`
- Supported transport modes:
  `dry_run`, `socketcan`, and `udp`
- Default safety state:
  `dry_run=true`, `shadow_mode=true`, `autonomous_enable=false`

The current interface is intended for bench testing, shadow-mode testing, and
protocol integration. Before real driving, the vendor chassis DBC / Ethernet ICD
must be integrated and verified, including CAN IDs, byte layout, scaling,
checksums, heartbeat, fault bits, E-stop, braking priority, mode switching, and
all feedback decoding.

In simulation, `simulated_chassis_feedback_node` publishes real-chassis-like
feedback under `/agv/chassis/*`, including actual velocity, acceleration, front
and rear steering, wheel speeds, brake pressure, drive/steering motor states,
gear, control mode, E-stop, fault, communication state, and a timestamped JSON
summary.

Deployment references:

- [Real vehicle deployment checklist](docs/real_vehicle_deployment_checklist.md)
- [Required chassis CAN / Ethernet signals](docs/can_interface_required_signals.md)
- [Sensor, time, and calibration guide](docs/sensor_time_calibration_guide.md)
- [Reference chassis protocol template](docs/reference_chassis_protocol.md)
- [Reference AGV DBC template](docs/reference_agv_chassis.dbc)
- [Remote-driving UDP protocol status and missing data](docs/remote_driving_udp_protocol_status.md)

中文：

当前项目里**已经有实车接口骨架**，但还不是已经完成验证的正式实车底盘接口。

- 接口节点：
  `src/agv_vehicle_interface/agv_vehicle_interface/real_vehicle_interface_node.py`
- 实车启动文件：
  `src/agv_bringup/launch/real_bringup.launch.py`
- 支持的通信模式：
  `dry_run`、`socketcan`、`udp`
- 默认安全状态：
  `dry_run=true`，`shadow_mode=true`，`autonomous_enable=false`

也就是说，默认不会真正驱动车辆。目前它适合做台架测试、影子模式测试和协议联调。
真正上车前，还需要接入并验证厂家正式底盘 DBC / 以太网 ICD，包括 CAN ID、字节
定义、缩放比例、校验、心跳、故障位、急停、制动优先级、模式切换和全部反馈解析。

在仿真中，`simulated_chassis_feedback_node` 会在 `/agv/chassis/*` 下发布接近
真实底盘的反馈，包括实际车速、实际加速度、前后轴转角、轮速、制动压力、驱动/
转向电机状态、档位、控制模式、急停、故障、通信状态，以及带时间戳的 JSON 汇总。

部署参考文档：

- [实车部署检查清单](docs/real_vehicle_deployment_checklist.md)
- [底盘 CAN / 以太网信号需求](docs/can_interface_required_signals.md)
- [传感器、时间同步和标定说明](docs/sensor_time_calibration_guide.md)
- [参考底盘协议模板](docs/reference_chassis_protocol.md)
- [参考 AGV DBC 模板](docs/reference_agv_chassis.dbc)

For load handling, start with discrete parameter sets such as:

```text
empty
loaded_light
loaded_heavy
```

Each set should update mass, yaw inertia, axle loads, tire cornering stiffness,
drive/brake response, and MPC prediction parameters. A fully adaptive MPC can be
added later after the simpler parameter-switching model is validated.

## Vehicle Model Parameter Status

Some parameters are from the supplied vehicle sheet; others are estimated or
back-calculated because measured values are not available yet.

Currently matched / configured:

- Vehicle length and width: `15.0 m x 2.98 m`.
- Axle x positions relative to the center of gravity:
  `+4.65, +3.00, -3.00, -4.65 m`.
- Mass: `23000 kg`.
- Wheel rolling radius: `0.66 m`.
- Steering rate limit: `0.261799 rad/s` = `15 deg/s`.
- Max drive acceleration: `0.35 m/s^2`.
- Max brake deceleration: `4.0 m/s^2`, aligned with the line-control braking
  requirement rather than the older table value of `0.35 m/s^2`.
- Ackermann wheel angles used by visualization, dynamics, and MPC:
  - axle 1: `27.4 deg` outer gives approximately `33.0 deg` inner,
  - axle 2: `22.6 deg` outer gives approximately `30.0 deg` inner.

Estimated / not measured:

- `yaw_inertia_kgm2 = 432363.7513`.
- Tire cornering stiffness values.
- Rolling resistance, aerodynamic drag, frontal area, and wheel inertia.
- Wheel tread, total height, center-of-gravity height, and wheel width still
  need formal measured values.

The controller still exposes two axle-center equivalent steering states. They
are converted to left/right wheel angles before the eight-wheel kinematic and
tire-force calculations. The equivalent Ackermann lengths are back-calculated,
not measured steering-linkage dimensions.

## Build

```bash
source /opt/ros/humble/setup.bash
cd /data/projects/AGV_sim_humble
colcon build --symlink-install
source install/setup.bash
```

The workspace is normally built with `--symlink-install`, so launch/config edits
under `src/` are reflected through `install/` without rebuilding. Rebuild after
changing C++ code.

## Run Standalone Gazebo

```bash
./sim.sh
```

This sources ROS Humble and this workspace, then launches:

```bash
ros2 launch agv_gazebo agv_gazebo_sim.launch.py
```

It starts Gazebo with the Xiamen lane scene, the AGV two-axis simulator, Gazebo
pose follower, standalone map visualization, RViz, and visualization marker
nodes. Control and trajectory nodes are disabled by the default `sim.sh` mode.

Useful launch arguments:

```bash
./sim.sh use_rviz:=false
./sim.sh use_gazebo_gui:=false
./sim.sh enable_mpc:=false
./sim.sh enable_camera_topics:=false
./sim.sh initial_x:=9.9991 initial_y:=28.0013 initial_yaw:=-1.5708
./sim.sh steering_mode:=crab
```

MPC tracking errors are written by default to:

```text
/tmp/agv_mpc_error.csv
```

The main standalone control chain is:

```text
centerline_trajectory_publisher
-> sampling_mpc_controller
-> /control/command/control_cmd
-> agv_two_axis_simulator_node
-> /localization/kinematic_state
```

## Run With Autoware

```bash
./autosim.sh
```

This additionally sources `/data/projects/autoware/install/setup.bash` and
launches:

```bash
ros2 launch agv_bringup autoware_gazebo_sim.launch.py
```

`autosim.sh` includes the same Gazebo launch used by `sim.sh`, then starts
Autoware planning, control, API, and system modules. Autoware vehicle and vehicle
interface launches are disabled because the local `agv_two_axis_simulator_node`
provides the simulated vehicle interface.

## Control Interfaces

The standalone simulator and Autoware launch are aligned on the same control
topics:

- `/control/command/control_cmd`: `autoware_control_msgs/msg/Control`
- `/control/command/gear_cmd`: `autoware_vehicle_msgs/msg/GearCommand`
- `/control/command/turn_indicators_cmd`:
  `autoware_vehicle_msgs/msg/TurnIndicatorsCommand`
- `/control/command/hazard_lights_cmd`:
  `autoware_vehicle_msgs/msg/HazardLightsCommand`
- `/vehicle/status/control_mode`: `autoware_vehicle_msgs/msg/ControlModeReport`
- `/control/control_mode_request`: `autoware_vehicle_msgs/srv/ControlModeCommand`
- `/api/operation_mode/state` and `/system/operation_mode/state`:
  `autoware_adapi_v1_msgs/msg/OperationModeState`

The simulator also accepts legacy `/vehicle/engage`
(`autoware_vehicle_msgs/msg/Engage`), but Autoware mode changes primarily use
the AD API operation mode services.

## Scene

The default Xiamen world contains both static map geometry and dynamic obstacles:

- `xiamen_site`: simplified lane surface and markings generated from
  `src/agv_maps/map/xiamen-prod.osm`
- `quay_crane_sts_1` through `quay_crane_sts_4`: STS quay crane mesh models
- `ego_agv`: AGV model sized from `3D模型合集/AGV/agv.glb`; Gazebo uses split
  STL visuals for body/deck materials plus clean wheel-group geometry extracted
  from the AGV model, while RViz uses the AGV DAE mesh.
- `yard_crane_A01` through `yard_crane_A08`: moving yard cranes, driven by
  `dynamic_obstacle_simulator_node`.
- `yard_container_pile_01` through `yard_container_pile_18`: container stacks in
  the yard area.
- `quay_hanging_container_01` through `quay_hanging_container_04`: suspended
  containers under quay cranes.

If the AGV flickers or jumps, first check for stale Gazebo or duplicate
simulation processes:

```bash
ps -ef | rg "ros2 launch|ign gazebo|gz sim|parameter_bridge|dynamic_obstacle|rviz2"
```

If an old `ign gazebo server` remains after closing a launch, stop it before
starting a new simulation:

```bash
kill <pid>
```

Avoid running `sim.sh` and `autosim.sh` at the same time unless they use separate
ROS/Gazebo domains and worlds.

## Lidar And RViz Notes

RViz uses `map` as the fixed frame. The AGV simulator publishes `map ->
base_link`, and the launch files publish static transforms from `base_link` to
the sensor frames.

GNSS and IMU topics:

- `/chnav/fix`: `sensor_msgs/msg/NavSatFix`, frame `gnss_link`
- `/chnav/imu/data`: `sensor_msgs/msg/Imu`, frame `imu_link`

GNSS and IMU frames:

- `imu_link`
- `gnss_link`

Lidar topics and frames:

- Raw Gazebo bridge topics:
  `/sensing/lidar/front_left/points`, `/sensing/lidar/front_right/points`,
  `/sensing/lidar/rear_left/points`, and `/sensing/lidar/rear_right/points`
- `/rslidar_points_2`: right-front Helios-32, frame `rslidar2`
- `/rslidar_points_4`: left-rear Helios-32, frame `rslidar4`
- `/hesai_left_front`: left-front Pandar32, frame `hesai_left_front`
- `/hesai_right_rear`: right-rear Pandar32, frame `hesai_right_rear`

The IMU/GNSS extrinsics are launch arguments such as `imu_x`, `imu_y`, `imu_z`,
`gnss_x`, `gnss_y`, and `gnss_z`. These static transforms must match the sensor
poses in the Gazebo SDF model or real calibration. A height or frame mismatch can
make lidar markers appear offset or flickery in RViz.

The simulator also publishes dynamic wheel frames under `base_link`:
`front_left_wheel`, `front_right_wheel`, `rear_left_wheel`, and
`rear_right_wheel`. Their positions follow the Xiamen AGV mesh dimensions, while
their yaw follows the simulated front and rear steering angles.

Lidar extrinsics are launch arguments too, using each frame prefix:
`rslidar2_*`, `rslidar4_*`, `hesai_left_front_*`, and `hesai_right_rear_*`,
where `*` is `x`, `y`, `z`, `yaw`, `pitch`, or `roll`.

## Remote Driving Video Input

The Humble simulation includes six 120-degree camera inputs for remote-driving
experiments. Each camera publishes an image and camera info topic:

- Front: `/sensing/camera/front/image`, frame `front_camera`.
- Rear: `/sensing/camera/rear/image`, frame `rear_camera`.
- Left front side: `/sensing/camera/left_front/image`, frame
  `left_front_camera`.
- Left rear side: `/sensing/camera/left_rear/image`, frame `left_rear_camera`.
- Right front side: `/sensing/camera/right_front/image`, frame
  `right_front_camera`.
- Right rear side: `/sensing/camera/right_rear/image`, frame
  `right_rear_camera`.

All six simulated cameras use:

- Mount height: `1.95 m`.
- Horizontal field of view: `120 deg`.
- Resolution: `960 x 540`.
- Update rate: `15 Hz`.

For a remote-driving style combined view, run:

```bash
ros2 run agv_gazebo remote_drive_camera_viewer
```

The viewer shows the front camera as the large main image, with the remaining
five cameras as smaller tiles on the right side. Each tile is labeled and shows
whether the stream is live, waiting, or stale.

You can still check a single stream with:

```bash
ros2 run rqt_image_view rqt_image_view
```

or inspect the topics:

```bash
ros2 topic list | rg "/sensing/camera"
ros2 topic hz /sensing/camera/front/image
```

The launch argument `enable_camera_topics` can disable only the camera bridges
while leaving lidar topics unchanged:

```bash
./sim.sh enable_camera_topics:=false
```

中文：

Humble 仿真已经加入 6 路 120 度摄像头输入，可用于远程驾驶仿真。车头、车尾各
一个摄像头，车身左右两侧各两个摄像头。每路都有 `image` 和 `camera_info` 话题。

当前参数为：

- 安装高度：`1.95 m`。
- 水平视场角：`120 deg`。
- 分辨率：`960 x 540`。
- 刷新率：`15 Hz`。

查看视频推荐使用远程驾驶拼接窗口：

```bash
ros2 run agv_gazebo remote_drive_camera_viewer
```

这个窗口会把前视摄像头作为最大主画面，其他五路摄像头以小窗口显示在右侧，并标出
每路画面是否在线。

单独查看某一路视频也可以使用：

```bash
ros2 run rqt_image_view rqt_image_view
```

### Remote driving control / 远程驾驶控制

The Humble control path includes an AUTO/REMOTE command arbiter. The remote
operator now passes through a 40 Hz UDP/TLV domain gateway and a simulated B&R
vehicle controller before reaching the REMOTE input. Only the selected source
reaches `agv_safety_supervisor`. The default authority is `AUTO`. Authority
changes are accepted only while vehicle speed is at or below `0.05 m/s`.

```text
WASD -> /remote/operator/* -> UDP/TLV gateway -> mock B&R
     -> /remote/control/* -> command mux -> safety supervisor -> vehicle
```

Start the complete remote-driving session with one command:

```bash
cd /data/projects/xiamen-agv-sim/humble
./sim.sh remote
```

This one command starts the Gazebo server, RViz, the command arbiter, and one
GTA-style WASD console containing all six camera views. Gazebo GUI remains off
by default; set `AGV_REMOTE_USE_GAZEBO_GUI=true` to display it. RViz can be
disabled for maximum sensor-rendering performance with
`AGV_REMOTE_USE_RVIZ=false`. Closing the control window or pressing `Ctrl+C`
stops the whole session. The old separate camera window is available only when
explicitly requested with
`AGV_REMOTE_CAMERA_VIEWER=1 ./sim.sh remote`.

If a terminal or GUI is force-closed, stop every process from this simulation
with `./sim.sh stop` before starting it again.

WASD control-window keys:

- `W`: increase target speed by `0.10 m/s` per key press; the speed remains set
  after release.
- `S`: decrease target speed by `0.10 m/s` per key press; further presses after
  zero select reverse.
- Hold `A` / `D`: steer left / right; steering returns to center on release.
- Hold `Space`: brake to zero.
- `E`: latch remote E-stop; `Z`: reset at zero speed.
- Click `REMOTE` or press `R`: arm REMOTE control while stopped. The console
  clears any held directional key before arming, so a fresh `W/S/A/D` press is
  required to command motion.
- Click `AUTO` or press `M`: return to AUTO and disarm the remote console.
- `Q` or `Esc`: close the complete remote-driving session.
- Click any side/rear camera tile to promote it to the large main view.

The remote-driving simulation defaults to `3.0 m/s` forward, `0.3 m/s` reverse,
and `0.35 m/s^2` maximum drive acceleration. These are simulation limits, not
validated real-vehicle operating limits.

REMOTE control requires both explicit arming and window focus. Losing focus,
closing the window, or losing the operator process immediately disarms the
console and makes the vehicle stop. The legacy terminal controller remains available with
`AGV_REMOTE_TERMINAL_CONTROL=1 ./sim.sh remote`.

All six camera panels target the same `15 Hz` rate as the Gazebo cameras. Their
captions show received/displayed FPS; camera subscriptions keep only the newest
frame so temporary rendering load cannot build up an increasing video delay.

Inspect authority and safety state with:

```bash
ros2 topic echo /agv/control/authority/state
ros2 topic echo /agv/safety/state
```

中文说明：Humble 控制链现在包含 AUTO/REMOTE 仲裁器。远驾操作指令先经过 40 Hz
UDP/TLV 域控网关和模拟 B&R 车端控制器，再进入 REMOTE 输入；MPC 使用独立 AUTO
输入。只有当前获得控制权的一方能够进入 `agv_safety_supervisor`。默认是 `AUTO`，
并且车辆速度不超过 `0.05 m/s` 时才允许切换控制权。

只需执行 `./sim.sh remote`：它会启动无界面的 Gazebo server、RViz、控制仲裁器和
一个包含六路摄像头画面的 GTA 风格 WASD 驾驶台；Gazebo GUI 默认关闭，设置
`AGV_REMOTE_USE_GAZEBO_GUI=true` 可打开。若需要最高传感器渲染性能，可通过
`AGV_REMOTE_USE_RVIZ=false` 关闭 RViz。关闭驾驶台或按 `Ctrl+C` 会关闭整套远程
驾驶进程。必须点击 `REMOTE` 或按 `R` 显式解锁远驾，解锁时会清空残留方向键，随后
重新按下 `W/S/A/D` 才会产生运动指令。点击驾驶台窗口后，`W` 每次按下将目标
速度增加 `0.10 m/s`，`S` 每次按下降低 `0.10 m/s`，松开后保持定速，继续降低可
经过零速进入倒车；`A/D` 按住左右转向、松开自动回正，`Space` 制动。
`E` 锁存急停，停车后按 `Z` 复位；点击 `AUTO` 或按 `M` 会返回 AUTO 并撤销远驾
解锁。远驾同时要求显式解锁和窗口焦点，窗口失去焦点、被关闭或控制进程中断时，车辆
都会自动停车。旧终端控制仍可通过
`AGV_REMOTE_TERMINAL_CONTROL=1 ./sim.sh remote` 使用。

远驾仿真默认前进速度上限为 `3.0 m/s`、倒车上限为 `0.3 m/s`、最大驱动加速度为
`0.35 m/s^2`。它们是仿真限值，尚不是经过实车验证的运行限值。
如果终端或图形窗口被强制关闭，可执行 `./sim.sh stop` 一次性关闭本项目残留的
Gazebo、RViz、传感器桥和控制节点，再重新启动。
点击任意侧视或后视小画面，可以把该摄像头切换到上方主画面，再次点击其他画面即可
继续切换。

六个画面的目标频率与 Gazebo 摄像头相同，均为 `15 Hz`；实际频率取决于 Gazebo
传感器渲染负载。标题显示输入、编码、解码/显示 FPS、码率和画面年龄。每路使用
Gazebo Transport/C++ H.264 发送器、C++ 接收器和最新帧队列，短时处理不及时会
丢弃旧帧，不会无限累积延迟。`remote` 配置仅降低非主视相机分辨率和 H16 水平
采样，场景、频率、FOV、垂直线数和测距模型不变；`full` 配置用于原始参数对照，
`fast` 配置用于帧率优先运行。
默认单向网络延迟为 `50 ms`，也可切换到旧的原始图像链路：

```bash
./sim.sh remote remote_video_transport:=raw
```

视频和控制网络可以分别注入故障。例如保持控制链正常，仅让视频完全丢包：

```bash
./sim.sh remote remote_video_loss_percent:=100.0
```

此时前视接收端会变为 `WAITING/STALE`，控制状态变为
`REMOTE:STOPPED_VIDEO_STALE`。也可以独立设置 `remote_video_delay_ms` 和
`remote_video_jitter_ms`；未设置时继承公共的 `remote_network_*` 参数。

下一阶段是接入正式 RTP/SRT/WebRTC、真实视频服务器与 5G/QoS 接口，而不是继续
使用本机环回 UDP。

协议实现中的已知假设及向同事索取资料的清单见
[远程驾驶 UDP 协议状态](docs/remote_driving_udp_protocol_status.md)。
