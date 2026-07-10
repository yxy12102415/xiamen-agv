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
- 当前 Ackermann 内外轮差角已经用于 TF 和 Gazebo 车轮可视化；MPC 和车辆动力学
  模型仍使用轴中心等效转角。

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

RViz 使用 `map` 作为固定坐标系。仿真器发布 `map -> base_link`，启动文件发布
从 `base_link` 到激光雷达、IMU、GNSS 等传感器坐标系的静态 TF。若点云位置漂移
或闪烁，优先检查 Gazebo SDF、启动参数和实车标定外参是否一致。

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
- Wheel TF / Gazebo wheel visualization now applies Ackermann inner/outer wheel
  angle geometry for the first two steering axles.

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

- Mass: `23000 kg`.
- Wheel rolling radius: `0.66 m`.
- Steering rate limit: `0.261799 rad/s` = `15 deg/s`.
- Max drive acceleration: `0.35 m/s^2`.
- Max brake deceleration: `4.0 m/s^2`, aligned with the line-control braking
  requirement rather than the older table value of `0.35 m/s^2`.
- Ackermann visual wheel angles:
  - front: `27.4 deg` outer gives approximately `33.0 deg` inner,
  - rear: `22.6 deg` outer gives approximately `30.0 deg` inner.

Estimated / not measured:

- `yaw_inertia_kgm2 = 432363.7513`.
- Tire cornering stiffness values.
- Rolling resistance, aerodynamic drag, frontal area, and wheel inertia.
- Current axle positions in the model are based on the Gazebo geometry and are
  not yet matched to the sheet values of 4.65 m for axles 1/4 and 3.0 m for
  axles 2/3.

Important limitation: Ackermann inner/outer angles are currently applied to TF
and Gazebo wheel visualization. The MPC and vehicle dynamics model still use
axis-center equivalent steering angles.

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

- Horizontal field of view: `120 deg`.
- Resolution: `960 x 540`.
- Update rate: `15 Hz`.

You can check a stream with:

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

- 水平视场角：`120 deg`。
- 分辨率：`960 x 540`。
- 刷新率：`15 Hz`。

查看视频可以使用：

```bash
ros2 run rqt_image_view rqt_image_view
```

远程驾驶的下一步建议是在这些图像话题后面增加视频编码/传输节点，例如
H.264/WebRTC/GStreamer，并加入网络延迟、抖动、丢包和中断仿真。当网络中断或
控制指令超时时，车辆端应通过 safety supervisor 自动制动并进入安全停车模式。
