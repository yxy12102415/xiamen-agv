# Sensor, Time, And Calibration Guide / 传感器、时间同步和标定说明

This guide explains how the simulation currently models timestamping,
coordinate frames, sensor mounting, lidar distortion, and camera calibration,
and what must be done on the real vehicle.

本文说明当前仿真如何模拟时间戳、坐标系、传感器安装、点云畸变和相机标定，以及实
车上需要如何校准。

## What The Simulation Can Already Provide / 仿真里已能提供

- Chassis-like feedback topics under `/agv/chassis/*`.
  `/agv/chassis/*` 下的模拟底盘反馈。
- Sensor frames under `base_link`.
  `base_link` 下的传感器坐标系。
- Six 120-degree cameras at `z = 1.95 m`.
  6 路 120 度摄像头，高度 `1.95 m`。
- Lidar point clouds and camera images from Gazebo.
  Gazebo 生成的点云和图像。
- A ROS `/clock` bridge from Gazebo.
  Gazebo 到 ROS 的 `/clock` 桥接。

## Timestamp Source / 时间戳来源

Current simulation:

- Gazebo publishes `/clock`.
- ROS nodes currently use their normal node clock unless `use_sim_time` is set.
- The simulated chassis feedback summary publishes a JSON field:
  `timestamp_source = ros_node_clock`.

当前仿真：

- Gazebo 发布 `/clock`。
- ROS 节点默认使用普通 ROS 节点时钟；若设置 `use_sim_time`，则使用仿真时间。
- 模拟底盘反馈的 summary 中包含 `timestamp_source = ros_node_clock`。

Recommended real vehicle rule:

- All sensor drivers and chassis feedback should use one time base.
- Prefer GNSS/PTP synchronized host time if available.
- Record both receive time and hardware time when the device provides hardware
  timestamps.

实车建议：

- 所有传感器驱动和底盘反馈使用统一时间基准。
- 优先使用 GNSS/PTP 同步后的主机时间。
- 如果设备提供硬件时间戳，同时记录接收时间和硬件时间。

## Coordinate Frames / 坐标系方向

The project uses the standard `base_link` convention:

```text
+x: vehicle forward
+y: vehicle left
+z: up
```

项目采用标准 `base_link`：

```text
+x：车辆前方
+y：车辆左侧
+z：向上
```

Before real deployment, confirm where `base_link` is on the actual AGV: usually
ground projection of the vehicle center or rear/center axle reference. All
sensor extrinsics must be measured relative to that exact point.

实车部署前必须确认 `base_link` 在真实车体上的位置，通常是车辆中心地面投影或某
个轴系参考点。所有传感器外参都必须相对这个点测量。

## Mount Height And Angle / 安装高度和角度

Current simulated remote-driving camera poses:

| Camera | x m | y m | z m | yaw rad | pitch rad |
| --- | ---: | ---: | ---: | ---: | ---: |
| front | 7.65 | 0.00 | 1.95 | 0.000000 | 0.174533 |
| rear | -7.65 | 0.00 | 1.95 | 3.141593 | 0.174533 |
| left_front | 3.75 | 1.65 | 1.95 | 1.570796 | 0.174533 |
| left_rear | -3.75 | 1.65 | 1.95 | 1.570796 | 0.174533 |
| right_front | 3.75 | -1.65 | 1.95 | -1.570796 | 0.174533 |
| right_rear | -3.75 | -1.65 | 1.95 | -1.570796 | 0.174533 |

To calibrate real mounting:

1. Measure bracket position from the chosen `base_link`.
2. Measure camera optical center, not only the bracket surface.
3. Measure yaw/pitch/roll with a level/inclinometer or calibration target.
4. Validate in RViz by projecting known lane lines or calibration markers.

实车安装校准：

1. 从确定好的 `base_link` 测量支架位置。
2. 测量摄像头光心位置，而不只是支架表面。
3. 用水平仪、倾角仪或标定板测量 yaw/pitch/roll。
4. 在 RViz 中用已知车道线或标定目标验证投影是否对齐。

## Lidar Motion Distortion / 点云畸变

Gazebo currently emits idealized lidar scans. It does not fully model all real
motion distortion, rolling acquisition timing, packet timing, or per-point time
offsets.

当前 Gazebo 点云是理想化扫描，没有完整模拟真实激光雷达的运动畸变、逐线扫描时序、
包时间和每点时间偏移。

For real vehicle deployment:

- Use lidar packets or point fields that include per-point time when available.
- Deskew point clouds using IMU/chassis odometry.
- Validate deskewing by driving past straight container edges, poles, or lane
  lines and checking whether they stay straight in the point cloud.

实车部署：

- 若雷达支持，使用带每点时间的点云或原始 packet。
- 使用 IMU/底盘里程计进行点云去畸变。
- 通过直线集装箱边缘、杆件、车道线验证去畸变后点云是否保持直线。

## Camera Intrinsics / 相机内参

Gazebo cameras currently use a simplified pinhole model with:

```text
horizontal_fov = 120 deg
image size = 960 x 540
```

仿真相机当前是简化针孔模型：

```text
水平视场角 = 120 deg
分辨率 = 960 x 540
```

For real cameras, calibrate:

- `fx`, `fy`, `cx`, `cy`.
- distortion coefficients, usually `k1`, `k2`, `p1`, `p2`, `k3`.
- camera frame convention and optical frame convention.

实车相机需要标定：

- `fx`、`fy`、`cx`、`cy`。
- 畸变参数，通常是 `k1`、`k2`、`p1`、`p2`、`k3`。
- camera frame 和 optical frame 的坐标系方向。

Recommended tools:

- ROS camera calibration package with checkerboard or AprilTag grid.
- Save calibration as `sensor_msgs/CameraInfo` YAML.
- Compare projected map/markers against the image after calibration.

建议工具：

- ROS camera calibration，使用棋盘格或 AprilTag 标定板。
- 将结果保存为 `sensor_msgs/CameraInfo` YAML。
- 标定后把地图线或标定物投影到图像中检查误差。

## Camera Extrinsics / 相机外参

Extrinsics are the transform from `base_link` to each camera frame:

```text
base_link -> front_camera
base_link -> rear_camera
base_link -> left_front_camera
...
```

外参是从 `base_link` 到每个相机坐标系的变换：

```text
base_link -> front_camera
base_link -> rear_camera
base_link -> left_front_camera
...
```

Calibration methods:

- Manual measurement plus RViz validation for early simulation.
- Target-based calibration using known 3D points.
- Multi-sensor calibration using lidar-camera targets if lidar and camera must
  align precisely.

校准方法：

- 早期仿真可以人工测量加 RViz 验证。
- 使用已知 3D 点和标定板做目标标定。
- 若需要 lidar-camera 精确对齐，使用雷达-相机联合标定目标。

## What Was Added For Real-Vehicle Replication / 本次为复刻实车增加的内容

- `simulated_chassis_feedback_node` publishes real-chassis-like feedback from
  simulator state.
- `/agv/chassis/state_summary` publishes a timestamped JSON summary.
- The main Humble launch starts simulated chassis feedback by default.
- This guide documents time, frame, mounting, lidar distortion, camera intrinsic,
  and camera extrinsic calibration requirements.

本次增加：

- `simulated_chassis_feedback_node`：从仿真状态发布接近真实底盘的反馈。
- `/agv/chassis/state_summary`：发布带时间戳的 JSON 状态汇总。
- Humble 主启动文件默认启动模拟底盘反馈节点。
- 本文档说明时间、坐标系、安装、点云畸变、相机内参和外参标定要求。
