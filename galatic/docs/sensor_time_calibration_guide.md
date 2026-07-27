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
- Front and rear cameras using the existing Galactic sensor profile.
  使用 Galactic 原有参数的前、后摄像头。
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

Current Galactic camera poses:

| Camera | x m | y m | z m | yaw rad | pitch rad |
| --- | ---: | ---: | ---: | ---: | ---: |
| front | 7.15 | 0.00 | 1.25 | 0.000000 | 0.000000 |
| rear | -7.15 | 0.00 | 1.25 | 3.141593 | 0.000000 |

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

### Helios reference profile / Helios 参考参数

The simulator uses the following RoboSense Helios reference profile. These are
manufacturer-based nominal values, not a replacement for the calibration file
and operating mode read from each physical lidar.

仿真采用以下 RoboSense Helios 厂家参考参数。这些是名义参数，不能替代从每台实物雷达
读取的标定文件和实际工作模式。

| Item / 项目 | Helios16 | Helios32 reference / 参考 | Simulator / 仿真 |
| --- | --- | --- | --- |
| Horizontal FoV / 水平视场 | 360 deg | 360 deg | 360 deg |
| Vertical FoV / 垂直视场 | -15 to +15 deg | H32F70: -55 to +15 deg | Same nominal FoV / 相同名义视场 |
| Horizontal step / 水平角分辨率 | 0.1/0.2/0.4 deg | 0.1/0.2/0.4 deg | H16: 0.2 deg/1800; H32: 0.4 deg/900 compatibility profile |
| Update rate / 更新率 | 5/10/20 Hz | 5/10/20 Hz | 10 Hz |
| Range / 量程 | 0.2-150 m | 0.2-150 m | 0.2-150 m |
| Nominal precision / 标称精度 | 1 cm | 1 cm | 1 cm Gaussian noise approximation / 高斯噪声近似 |
| Housing / 外形 | diameter 100 x 100 mm | diameter 100 x 100 mm | diameter 100 x 100 mm |

At the configured 10 Hz mode, 0.2 degree horizontal steps produce the nominal
single-return rates of 288,000 points/s for H16 and 576,000 points/s for H32.
H16 uses that native profile. To keep Humble and Galactic behavior aligned, H32
uses the stable 900x32 compatibility profile required by the current Humble
Gazebo GPU lidar stack. This preserves the H32F70 line count and vertical FoV,
but its simulated horizontal ray spacing is 0.4 degrees.

当前配置为 10 Hz，实物的 0.2 度模式下 H16 和 H32 单回波标称点频分别为
288,000 点/秒和 576,000 点/秒。H16 已按该模式设置；为了与 Humble 的稳定输出保持
一致，H32 暂用 900x32 兼容配置。该配置保留 H32F70 的线数和垂直视场，但仿真水平
射线间隔为 0.4 度。

`FR_LiDAR-1` and `RR_LiDAR-1` currently use the H32F70 profile because the exact
Helios32 subtype is not yet recorded in the project. `FL_LiDAR-2` and
`RR_LiDAR-2` retain the required upward-looking role with a 0 to +30 degree
simulation approximation. Standard Helios16 is -15 to +15 degrees; replace this
approximation with the measured mounting rotation and device calibration when
those values are available.

项目目前没有记录 Helios32 的完整子型号，因此 `FR_LiDAR-1` 和 `RR_LiDAR-1`
暂按 H32F70 建模。`FL_LiDAR-2` 和 `RR_LiDAR-2` 为保留上扫用途，暂用
0 到 +30 度的仿真近似；标准 Helios16 为 -15 到 +15 度。拿到实车安装角和设备标定
文件后，应使用实测外参和逐线角度替换该近似。

Gazebo distributes vertical channels uniformly. A physical Helios32 uses
factory-calibrated per-channel angles, so exact one-to-one point patterns require
the lidar calibration file or a recorded packet/PCAP sample.

Gazebo 当前将垂直线束均匀分布，而实物 Helios32 使用厂家逐线标定角。若要一比一复刻
点云线束，需要雷达标定文件或一份原始 packet/PCAP 样本。

Manufacturer references / 厂家资料：

- RoboSense Helios series: <https://store.robosense.ai/ja/products/helios-series>
- RS-Helios-16P manual: <https://cdn.robosense.cn/20220923161259_62013.pdf>
- RS-Helios-5515 manual: <https://cdn.robosense.cn/20220923164048_44264.pdf>

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
horizontal_fov = 80 deg
image size = 1280 x 720
```

仿真相机当前是简化针孔模型：

```text
水平视场角 = 80 deg
分辨率 = 1280 x 720
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
```

外参是从 `base_link` 到每个相机坐标系的变换：

```text
base_link -> front_camera
base_link -> rear_camera
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
