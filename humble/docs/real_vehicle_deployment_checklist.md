# Real Vehicle Deployment Checklist / 实车部署检查清单

This document describes the gap between the current Humble simulation stack and
a 1:1 real AGV deployment. 当前项目已经有仿真、MPC、安全监督器和实车接口骨架，
但还不是完整的实车部署包。

## Target Architecture / 目标架构

Current simulation mode / 当前仿真模式：

```text
trajectory or remote-control command
  -> /control/command/control_cmd_raw
  -> /agv/control/two_axis_steer_cmd_raw
  -> agv_safety_supervisor
  -> /control/command/control_cmd
  -> /agv/control/two_axis_steer_cmd
  -> agv_two_axis_simulator_node
  -> /localization/kinematic_state
  -> Gazebo AGV pose
```

Real-vehicle target mode / 实车目标模式：

```text
trajectory or remote-control command
  -> /control/command/control_cmd_raw
  -> /agv/control/two_axis_steer_cmd_raw
  -> agv_safety_supervisor
  -> /control/command/control_cmd
  -> /agv/control/two_axis_steer_cmd
  -> real_vehicle_interface_node
  -> socketcan or Ethernet ICD
  -> VCU / steering ECU / drive ECU / brake ECU
  -> real chassis feedback
  -> /vehicle/status/*
  -> /localization/kinematic_state
```

The key change is replacing the simulator as the final actuator with the real
chassis interface and real feedback. 核心区别是实车时不能再由
`agv_two_axis_simulator_node` 作为最终底盘，必须接入真实底盘接口和真实反馈。

## Required Vendor Inputs / 需要厂家提供

- Official CAN DBC or Ethernet ICD. 正式 CAN DBC 或以太网 ICD。
- Command and feedback message IDs. 控制和反馈报文 ID。
- Byte layout, bit positions, endianness, signedness, scale, and offset.
  字节布局、bit 位置、大小端、有无符号、scale、offset。
- Message period and timeout behavior. 报文周期和超时行为。
- Alive counter and checksum/CRC algorithm. alive counter 和 checksum/CRC。
- Autonomous/manual/remote/local mode state machine. 自动、手动、远程、本地模式状态机。
- E-stop, remote E-stop, manual override, brake priority, and fault reset rules.
  急停、远程急停、人工接管、制动优先级、故障复位规则。
- Drive, brake, steering, gear, parking brake, and enable commands.
  驱动、制动、转向、档位、驻车制动、使能命令。
- Velocity, acceleration, steering, wheel speed, brake, gear, mode, fault,
  heartbeat, and E-stop feedback. 车速、加速度、转角、轮速、制动、档位、
  模式、故障、心跳、急停反馈。

## Software Work Before Real Driving / 实车前软件工作

- Replace the reference protocol in `real_vehicle_interface_node.py` with the
  official vendor protocol. 用厂家正式协议替换当前参考协议。
- Add pack/unpack tests using recorded CAN frames. 用真实 CAN 录包做打包/解析测试。
- Validate command signs and scaling on a bench rig. 台架验证命令正负方向和缩放。
- Decode all required feedback and faults. 补全全部状态和故障反馈解析。
- Publish real feedback to:
  - `/vehicle/status/velocity_status`
  - `/vehicle/status/steering_status`
  - `/vehicle/status/gear_status`
  - `/vehicle/status/control_mode`
  - `/agv/status/front_steer`
  - `/agv/status/rear_steer`
  - `/localization/kinematic_state`
- Keep `dry_run=true`, `shadow_mode=true`, and `autonomous_enable=false` until
  bench tests pass. 台架通过前保持默认安全状态，不实际驱动车辆。
- Verify command timeout, feedback timeout, E-stop, and manual takeover.
  验证控制超时、反馈超时、急停和人工接管。

## Vehicle Parameters / 车辆参数

Measured values required for a 1:1 model / 一比一模型需要实测：

- Empty and loaded mass. 空载和负载质量。
- Center of gravity position and height. 质心位置和高度。
- Yaw inertia. 横摆转动惯量。
- Axle positions and axle loads. 各轴位置和轴荷。
- Tire cornering stiffness. 轮胎侧偏刚度。
- Empty/loaded rolling radius. 空载/负载滚动半径。
- Steering limits and steering rate limits. 转角和转角速度限制。
- Drive/brake acceleration limits. 驱动和制动加速度限制。
- Drive, brake, and steering delays/time constants. 驱动、制动、转向响应延迟和时间常数。

For loads, start with parameter sets / 负载建议先做离散参数集：

```text
empty
loaded_light
loaded_heavy
```

Then move to gain scheduling or adaptive MPC after validation. 验证后再做增益调度
或 adaptive MPC。

## Sensor And Localization / 传感器和定位

- Calibrate lidar, camera, GNSS, and IMU extrinsics against `base_link`.
  标定各传感器相对 `base_link` 的外参。
- Confirm the real `base_link` definition. 确认实车 `base_link` 定义。
- Confirm map origin and projection parameters. 确认地图原点和投影参数。
- Verify time synchronization across sensors and chassis CAN. 验证传感器和 CAN 时间同步。
- Validate remote-driving camera height, pitch, FOV, and latency. 验证远程驾驶摄像头高度、
  俯仰角、视场角和延迟。
- Validate lidar blind zones with actual brackets. 使用真实支架验证 lidar 盲区。

## Safety Tests / 安全测试

- CAN disconnect. CAN 断开。
- Network disconnect. 网络断开。
- Command timeout. 控制指令超时。
- Feedback timeout. 反馈超时。
- Hardware E-stop. 硬件急停。
- Remote E-stop. 远程急停。
- Manual takeover. 人工接管。
- Brake priority. 制动优先级。
- Steering limit. 转向限幅。
- Low-speed straight-line and steering tests. 低速直线和转向测试。
- Low-speed closed-course trajectory tracking. 低速封闭路线跟踪。
- Empty and loaded braking tests. 空载/负载制动测试。

## Bringup Stages / 上车阶段

1. Dry run: no physical output, only log frames. 不输出真实控制，只记录报文。
2. Shadow mode: decode real feedback, no actuation. 解析真实反馈，不驱动车。
3. Bench mode: lifted wheels or actuator-safe rig. 车轮离地或安全台架。
4. Low-speed remote driving. 低速远程驾驶。
5. Low-speed autonomous trajectory tracking. 低速自动轨迹跟踪。
6. Empty closed-course validation. 空载封闭场地验证。
7. Loaded closed-course validation. 负载封闭场地验证。
8. Operational scenario validation. 真实作业场景验证。
