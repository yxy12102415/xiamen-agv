# Required Chassis CAN / Ethernet Signals / 底盘 CAN 或以太网信号需求

The existing `reference_chassis_protocol.md` and `reference_agv_chassis.dbc`
are templates only. They must be replaced by the official vendor DBC or Ethernet
ICD before real actuation. 现有参考协议和 DBC 只能用于讨论和台架 mock，不能直接
作为厂家实车协议使用。

## Command Signals / 控制信号

| ROS input | Chassis signal | Unit | Notes |
| --- | --- | --- | --- |
| `/control/command/control_cmd.longitudinal.velocity` | target velocity | m/s | Positive forward. 正值前进。 |
| `/control/command/control_cmd.longitudinal.acceleration` | target acceleration or drive/brake request | m/s^2 | Negative may mean braking. 负值可表示制动。 |
| `/agv/control/two_axis_steer_cmd.data[0]` | front axle steering command | rad or deg | Sign must be confirmed. 需确认符号方向。 |
| `/agv/control/two_axis_steer_cmd.data[1]` | rear axle steering command | rad or deg | Sign must be confirmed. 需确认符号方向。 |
| `/agv/control/two_axis_steer_cmd.data[2]` | optional target velocity | m/s | Used by current two-axis path. 当前双轴链路会用。 |
| `/agv/control/two_axis_steer_cmd.data[3]` | optional acceleration | m/s^2 | Used by current two-axis path. 当前双轴链路会用。 |
| `/control/command/gear_cmd` | gear command | enum | PARK/DRIVE/REVERSE/NEUTRAL mapping required. |
| safety supervisor output | controlled stop | bool/enum | Higher priority than normal motion. 高于普通运动。 |
| safety supervisor output | emergency stop | bool/enum | Highest priority. 最高优先级。 |
| vehicle interface | autonomous enable | bool | Must have accepted feedback. 必须有底盘确认反馈。 |
| vehicle interface | clear fault / reset | bool | Only if allowed by vendor safety process. |
| vehicle interface | heartbeat / alive counter | count | Required for watchdog. 用于看门狗。 |
| vehicle interface | checksum / CRC | vendor-defined | Required if protocol uses it. |

## Feedback Signals / 反馈信号

| Chassis feedback | ROS output | Unit | Notes |
| --- | --- | --- | --- |
| actual velocity | `/vehicle/status/velocity_status.longitudinal_velocity` | m/s | Signed. 有符号。 |
| actual acceleration | `/vehicle/status/velocity_status.longitudinal_acceleration` | m/s^2 | Strongly recommended. 强烈建议提供。 |
| yaw rate | odometry twist or diagnostic topic | rad/s | Needed for model validation. 用于模型验证。 |
| front axle actual steering | `/agv/status/front_steer` | rad | Required. 必需。 |
| rear axle actual steering | `/agv/status/rear_steer` | rad | Required for two-axis steering. 双轴转向必需。 |
| wheel speeds | diagnostic or odometry input | m/s or rad/s | Useful for slip checks. 用于打滑检查。 |
| brake pressure / brake actual | diagnostic topic | bar or normalized | Needed for brake validation. |
| gear actual | `/vehicle/status/gear_status` | enum | Must match Autoware gear enum. |
| control mode actual | `/vehicle/status/control_mode` | enum | Manual/autonomous/partial autonomous. |
| autonomous accepted | `/vehicle/status/control_mode` or interface state | bool | Do not drive before accepted. 未确认前不能驱车。 |
| manual override active | interface state / safety topic | bool | Must override autonomous output. |
| E-stop active | `/agv/safety/emergency_stop` | bool | Preserve latch behavior. 保持锁存逻辑。 |
| fault level | `/agv/vehicle_interface/state` and diagnostics | enum | OK/WARN/FAULT/ESTOP. |
| fault codes | diagnostics | vendor-defined | Preserve raw values. 保留原始值。 |
| feedback heartbeat / counter | diagnostics | count | Used for feedback timeout. |

## Details Required For Every Message / 每条报文必须明确

- CAN ID or Ethernet packet type. CAN ID 或以太网 packet type。
- Standard/extended frame and CAN 2.0/CAN FD. 标准帧/扩展帧，CAN 2.0/CAN FD。
- DLC or payload length. DLC 或 payload 长度。
- Byte order. 大小端。
- Signal bit position and length. bit 位置和长度。
- Signedness. 有无符号。
- Scale and offset. 缩放和偏置。
- Valid range and default value. 有效范围和默认值。
- Send period and timeout behavior. 周期和超时行为。
- Counter and checksum/CRC algorithm. counter 和校验算法。
- Receiver behavior on invalid checksum/counter. 校验或 counter 错误时的处理。

## Safety States To Confirm / 需要确认的安全状态

```text
POWER_OFF
STANDBY
MANUAL
REMOTE_MANUAL
AUTONOMOUS_READY
AUTONOMOUS_ACTIVE
CONTROLLED_STOP
FAULT
EMERGENCY_STOP
```

At minimum, confirm / 至少确认：

- Which command requests autonomous mode. 哪个命令请求自动驾驶。
- Which feedback confirms autonomous mode. 哪个反馈确认自动驾驶已接受。
- What happens if commands stop for 100 ms, 200 ms, 500 ms, and 1 s.
  控制命令停止不同时间后的底盘行为。
- What happens if feedback stops. 反馈停止后的行为。
- What happens if the remote network drops. 远程网络断开后的行为。
- What happens if CAN bus-off occurs. CAN bus-off 后的行为。
- How manual takeover is detected. 如何检测人工接管。
- Whether E-stop is latched and how reset is allowed. 急停是否锁存及复位条件。

## Bench Acceptance / 台架通过标准

- Outbound frames match the DBC/ICD in CAN logs. 发出报文和 DBC/ICD 一致。
- Inbound frames decode correctly from real logs. 真实录包可正确解析。
- Steering left/right sign is confirmed. 转向左右符号确认。
- Drive/brake sign and priority are confirmed. 驱动/制动符号和优先级确认。
- Autonomous enable requires accepted feedback. 自动使能必须有确认反馈。
- Command timeout produces controlled stop. 控制超时触发受控停车。
- E-stop feedback is decoded and published. 急停反馈正确发布。
- Manual override disables autonomous output. 人工接管禁止自动输出。
