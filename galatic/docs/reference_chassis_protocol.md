# Reference AGV Chassis Protocol

This is a reference protocol for early integration of the Xiamen AGV controller
with a real chassis interface.

It is not an official ZPMC protocol. Use it as a template for discussion,
mock testing, and initial `vehicle_interface_node` design. Replace the message
IDs, scale factors, limits, watchdog timing, and safety state machine with the
vendor supplied DBC or Ethernet ICD before driving a real vehicle.

## Scope

The protocol maps the current ROS control surface to a chassis bus:

```text
ROS2 controller
  /control/command/control_cmd
  /agv/control/two_axis_steer_cmd
  /control/command/gear_cmd
        |
        v
vehicle_interface_node
        |
        v
CAN or Ethernet chassis protocol
        |
        v
VCU / steering ECU / drive ECU / brake ECU
```

The chassis feedback is mapped back to:

```text
/localization/kinematic_state
/vehicle/status/velocity_status
/vehicle/status/steering_status
/agv/status/front_steer
/agv/status/rear_steer
/vehicle/status/gear_status
/vehicle/status/control_mode
```

## Units And Conventions

All high level fields use SI units.

| Quantity | Unit | Positive Direction |
| --- | --- | --- |
| velocity | m/s | vehicle forward |
| acceleration | m/s^2 | vehicle forward acceleration |
| steering angle | rad | left turn positive |
| yaw rate | rad/s | counter-clockwise positive |
| brake command | normalized 0.0 to 1.0 | 1.0 is full brake |

Vehicle frame convention:

```text
base_link +x: forward
base_link +y: left
base_link +z: up
```

For an 8-wheel / 4-axle AGV, the reference protocol supports axle-level steering:

```text
axle1_steer: front axle pair
axle2_steer: second axle pair
axle3_steer: third axle pair
axle4_steer: rear axle pair
```

For the current simulation model:

```text
axle1_steer = front_steer
axle2_steer = rear_steer
axle3_steer = 0
axle4_steer = 0
```

## Bus Parameters

Reference CAN settings:

| Item | Value |
| --- | --- |
| bus | CAN 2.0B or CAN FD |
| nominal bitrate | 500 kbit/s |
| byte order | little endian |
| command rate | 50 Hz recommended, 20 Hz minimum |
| feedback rate | 50 Hz recommended |
| command watchdog | 100 ms |

For Ethernet, keep the same logical fields and send them at the same rates over
UDP or TCP. UDP is common for low-latency control, but the real vehicle vendor
must define packet layout, checksums, and timeout behavior.

## Command Messages

### 0x180 AGV_CMD_MODE

Sent by `vehicle_interface_node` to the VCU at 20 to 50 Hz.

| Field | Type | Unit | Description |
| --- | --- | --- | --- |
| alive_counter | uint8 | count | increments every frame |
| autonomous_enable | bool | - | request autonomous control |
| clear_fault | bool | - | request fault clear |
| estop_reset | bool | - | request E-stop reset, if allowed |
| gear_cmd | uint8 | enum | 0=PARK, 1=DRIVE, 2=REVERSE, 3=NEUTRAL |
| steering_mode | uint8 | enum | 0=front_only, 1=crab, 2=opposite_4ws, 3=axle12 |
| checksum | uint8 | - | simple checksum or vendor CRC |

### 0x181 AGV_CMD_MOTION

Sent by `vehicle_interface_node` to the VCU at 20 to 50 Hz.

| Field | Type | Scale | Unit | Range |
| --- | --- | --- | --- | --- |
| target_velocity | int16 | 0.001 | m/s | -32.768 to 32.767 |
| target_acceleration | int16 | 0.001 | m/s^2 | -32.768 to 32.767 |
| brake_cmd | uint16 | 0.001 | normalized | 0.000 to 1.000 |
| alive_counter | uint8 | 1 | count | 0 to 255 |
| checksum | uint8 | 1 | - | 0 to 255 |

Recommended behavior:

```text
If autonomous_enable is false: ignore target motion.
If brake_cmd > 0: brake command has priority over acceleration.
If command timeout > 100 ms: controlled stop.
```

### 0x182 AGV_CMD_STEER_AXLE12

Sent by `vehicle_interface_node` to the steering ECU at 20 to 50 Hz.

| Field | Type | Scale | Unit | Range |
| --- | --- | --- | --- | --- |
| axle1_steer_cmd | int16 | 0.0001 | rad | -3.2768 to 3.2767 |
| axle2_steer_cmd | int16 | 0.0001 | rad | -3.2768 to 3.2767 |
| axle3_steer_cmd | int16 | 0.0001 | rad | -3.2768 to 3.2767 |
| axle4_steer_cmd | int16 | 0.0001 | rad | -3.2768 to 3.2767 |

For the current controller:

```text
axle1_steer_cmd = /agv/control/two_axis_steer_cmd.data[0]
axle2_steer_cmd = /agv/control/two_axis_steer_cmd.data[1]
axle3_steer_cmd = 0
axle4_steer_cmd = 0
```

### 0x183 AGV_CMD_SAFETY

Sent by `safety_supervisor` or `vehicle_interface_node` to the VCU at 20 to
50 Hz.

| Field | Type | Description |
| --- | --- | --- |
| requested_state | uint8 | 0=STOP, 1=STANDBY, 2=AUTO, 3=MANUAL |
| controlled_stop | bool | request normal stop |
| emergency_stop | bool | request immediate stop |
| horn | bool | optional |
| lights | uint8 | optional |
| max_velocity_limit | uint16, scale 0.001 m/s | dynamic speed limit |
| checksum | uint8 | checksum or vendor CRC |

Safety priority:

```text
emergency_stop > manual_override > controlled_stop > autonomous_enable
```

## Feedback Messages

### 0x280 AGV_FB_STATUS

Sent by VCU to `vehicle_interface_node` at 20 to 50 Hz.

| Field | Type | Description |
| --- | --- | --- |
| autonomous_enabled | bool | chassis accepted autonomous mode |
| manual_override | bool | operator has control |
| emergency_stop_active | bool | E-stop chain active |
| gear_actual | uint8 | 0=PARK, 1=DRIVE, 2=REVERSE, 3=NEUTRAL |
| control_mode | uint8 | 0=MANUAL, 1=AUTO, 2=AUTO_STEER_ONLY, 3=AUTO_VELOCITY_ONLY |
| fault_level | uint8 | 0=OK, 1=WARN, 2=FAULT, 3=ESTOP |
| alive_counter | uint8 | count |

### 0x281 AGV_FB_MOTION

Sent by VCU to `vehicle_interface_node` at 50 Hz.

| Field | Type | Scale | Unit |
| --- | --- | --- | --- |
| actual_velocity | int16 | 0.001 | m/s |
| actual_acceleration | int16 | 0.001 | m/s^2 |
| actual_yaw_rate | int16 | 0.0001 | rad/s |
| brake_actual | uint16 | 0.001 | normalized |

### 0x282 AGV_FB_STEER_AXLE12

Sent by steering ECU to `vehicle_interface_node` at 50 Hz.

| Field | Type | Scale | Unit |
| --- | --- | --- | --- |
| axle1_steer_actual | int16 | 0.0001 | rad |
| axle2_steer_actual | int16 | 0.0001 | rad |
| axle3_steer_actual | int16 | 0.0001 | rad |
| axle4_steer_actual | int16 | 0.0001 | rad |

Mapping to current ROS feedback:

```text
/agv/status/front_steer = axle1_steer_actual
/agv/status/rear_steer  = axle2_steer_actual
/vehicle/status/steering_status.steering_tire_angle = axle1_steer_actual
```

### 0x283 AGV_FB_FAULT

Sent by VCU to `vehicle_interface_node` at 10 to 20 Hz.

| Field | Type | Description |
| --- | --- | --- |
| fault_code_0 | uint16 | vendor defined |
| fault_code_1 | uint16 | vendor defined |
| fault_code_2 | uint16 | vendor defined |
| fault_code_3 | uint16 | vendor defined |

Recommended reference fault code ranges:

```text
0x0000: no fault
0x0100-0x01FF: communication faults
0x0200-0x02FF: steering faults
0x0300-0x03FF: drive/brake faults
0x0400-0x04FF: localization/sensor faults
0x0500-0x05FF: safety supervisor faults
```

## ROS Mapping

### Command Mapping

`/control/command/control_cmd`:

```text
longitudinal.velocity     -> target_velocity
longitudinal.acceleration -> target_acceleration
lateral.steering_tire_angle -> fallback axle1 steering, if two-axis command is absent
```

`/agv/control/two_axis_steer_cmd`:

```text
data[0] -> axle1_steer_cmd
data[1] -> axle2_steer_cmd
data[2] -> target_velocity
data[3] -> target_acceleration
```

`/control/command/gear_cmd`:

```text
DRIVE   -> gear_cmd = 1
REVERSE -> gear_cmd = 2
PARK    -> gear_cmd = 0
```

### Feedback Mapping

```text
actual_velocity -> /vehicle/status/velocity_status.longitudinal_velocity
actual_yaw_rate -> /vehicle/status/velocity_status.heading_rate
axle1_steer_actual -> /agv/status/front_steer
axle2_steer_actual -> /agv/status/rear_steer
gear_actual -> /vehicle/status/gear_status
control_mode -> /vehicle/status/control_mode
```

`/localization/kinematic_state` should normally come from GNSS/INS plus vehicle
odometry fusion, not from chassis CAN alone. Chassis feedback may provide `vx`
and yaw rate as fusion inputs.

## Watchdog And Safety Rules

Minimum recommended safety behavior:

```text
If command frames missing for > 100 ms: controlled stop.
If autonomous_enable false: ignore motion and steer commands.
If emergency_stop_active true: ignore all motion commands.
If manual_override true: ignore all autonomous commands.
If localization timeout > 200 ms: safety_supervisor requests controlled_stop.
If lateral error > configured limit: safety_supervisor requests controlled_stop.
If yaw error > configured limit: safety_supervisor requests controlled_stop.
```

The VCU should independently enforce:

```text
velocity limit
acceleration limit
steering angle limit
steering rate limit
brake priority
watchdog timeout
E-stop priority
```

## Validation Checklist

Before closed-loop driving:

```text
1. Replay command frames on a bench, with wheels off the ground if possible.
2. Verify endian, sign, scale, and units for every signal.
3. Verify left steering is positive in ROS and on the vehicle.
4. Verify rear/second axle steering sign.
5. Verify watchdog stop by stopping command transmission.
6. Verify emergency stop overrides all commands.
7. Verify manual override overrides all commands.
8. Run shadow mode and compare MPC commands with human driving.
9. Start closed-loop tests below 0.5 m/s.
```

