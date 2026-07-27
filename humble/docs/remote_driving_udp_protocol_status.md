# Remote Driving UDP Protocol Status / 远程驾驶 UDP 协议状态

## 已实现 / Implemented

本项目已根据《功能设计》在 Humble 中加入一个可运行的协议回环：

```text
WASD 操作台
  -> /remote/operator/*
  -> domain_udp_gateway
  -> UDP 40 Hz + binary TLV
  -> mock_br_controller
  -> /remote/control/*
  -> remote_drive_command_mux
  -> agv_safety_supervisor
  -> 车辆模型
```

The Humble workspace now contains an executable loopback implementation based on
the supplied design document. It includes:

- fixed 20-byte message header and TLV item encoding;
- domain command `0x7100` with items `0x7101` and `0x7102`;
- vehicle state `0x7000` with item `0x7101`;
- 40 Hz UDP send/receive loops;
- operator timeout, command timeout, deadman, E-stop, and safe-stop behavior;
- simulated B&R vehicle-controller endpoint;
- vehicle feedback returned to `/remote/protocol/vehicle_state`;
- link diagnostics on `/remote/protocol/domain_state`.
- application-level delay, jitter, and packet-loss simulation for both UDP directions;
- six independent H.264 camera links with per-camera FPS, bitrate, frame age,
  delivery delay, queue depth, and drop counters;
- C++ decoded-image receivers to avoid Python raw-image serialization bottlenecks;
- a front-video health interlock that stops REMOTE output on freeze or timeout;
- default one-way delay of `50 ms` for commands, state feedback, and video.

当前网络仿真默认值是：单向延迟 `50 ms`、抖动 `0 ms`、丢包率 `0%`。控制指令
和状态反馈分别经过一次延迟，因此默认控制往返链路约为 `100 ms`，另有少量操作系统
和 ROS 调度开销。六路摄像头从 `/sensing/camera/*/image` 经 H.264 编码、UDP
网络损伤代理和解码后发布到 `/remote/video/*/image`，远驾控制台只订阅接收端图像。

默认仿真地址是域控 `127.0.0.1:46000`、车端 `127.0.0.1:46001`。启动命令：

```bash
cd /data/projects/xiamen-agv-sim/humble
./sim.sh remote
```

自定义网络条件：

```bash
# 50 ms 单向延迟（默认）
./sim.sh remote remote_network_delay_ms:=50.0

# 100 ms 单向延迟、20 ms 随机抖动、2% 丢包
./sim.sh remote \
  remote_network_delay_ms:=100.0 \
  remote_network_jitter_ms:=20.0 \
  remote_network_loss_percent:=2.0

# 控制链保持正常，只中断视频
./sim.sh remote remote_video_loss_percent:=100.0
```

The common `remote_network_*` arguments configure command UDP, vehicle-state
UDP, and video by default. `remote_video_delay_ms`, `remote_video_jitter_ms`,
and `remote_video_loss_percent` override only the six camera links. This allows
video-only faults while the control link remains healthy. The simulator needs
neither root access nor Linux `tc/netem` configuration.

### Video transport boundary / 视频传输边界

The camera link keeps Gazebo rendering and sensor settings unchanged. Six
Python senders encode full frames using five `h264_nvenc` sessions plus one
reserved `libx264` session (with automatic runtime fallback),
carry Annex-B packets over loopback UDP, and apply delay/jitter/loss and
bandwidth limits. Six C++ receivers drive FFmpeg decoders and publish directly
to `/remote/video/*/image`. Defaults are
`1.5 Mbit/s` per camera, `100 Mbit/s` aggregate capacity, and `50 ms` one-way
network delay. Synthetic 15 Hz tests measured approximately `117-161 ms`
capture-to-decoded delivery with the 50 ms network setting.

当前链路不会改变 Gazebo 场景和摄像头参数。六路 Python 发送节点分别进行
`h264_nvenc` 编码（不可用时回退 `libx264`）、Annex-B/UDP 传输、延迟/抖动/丢包与
带宽限制；六路 C++ 接收节点通过 FFmpeg 解码并发布到 `/remote/video/*/image`。
默认每路 `1.5 Mbit/s`、总上限
`100 Mbit/s`、单向网络延迟 `50 ms`。15 Hz 合成画面测试的采集到解码时延约
`117-161 ms`。这仍是本机环回原型，不是正式 RTP/SRT/WebRTC/5G 链路，也没有鉴权、
加密、FEC 或真实运营商 QoS。

REMOTE 模式默认要求前视接收端保持 LIVE。连续约 `1 s` 没有前视解码帧，或超过
`1.5 s` 没有接收端健康诊断，仲裁器输出 `REMOTE:STOPPED_VIDEO_STALE` 并使用最大
`4 m/s^2` 制动限值生成停车指令。发送端诊断只说明码流已经发出，不能解除该安全门。

当前回归结果：320x180 合成源在 15 Hz 下编码和 C++ 解码均为约 `15.0 fps`，订阅端
约 `14.3 fps`，50 ms 配置下包延迟约 `51.8 ms`；100% 视频丢包测试中接收端为
`0` 帧、发送端正确统计全部丢包。持续控制命令下停止接收端视频健康消息后，仲裁器
在约 `1.46 s` 后进入 `STOPPED_VIDEO_STALE`。完整港区场景未做任何简化，六路观察
帧率约 `4.9-10.5 fps`，当前剩余瓶颈主要是 Gazebo 相机和激光雷达 GPU 渲染。

连接真实 B&R 控制器时关闭模拟端，并传入真实地址，例如：

```bash
./sim.sh remote \
  enable_remote_protocol_mock:=false \
  remote_protocol_local_host:=0.0.0.0 \
  remote_protocol_local_port:=46000 \
  remote_protocol_vehicle_host:=192.168.1.20 \
  remote_protocol_vehicle_port:=46001
```

在下列假设得到确认前，只能用于仿真和台架联调，不能直接控制实车。

## 当前协议假设 / Current Assumptions

- 字节序暂定为小端；可通过 `remote_protocol_endian:=big` 切换。
- TLV 的 `item_length` 暂定只包含 value，不包含 4 字节 TLV 头。
- `msg_length` 暂定为整个 UDP 数据报长度，`item_length` 为全部 TLV body 长度。
- `0x7101` 远程控制 value 按字段合计采用 12 字节，不采用文档中冲突的 8/15 字节描述。
- `0x7102` 固定为 20 字节，8 个按钮位暂时全部发送 0。
- 没有添加文档未定义的 CRC/checksum、alive counter 或额外对齐填充。
- 转向、速度、制动比例按 `-100..100` 或 `0..100` 百分比解释。
- 模拟反馈中的转向量暂按前轴实际转角（degree）解释。
- 模拟反馈位姿使用 ROS `map` 坐标，航向角使用 degree。
- 电量暂为 100%，障碍物多边形和故障数组暂为空。
- 文档写 5 路摄像头，但当前车辆布置图和仿真均为 6 路，暂按 6 路实现。

## 请向同事索取：P0 协议资料 / Required Before Wire Compatibility

下面内容决定真实控制器能否正确收包，优先级最高：

1. 正式 UDP ICD 的版本号，以及域控和 B&R 双方 IP、发送端口、监听端口、是否组播。
2. 完整报文的字节序、结构体是否按 1 字节对齐、float 是否 IEEE-754。
3. `msg_length`、包头 `item_length`、每个 TLV `item_length` 的精确定义。
4. `0x7101` 控制项的正式长度；请解决文档中 8、12、15 字节冲突。
5. checksum/CRC、alive counter、消息 ID 回绕和丢包判定算法。
6. 心跳周期、命令超时阈值、断网后的制动/驻车流程和恢复握手。
7. 一组真实十六进制收发样例或一段 Wireshark pcap，至少包含前进、转向、制动、急停和状态反馈。
8. `ControlMode`、档位、状态位、故障等级和故障码的完整枚举表。
9. 8 个 button 字节定义、障碍检测 bypass 权限和远程/本地模式切换状态机。
10. 转向反馈字段到底是角度、角速度还是百分比，单位和正负方向是什么。
11. 位姿 `x/y/heading` 的坐标系、原点、轴方向、角度单位及时间戳来源。

## 请向同事索取：车辆与执行器 / Vehicle and Actuator Data

你已经提供并已写入模型：车长 `15.0 m`、车宽 `2.98 m`、质量 `23000 kg`、四轴
相对质心 x 坐标 `+4.65/+3.00/-3.00/-4.65 m`、轮胎滚动半径、转角/转速限制。

仍缺正式测量值：

1. 空载和各典型负载下的总质量、质心 `x/y/z`、偏航转动惯量 `Iz`、各轴轴荷。
2. 正式轮距、轮胎宽度、车身总高、前后悬和 `base_link` 的实车定义位置。
3. 轮胎侧偏刚度或转向试验数据；湿滑、干燥地面摩擦参数。
4. 驱动、制动、前后转向执行器的延迟、时间常数、死区、回差、饱和和速率限制。
5. 制动压力到减速度的标定曲线，以及空载/负载差异。
6. 实车最大前后轴内外轮转角的定义和每个轮的角度反馈来源。

## 请向同事索取：视频与网络 / Video and Network

1. 实车到底是 5 路还是 6 路摄像头，型号、分辨率、帧率、码流和镜头参数。
2. H.264/H.265、RTSP/SRT/WebRTC 等正式传输协议、URL、鉴权及硬件编码能力。
3. 每帧时间戳来源、端到端延迟目标、抖动/丢包指标和 QoS/5G 切片接口。
4. 相机内参、畸变参数和相对 `base_link` 的六自由度外参标定文件。
5. 远驾断网、视频冻结、控制链路正常但视频失效时的安全策略。

With the current implementation, software architecture, message flow, timeout
behavior, UI, and simulator-side feedback can be developed now. Exact real-wire
compatibility, real dynamics, and safety sign-off remain blocked on the items
above.
