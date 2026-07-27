# Estimated AGV Dynamics / AGV 动力学估计

## Purpose / 用途

These parameters make the 15.0 m x 2.98 m, 23 t simulation usable before
measured vehicle data is available. They are engineering estimates, not
acceptance data for a real vehicle.

这些参数用于在实测数据到位前，使 15.0 m x 2.98 m、空载 23 t 的仿真车辆具备
可运行的负载动力学。它们是工程估计值，不能作为实车验收或安全认证数据。

## Load profiles / 负载工况

| Profile / 工况 | Payload / 载荷 | Total mass / 总质量 | CG z / 质心高度 | Yaw inertia / 偏航惯量 | Load per axle / 单轴轴荷 |
|---|---:|---:|---:|---:|---:|
| empty | 0 kg | 23,000 kg | 0.800 m | 432,364 kg m^2 | 5,750 kg |
| payload_20t | 20,000 kg | 43,000 kg | 1.877 m | 503,436 kg m^2 | 10,750 kg |
| payload_30_48t | 30,480 kg | 53,480 kg | 2.120 m | 825,019 kg m^2 | 13,370 kg |
| rated_70t | 70,000 kg | 93,000 kg | 2.543 m | 1,334,131 kg m^2 | 23,250 kg |

Equivalent suspension initialization / 等效悬架设计初值：

| Profile / 工况 | Total vertical K / 总垂向刚度 | Per-wheel K / 单轮刚度 | Per-wheel C / 单轮阻尼 |
|---|---:|---:|---:|
| empty | 1.308 MN/m | 163 kN/m | 13.0 kN s/m |
| payload_20t | 2.445 MN/m | 306 kN/m | 24.3 kN s/m |
| payload_30_48t | 3.040 MN/m | 380 kN/m | 30.2 kN s/m |
| rated_70t | 5.287 MN/m | 661 kN/m | 52.6 kN s/m |

These values assume a 1.2 Hz body natural frequency, 0.30 damping ratio and
eight equal wheel stations. They are low-confidence linear initialization
values for a future roll/pitch model, not active parameters in the current
planar model and not measured ZPMC suspension data.

以上数值按车身固有频率 1.2 Hz、阻尼比 0.30、八个轮位均分反算，仅作为后续
侧倾/俯仰模型的低置信度线性初值；当前平面模型尚未启用，也不是振华悬架实测值。

Start with a profile / 使用指定工况启动：

    ./sim.sh control vehicle_load_profile:=payload_30_48t
    ./sim.sh remote vehicle_load_profile:=rated_70t
    ./sim.sh control surface_friction_coefficient:=0.55

The default is empty. Profile files are in
src/agv_vehicle_model/config/*.param.yaml.

默认工况是 empty，参数文件位于
src/agv_vehicle_model/config/*.param.yaml。

## Estimation method / 估算方法

- The supplied vehicle sheet is authoritative for 15.0 m length, 2.98 m
  width, 23,000 kg empty mass, axle positions, 0.66/0.64 m unloaded/loaded
  wheel radius, 0.35 m/s^2 acceleration and 15 deg/s steering rate.
- A Kalmar FastCharge AGV is used only as a same-class boundary reference:
  15 m x 3 m x 2 m, 25 t vehicle weight, 70 t payload and 280 kW maximum
  output power.
- ZPMC confirms that it supplied the 18 electric AGVs used by the Xiamen
  automated terminal. Its later third-generation AGV uses eight steerable
  wheel groups with up to 100 deg steering, while ZPMC describes the earlier
  generation as linkage/hydraulic steering with about 35 deg maximum angle.
  The supplied Xiamen vehicle sheet (22.6-33 deg wheel limits) therefore takes
  priority over the later 100 deg product. This generation match is an
  engineering inference, not a ZPMC model identification.
- A ZPMC/Shanghai Jiao Tong University paper confirms that an electric
  autonomous container-transfer AGV used a parallel rubber and hydropneumatic
  suspension, but its public abstract does not expose usable stiffness,
  damping, CG or inertia values.
- The 30.48 t case represents a common maximum gross mass for a standard
  20/40 ft container. Container dimensions use 6.058/12.192 m length,
  2.438 m width and 2.591 m height.
- Payload CG is assumed centered at x=0, y=0 and at z=3.1155 m. This is the
  simulated 1.82 m deck plus half a standard container height.
- Combined CG uses the mass-weighted average. Combined yaw inertia uses
  Iz = Iz_empty + m_payload * (L^2 + W^2) / 12.
- Static axle loads are split equally because measured axle scales and real
  payload offsets are unavailable.
- Loaded wheel radius is linearly interpolated from the supplied 0.66 m empty
  value to 0.64 m at the 70 t rated payload.
- Axle cornering stiffness starts at 200 kN/rad empty and scales with
  (Fz/Fz_empty)^0.8. The exponent is an explicit engineering assumption,
  not tire test data.
- Surface friction defaults to 0.8. Values such as 0.55 for wet pavement and
  0.35 for a contaminated yard are scenario estimates, not measured limits.

- 用户提供的车辆表格优先级最高：车长 15.0 m、车宽 2.98 m、空载质量
  23,000 kg、轴位置、空载/满载轮胎半径 0.66/0.64 m、最大加速度
  0.35 m/s^2、转角速度 15 deg/s。
- Kalmar FastCharge AGV 只作为同级车辆边界参考：尺寸 15 m x 3 m x 2 m、
  自重 25 t、额定载荷 70 t、最大输出功率 280 kW。
- 振华重工官方资料确认厦门自动化码头采用其提供的 18 台电动 AGV。后续第三代
  产品采用八轮组、最大约 100 deg 转向，而官方将早期产品描述为机械连杆/液压
  转向、最大约 35 deg。用户车辆表给出的 22.6-33 deg 转角范围优先于后期 100 deg
  产品；代际对应关系属于工程判断，不代表已经确认具体振华产品型号。
- 振华重工与上海交通大学论文确认电驱自主集装箱转运 AGV 使用橡胶弹簧与油气
  弹簧并联复合悬架，但公开摘要没有可直接用于本项目的刚度、阻尼、质心和惯量数值。
- 30.48 t 工况代表常见标准 20/40 英尺集装箱最大总重；箱体尺寸采用长度
  6.058/12.192 m、宽度 2.438 m、高度 2.591 m。
- 假设载荷位于车辆中心 x=0、y=0，载荷质心 z=3.1155 m，即仿真承载面
  1.82 m 加半个标准箱高度。
- 总质心按质量加权，偏航惯量按
  Iz = Iz_empty + m_payload * (L^2 + W^2) / 12 估算。
- 因缺少四轴称重和载荷偏置数据，静态轴荷暂时四轴均分。
- 轮胎滚动半径按用户给出的空载 0.66 m 到额定 70 t 载荷 0.64 m 线性插值。
- 空载单轴侧偏刚度取 200 kN/rad，并按 (Fz/Fz_empty)^0.8 随载荷缩放；
  0.8 是体现轮胎载荷敏感性的工程假设，不是轮胎台架实测值。
- 路面附着系数默认取 0.8；湿地 0.55、污染堆场 0.35 仅用于工况模拟，均非实测边界。

## Steering and wheel-force model / 转向与逐轮受力模型

The vehicle remains a four-axle, eight-wheel vehicle with only axles 1 and 2
steering. The two controller states are axle-center equivalent angles. Each is
converted to independent left/right Ackermann angles before kinematic fitting
and tire-force calculation; axles 3 and 4 stay at zero steering.

车辆仍为四轴八轮，只有第 1、2 轴转向。控制器中的两个转角状态表示轴中心等效
转角，进入运动学和轮胎力计算前分别转换为左右轮 Ackermann 转角；第 3、4 轴固定
为零转角。

- Axle 1 center limit: `0.5229795759 rad` (`29.9645 deg`), producing
  `33.0 deg` inner and `27.4 deg` outer wheel angles.
- Axle 2 center limit: `0.4505604976 rad` (`25.8152 deg`), producing
  `30.0 deg` inner and `22.6 deg` outer wheel angles.
- Each wheel uses its own `(x, y)` velocity, slip angle, half-axle cornering
  stiffness, half-axle normal load, friction-circle limit and yaw moment arm.
- MPC uses the same eight-wheel no-slip least-squares geometry as the simulator.

上述中心转角由用户表格中的内外轮极限和暂估轮距反算。等效 Ackermann 长度
`6.1166 m`、`3.5527 m` 是拟合参数，不是转向拉杆或机械轴距实测值。轮距或真实
转向几何变化后必须重新标定。

## Implemented behavior / 已实现行为

- Eight individual wheel lateral forces across four axles, with steering on
  axles 1/2, fixed axles 3/4 and longitudinal front/rear load transfer.
- Smooth tanh tire-force saturation inside the friction circle.
- Separate drive, brake and steering transport delay plus first-order response.
- Estimated 0.02 m/s^2 drive dead zone, 0.05 m/s^2 brake dead zone and
  0.2 deg steering backlash.
- Rolling and aerodynamic resistance, rotating-wheel equivalent mass,
  280 kW drive power limit and load-specific tractive-force limits.
- Estimated nonlinear brake-pressure feedback: 3 bar take-up, 160 bar
  saturation and exponent 1.05.
- MPC prediction includes the same command dead zones and steering backlash.

- 四轴八个轮位分别计算侧偏力，第 1、2 轴转向，第 3、4 轴固定，并包含纵向前后
  轴荷转移。
- 摩擦圆内使用 tanh 连续轮胎力饱和，避免硬截断跳变。
- 驱动、制动、转向分别具有传输延迟和一阶响应。
- 暂估驱动死区 0.02 m/s^2、制动死区 0.05 m/s^2、转向回差半宽 0.2 deg。
- 包含滚阻、风阻、车轮转动等效质量、280 kW 功率限制和分工况牵引力限制。
- 制动压力反馈暂按 3 bar 建压死区、160 bar 饱和、1.05 指数曲线估计。
- MPC 预测模型同步加入指令死区和转向回差。

## Data still needed / 仍需实测数据

For one-to-one reproduction, replace the estimates with:

- empty and loaded CG x/y/z plus four axle scale readings;
- measured yaw inertia or a validated CAD mass model;
- tire make, size, pressure, vertical-load versus cornering-force data and
  measured dry/wet yard friction;
- drive motor torque-speed/power curve and regenerative-braking limits;
- steering, drive and brake step-response logs, dead zones and hysteresis;
- brake pressure-to-wheel-force curve;
- suspension/roll stiffness and damping if roll and pitch must be reproduced;
- payload location, lock geometry and allowable offset cases.

要做到一比一复刻，仍需用以下实测值替换估计值：空载/负载三维质心和四轴称重、
偏航惯量或可信 CAD 质量模型、轮胎型号/胎压/侧偏试验、干湿堆场附着系数、电机
转矩转速曲线、再生制动边界、转向/驱动/制动阶跃响应、死区与回差、制动压力到
轮端制动力曲线；若要复刻侧倾和俯仰，还需悬架刚度、阻尼和侧倾中心数据。

## Regression tests / 回归测试

Run `colcon test --packages-select agv_vehicle_model` and
`PYTHONPATH=src/agv_mpc_controller python3 -m pytest -q src/agv_mpc_controller/test`
after changing wheel geometry, steering limits, tire parameters or MPC prediction.
The tests cover inner/outer angle targets, fixed rear axles, left/right mirror
symmetry, straight-line zero yaw response and per-wheel friction-circle limits.

修改轮距、转角限制、轮胎参数或 MPC 预测后，应运行上述回归测试。当前测试覆盖
内外轮目标角、后两轴固定、左右转向镜像、直行零横摆和逐轮摩擦圆限幅。它们是
软件回归测试，不能替代实车定圆、S 弯和制动标定。

## References / 参考资料

- ZPMC Xiamen automated terminal project (18 electric AGVs):
  https://cn.zpmc.com/content/6765957b006017c80000017d
- ZPMC third-generation AGV steering and powertrain description:
  https://cn.zpmc.com/content/6765957d006017c8000002bd
- ZPMC/SJTU autonomous container-transfer AGV suspension paper:
  https://www.qk.sjtu.edu.cn/dst/CN/10.3969/j.issn.1006-8244.a0899
- Kalmar FastCharge AGV data sheet:
  https://www.kalmarglobal.com/49c56f/globalassets/media/204161/204161_AGV-data-sheet-Web-singles.pdf
- Hapag-Lloyd container packing guide (30,480 kg common gross limit):
  https://www.hapag-lloyd.com/content/dam/website/downloads/press_and_media/publications/Container_Packing_Broschuere_engl.pdf
- UNECE intermodal loading-unit dimensions:
  https://unece.org/sites/default/files/2026-07/ECE-TRANS-WP.24-2026-08E.pdf
- SAE tire vertical-load and cornering-stiffness study:
  https://doi.org/10.4271/10-03-01-0002
