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

Assumptions: 1.2 Hz body natural frequency, 0.30 damping ratio and eight equal
wheel stations. These are low-confidence initial values for a future roll/pitch
model, not measured ZPMC data. / 假设车身固有频率 1.2 Hz、阻尼比 0.30、八轮位
均分；仅作为后续侧倾/俯仰模型初值，不是振华实测值。

Use a profile / 使用指定工况：

    ./sim.sh control vehicle_load_profile:=payload_30_48t
    ./sim.sh control surface_friction_coefficient:=0.55

The default is empty. Profile files are in
src/agv_vehicle_model/config/*.param.yaml.

默认工况是 empty，参数文件位于
src/agv_vehicle_model/config/*.param.yaml。

## Estimation and implementation / 估算与实现

- Supplied values remain authoritative for geometry, empty mass, axle positions,
  wheel radii, acceleration and steering-rate limits.
- Same-class boundary reference: 15 m x 3 m x 2 m, 25 t vehicle, 70 t payload
  and 280 kW maximum output.
- ZPMC confirms that it supplied the 18 electric AGVs at the Xiamen automated
  terminal. Its later third-generation product has eight steerable wheel groups
  and up to 100 deg steering, while the earlier generation is described as
  linkage/hydraulic steering with about 35 deg maximum angle. The supplied
  22.6-33 deg Xiamen vehicle limits take priority; the generation match remains
  an engineering inference.
- Payload CG is centered at x=0, y=0 and estimated at z=3.1155 m.
- Combined yaw inertia uses
  Iz = Iz_empty + m_payload * (L^2 + W^2) / 12.
- Axle loads are equal until scale data is available.
- Wheel radius is linearly interpolated from 0.66 m empty to 0.64 m at 70 t.
- Cornering stiffness scales with (Fz/Fz_empty)^0.8; the exponent is an
  engineering assumption, not measured tire data.
- Surface friction defaults to 0.8. Wet 0.55 and contaminated-yard 0.35 are
  scenario estimates rather than measured limits.

- 用户提供的几何、空载质量、轴位置、轮胎半径、加速度和转角速度保持最高优先级。
- 同级车辆边界参考为 15 m x 3 m x 2 m、自重 25 t、载荷 70 t、功率 280 kW。
- 振华官方确认厦门码头采用其 18 台电动 AGV。后续第三代产品为八轮组、最大约
  100 deg 转向，早期产品约 35 deg；本项目仍以用户车辆表 22.6-33 deg 为准，具体
  代际对应关系属于工程判断。
- 假设载荷位于 x=0、y=0，载荷质心高度暂估为 z=3.1155 m。
- 偏航惯量使用 Iz = Iz_empty + m_payload * (L^2 + W^2) / 12。
- 在取得四轴称重前暂时平均分配轴荷。
- 轮胎滚动半径从空载 0.66 m 到额定 70 t 载荷 0.64 m 线性插值。
- 侧偏刚度按 (Fz/Fz_empty)^0.8 缩放，指数为工程假设而非轮胎实测。
- 路面附着系数默认 0.8；湿地 0.55、污染堆场 0.35 仅为工况估计。

## Steering and wheel-force model / 转向与逐轮受力模型

The vehicle remains a four-axle, eight-wheel vehicle with only axles 1 and 2
steering. Controller states are axle-center equivalent angles and are converted
to independent left/right Ackermann angles before the eight-wheel kinematic fit
and tire-force calculation. Axles 3 and 4 remain fixed.

车辆仍为四轴八轮，只有第 1、2 轴转向。两个控制状态是轴中心等效转角，进入八轮
运动学拟合和轮胎力计算前转换为左右轮 Ackermann 转角，第 3、4 轴保持固定。

- Axle 1 center limit `0.5229795759 rad` produces `33.0/27.4 deg` inner/outer.
- Axle 2 center limit `0.4505604976 rad` produces `30.0/22.6 deg` inner/outer.
- Every wheel has its own position velocity, slip angle, normal load,
  friction-circle limit and yaw moment arm.
- MPC and simulator use the same eight-wheel no-slip geometry.

等效 Ackermann 长度 `6.1166 m`、`3.5527 m` 由用户表格转角和暂估轮距反算，
属于拟合参数，不是机械尺寸；取得真实轮距和转向机构数据后必须重新标定。

Implemented: load-dependent mass/CG/inertia/axle loads/tire stiffness, smooth
tire saturation, eight individual wheel forces, actuator delays, drive/brake dead zones, steering backlash,
rolling/aero resistance, wheel inertia, drive force/power limits and nonlinear
brake-pressure feedback.

已实现：分工况质量、质心、惯量、轴荷、逐轮轮胎力，连续轮胎饱和，执行器延迟，
驱动/制动死区，转向回差，滚阻/风阻，车轮惯量，牵引力/功率限制和非线性制动
压力反馈。

Still required for one-to-one reproduction: axle scale data, CG and yaw-inertia
measurement/CAD validation, tire test data and yard friction, motor torque-speed
curve, actuator step-response logs, brake pressure-force curve, payload offsets,
and suspension stiffness/damping for roll and pitch.

一比一复刻仍需：四轴称重、质心与偏航惯量实测或 CAD 校核、轮胎试验和场地摩擦、
电机转矩转速曲线、执行器阶跃日志、制动压力-制动力曲线、载荷偏置，以及用于侧倾/
俯仰的悬架刚度和阻尼。

## Regression tests / 回归测试

Run `colcon test --packages-select agv_vehicle_model` and
`PYTHONPATH=src/agv_mpc_controller python3 -m pytest -q src/agv_mpc_controller/test`
after changing steering geometry, tire parameters or MPC prediction. Tests cover
inner/outer angle targets, fixed rear axles, turn symmetry, straight response
and the per-wheel friction circle. These software tests do not replace real
constant-radius, S-turn or brake calibration.

修改转向几何、轮胎参数或 MPC 后运行上述测试；软件回归不能替代实车定圆、S 弯
和制动标定。

## References / 参考资料

- ZPMC Xiamen automated terminal project:
  https://cn.zpmc.com/content/6765957b006017c80000017d
- ZPMC third-generation AGV description:
  https://cn.zpmc.com/content/6765957d006017c8000002bd
- ZPMC/SJTU container-transfer AGV suspension paper:
  https://www.qk.sjtu.edu.cn/dst/CN/10.3969/j.issn.1006-8244.a0899
- Kalmar FastCharge AGV data sheet:
  https://www.kalmarglobal.com/49c56f/globalassets/media/204161/204161_AGV-data-sheet-Web-singles.pdf
- Hapag-Lloyd container packing guide:
  https://www.hapag-lloyd.com/content/dam/website/downloads/press_and_media/publications/Container_Packing_Broschuere_engl.pdf
- UNECE intermodal loading-unit dimensions:
  https://unece.org/sites/default/files/2026-07/ECE-TRANS-WP.24-2026-08E.pdf
- SAE tire load/cornering-stiffness study:
  https://doi.org/10.4271/10-03-01-0002
