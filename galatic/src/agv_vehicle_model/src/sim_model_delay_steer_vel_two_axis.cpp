#include "simple_planning_simulator/vehicle_model/sim_model_delay_steer_vel_two_axis.hpp"

#include "agv_vehicle_model/steering_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>

namespace autoware::simulator::simple_planning_simulator
{

SimModelDelaySteerVelTwoAxis::SimModelDelaySteerVelTwoAxis(
  double vx_lim, double steer_lim_front, double steer_lim_rear, double max_drive_accel,
  double max_brake_decel, double steer_rate_lim, double wheelbase_f, double wheelbase_r, double vehicle_mass_kg,
  double yaw_inertia_kgm2, double front_cornering_stiffness_n_rad,
  double rear_cornering_stiffness_n_rad, double front_normal_load_n, double rear_normal_load_n,
  const std::array<double, 4> & axle_positions_m,
  const std::array<double, 4> & axle_cornering_stiffness_n_rad,
  const std::array<double, 4> & axle_normal_loads_n, double tire_friction_coefficient,
  double wheel_tread_m, double ackermann_axle1_wheelbase_m,
  double ackermann_axle2_wheelbase_m, bool enable_ackermann_wheel_angles,
  double center_of_gravity_height_m, double rolling_resistance_coefficient,
  double aerodynamic_drag_coefficient, double frontal_area_m2, double air_density_kg_m3,
  double wheel_radius_m, double wheel_inertia_kgm2, int wheel_count,
  double dt, double vx_delay, double vx_time_constant,
  double drive_delay, double brake_delay, double drive_time_constant, double brake_time_constant,
  double drive_accel_dead_band, double brake_decel_dead_band, double max_drive_force_n,
  double max_drive_power_kw, double steer_delay, double steer_time_constant,
  double steer_dead_band, double steer_backlash, double steer_bias, bool all_wheel_drive,
  bool use_smooth_tire_saturation)
: SimModelInterface(9 /* dim x */, 5 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  max_drive_accel_(max_drive_accel),
  max_brake_decel_(max_brake_decel),
  steer_lim_front_(steer_lim_front),
  steer_lim_rear_(steer_lim_rear),
  steer_rate_lim_(steer_rate_lim),
  wheelbase_f(wheelbase_f),
  wheelbase_r(wheelbase_r),
  vehicle_mass_kg_(vehicle_mass_kg),
  yaw_inertia_kgm2_(yaw_inertia_kgm2),
  front_cornering_stiffness_n_rad_(front_cornering_stiffness_n_rad),
  rear_cornering_stiffness_n_rad_(rear_cornering_stiffness_n_rad),
  front_normal_load_n_(front_normal_load_n),
  rear_normal_load_n_(rear_normal_load_n),
  axle_positions_m_(axle_positions_m),
  axle_cornering_stiffness_n_rad_(axle_cornering_stiffness_n_rad),
  axle_normal_loads_n_(axle_normal_loads_n),
  tire_friction_coefficient_(tire_friction_coefficient),
  wheel_tread_m_(wheel_tread_m),
  ackermann_axle1_wheelbase_m_(ackermann_axle1_wheelbase_m),
  ackermann_axle2_wheelbase_m_(ackermann_axle2_wheelbase_m),
  enable_ackermann_wheel_angles_(enable_ackermann_wheel_angles),
  center_of_gravity_height_m_(center_of_gravity_height_m),
  rolling_resistance_coefficient_(std::max(0.0, rolling_resistance_coefficient)),
  aerodynamic_drag_coefficient_(std::max(0.0, aerodynamic_drag_coefficient)),
  frontal_area_m2_(std::max(0.0, frontal_area_m2)),
  air_density_kg_m3_(std::max(0.0, air_density_kg_m3)),
  wheel_radius_m_(std::max(0.01, wheel_radius_m)),
  wheel_inertia_kgm2_(std::max(0.0, wheel_inertia_kgm2)),
  wheel_count_(std::max(0, wheel_count)),
  effective_longitudinal_mass_kg_(
    vehicle_mass_kg +
    std::max(0, wheel_count) * std::max(0.0, wheel_inertia_kgm2) /
    std::pow(std::max(0.01, wheel_radius_m), 2.0)),
  drive_delay_(drive_delay),
  brake_delay_(brake_delay),
  drive_time_constant_(std::max(drive_time_constant, MIN_TIME_CONSTANT)),
  brake_time_constant_(std::max(brake_time_constant, MIN_TIME_CONSTANT)),
  drive_accel_dead_band_(std::max(0.0, drive_accel_dead_band)),
  brake_decel_dead_band_(std::max(0.0, brake_decel_dead_band)),
  max_drive_force_n_(std::max(0.0, max_drive_force_n)),
  max_drive_power_w_(std::max(0.0, max_drive_power_kw) * 1000.0),
  all_wheel_drive_(all_wheel_drive),
  use_smooth_tire_saturation_(use_smooth_tire_saturation),
  enable_lateral_dynamics_(
    vehicle_mass_kg > 1.0 && yaw_inertia_kgm2 > 1.0 &&
    std::all_of(
      axle_cornering_stiffness_n_rad.begin(), axle_cornering_stiffness_n_rad.end(),
      [](const double stiffness) { return stiffness > 1.0; }) &&
    std::all_of(
      axle_normal_loads_n.begin(), axle_normal_loads_n.end(),
      [](const double load) { return load > 1.0; })),
  vx_delay_(vx_delay),
  vx_time_constant_(std::max(vx_time_constant, MIN_TIME_CONSTANT)),
  steer_delay_(steer_delay),
  steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT)),
  steer_dead_band_(steer_dead_band),
  steer_backlash_(std::max(0.0, steer_backlash)),
  steer_bias_(steer_bias)
{
  initializeInputQueue(dt);
}

double SimModelDelaySteerVelTwoAxis::applySteeringBacklash(
  const double command, double & effective_command) const
{
  if (steer_backlash_ <= 0.0) {
    effective_command = command;
  } else if (command > effective_command + steer_backlash_) {
    effective_command = command - steer_backlash_;
  } else if (command < effective_command - steer_backlash_) {
    effective_command = command + steer_backlash_;
  }
  return effective_command;
}

double SimModelDelaySteerVelTwoAxis::resistanceForce(
  const double vx, const double requested_accel) const
{
  double direction = 0.0;
  if (std::abs(vx) > 0.03) {
    direction = std::copysign(1.0, vx);
  } else if (std::abs(requested_accel) > 1.0e-4) {
    direction = std::copysign(1.0, requested_accel);
  }
  if (direction == 0.0) {
    return 0.0;
  }

  constexpr double gravity = 9.81;
  const double rolling_force =
    rolling_resistance_coefficient_ * vehicle_mass_kg_ * gravity;
  const double aerodynamic_force =
    0.5 * air_density_kg_m3_ * aerodynamic_drag_coefficient_ *
    frontal_area_m2_ * vx * vx;
  return direction * (rolling_force + aerodynamic_force);
}

double SimModelDelaySteerVelTwoAxis::getX()
{
  return state_(IDX::X);
}
double SimModelDelaySteerVelTwoAxis::getY()
{
  return state_(IDX::Y);
}
double SimModelDelaySteerVelTwoAxis::getYaw()
{
  return state_(IDX::YAW);
}
double SimModelDelaySteerVelTwoAxis::getVx()
{
  return state_(IDX::VX);
}
double SimModelDelaySteerVelTwoAxis::getVy()
{
  return enable_lateral_dynamics_ ? state_(IDX::VY) : beta_ * state_(IDX::VX);
}
double SimModelDelaySteerVelTwoAxis::getAx()
{
  return state_(IDX::AX) -
         resistanceForce(state_(IDX::VX), state_(IDX::AX)) /
         effective_longitudinal_mass_kg_;
}
double SimModelDelaySteerVelTwoAxis::getWz()
{
  return enable_lateral_dynamics_ ? state_(IDX::WZ) : omega_;
}
double SimModelDelaySteerVelTwoAxis::getSteer()
{
  return state_(IDX::STEER_F) + steer_bias_;
}

double SimModelDelaySteerVelTwoAxis::getSteerF()
{
  return state_(IDX::STEER_F) + steer_bias_;
}

double SimModelDelaySteerVelTwoAxis::getSteerR()
{
  return state_(IDX::STEER_R) + steer_bias_;
}


void SimModelDelaySteerVelTwoAxis::update(const double & dt)
{
  const double vx_before_update = state_(IDX::VX);
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);

  vx_input_queue_.push_back(input_(IDX_U::VX_DES));
  delayed_input(IDX_U::VX_DES) = vx_input_queue_.front();
  vx_input_queue_.pop_front();

  drive_ax_input_queue_.push_back(input_(IDX_U::AX_DES));
  brake_ax_input_queue_.push_back(input_(IDX_U::AX_DES));
  drive_use_ax_input_queue_.push_back(input_(IDX_U::USE_AX_DES));
  brake_use_ax_input_queue_.push_back(input_(IDX_U::USE_AX_DES));
  const bool braking_command = input_(IDX_U::AX_DES) < 0.0;
  auto & ax_queue = braking_command ? brake_ax_input_queue_ : drive_ax_input_queue_;
  auto & use_ax_queue =
    braking_command ? brake_use_ax_input_queue_ : drive_use_ax_input_queue_;
  delayed_input(IDX_U::AX_DES) = ax_queue.front();
  delayed_input(IDX_U::USE_AX_DES) = use_ax_queue.front();
  ax_queue.pop_front();
  use_ax_queue.pop_front();
  if (braking_command) {
    drive_ax_input_queue_.pop_front();
    drive_use_ax_input_queue_.pop_front();
  } else {
    brake_ax_input_queue_.pop_front();
    brake_use_ax_input_queue_.pop_front();
  }

  steer_f_input_queue_.push_back(input_(IDX_U::STEER_F_DES));
  delayed_input(IDX_U::STEER_F_DES) = steer_f_input_queue_.front();
  steer_f_input_queue_.pop_front();

  steer_r_input_queue_.push_back(input_(IDX_U::STEER_R_DES));
  delayed_input(IDX_U::STEER_R_DES) = steer_r_input_queue_.front();
  steer_r_input_queue_.pop_front();

  delayed_input(IDX_U::STEER_F_DES) = applySteeringBacklash(
    delayed_input(IDX_U::STEER_F_DES), effective_steer_cmd_f_);
  delayed_input(IDX_U::STEER_R_DES) = applySteeringBacklash(
    delayed_input(IDX_U::STEER_R_DES), effective_steer_cmd_r_);

  // do not use deadzone_delta_steer (Steer IF does not exist in this model)
  updateRungeKutta(dt, delayed_input);
  const bool zero_velocity_target = std::abs(delayed_input(IDX_U::VX_DES)) < 1.0e-4;
  const bool crossed_zero =
    (vx_before_update >= 0.0 && state_(IDX::VX) < 0.0) ||
    (vx_before_update <= 0.0 && state_(IDX::VX) > 0.0);
  if (zero_velocity_target && crossed_zero) {
    state_(IDX::VX) = 0.0;
    state_(IDX::AX) = 0.0;
  }
  prev_vx_ = state_(IDX::VX);
}

void SimModelDelaySteerVelTwoAxis::initializeInputQueue(const double & dt)
{
  size_t vx_input_queue_size = static_cast<size_t>(round(vx_delay_ / dt));
  for (size_t i = 0; i < vx_input_queue_size; i++) {
    vx_input_queue_.push_back(0.0);
  }

  const size_t drive_queue_size = static_cast<size_t>(round(drive_delay_ / dt));
  for (size_t i = 0; i < drive_queue_size; i++) {
    drive_ax_input_queue_.push_back(0.0);
    drive_use_ax_input_queue_.push_back(0.0);
  }

  const size_t brake_queue_size = static_cast<size_t>(round(brake_delay_ / dt));
  for (size_t i = 0; i < brake_queue_size; i++) {
    brake_ax_input_queue_.push_back(0.0);
    brake_use_ax_input_queue_.push_back(0.0);
  }

  size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  for (size_t i = 0; i < steer_input_queue_size; i++) {
    steer_f_input_queue_.push_back(0.0);
    steer_r_input_queue_.push_back(0.0);
  }
}

Eigen::VectorXd SimModelDelaySteerVelTwoAxis::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  auto sat = [](double val, double u, double l) { return std::max(std::min(val, u), l); };

  const double vx = sat(state(IDX::VX), vx_lim_, -vx_lim_);
  const double dynamic_vy = state(IDX::VY);
  const double dynamic_wz = state(IDX::WZ);
  const double dynamic_ax = state(IDX::AX);
  const double steer_f = sat(state(IDX::STEER_F), steer_lim_front_, -steer_lim_front_);
  const double steer_r = sat(state(IDX::STEER_R), steer_lim_rear_, -steer_lim_rear_);
  const double yaw = state(IDX::YAW);

  const double delay_input_vx = input(IDX_U::VX_DES);
  const double delay_input_ax = input(IDX_U::AX_DES);
  const bool use_ax_command = input(IDX_U::USE_AX_DES) > 0.5;
  const double delay_input_steer_f = input(IDX_U::STEER_F_DES);
  const double delay_input_steer_r = input(IDX_U::STEER_R_DES);

  const double delay_vx_des = sat(delay_input_vx, vx_lim_, -vx_lim_);
  const double velocity_error_ax =
    sat((delay_vx_des - vx) / vx_time_constant_, max_drive_accel_, -max_brake_decel_);
  double requested_net_ax =
    sat(use_ax_command ? delay_input_ax : velocity_error_ax, max_drive_accel_, -max_brake_decel_);
  if (requested_net_ax > 0.0 && requested_net_ax < drive_accel_dead_band_) {
    requested_net_ax = 0.0;
  } else if (requested_net_ax < 0.0 && -requested_net_ax < brake_decel_dead_band_) {
    requested_net_ax = 0.0;
  }
  const double resistance_force_n = resistanceForce(vx, requested_net_ax);
  if (requested_net_ax > 0.0 && (max_drive_force_n_ > 0.0 || max_drive_power_w_ > 0.0)) {
    double available_drive_force_n = max_drive_force_n_ > 0.0 ?
      max_drive_force_n_ : std::numeric_limits<double>::max();
    if (max_drive_power_w_ > 0.0) {
      available_drive_force_n = std::min(
        available_drive_force_n, max_drive_power_w_ / std::max(std::abs(vx), 0.5));
    }
    const double available_net_ax = std::max(
      0.0, (available_drive_force_n - std::abs(resistance_force_n)) /
      effective_longitudinal_mass_kg_);
    requested_net_ax = std::min(requested_net_ax, available_net_ax);
  }
  const double ax_des =
    requested_net_ax + resistance_force_n / effective_longitudinal_mass_kg_;
  const double longitudinal_time_constant =
    ax_des >= dynamic_ax ? drive_time_constant_ : brake_time_constant_;
  const double ax_rate =
    (ax_des - dynamic_ax) / longitudinal_time_constant;
  double vx_rate =
    dynamic_ax - resistanceForce(vx, dynamic_ax) / effective_longitudinal_mass_kg_;
  vx_rate = sat(vx_rate, max_drive_accel_, -max_brake_decel_);
  if (
    std::abs(vx) < 0.03 && std::abs(dynamic_ax) <
    rolling_resistance_coefficient_ * vehicle_mass_kg_ * 9.81 /
    effective_longitudinal_mass_kg_) {
    vx_rate = 0.0;
  }

  const double delay_steer_f_des = sat(delay_input_steer_f, steer_lim_front_, -steer_lim_front_);
  const double steer_diff_f = steer_f + steer_bias_ - delay_steer_f_des;
  const double delay_steer_r_des = sat(delay_input_steer_r, steer_lim_rear_, -steer_lim_rear_);
  const double steer_diff_r = steer_r + steer_bias_ - delay_steer_r_des;

  const auto dead_band = [&](const auto &steer_diff){
    if (steer_diff > steer_dead_band_) {
      return steer_diff - steer_dead_band_;
    } else if (steer_diff < -steer_dead_band_) {
      return steer_diff + steer_dead_band_;
    } else {
      return 0.0;
    }
  };

  const double steer_diff_f_with_deadband = dead_band(steer_diff_f);
  const double steer_diff_r_with_deadband = dead_band(steer_diff_r);
  const double steer_rate_f = sat(-steer_diff_f_with_deadband / steer_time_constant_, steer_rate_lim_, -steer_rate_lim_);
  const double steer_rate_r = sat(-steer_diff_r_with_deadband / steer_time_constant_, steer_rate_lim_, -steer_rate_lim_);

  // The two states are axle-center steering angles for axles 1 and 2. Each axle is
  // expanded to left/right Ackermann angles; axles 3 and 4 remain fixed.
  const auto wheel_steers = agv_vehicle_model::frontTwoAxleWheelSteers(
    steer_f, steer_r, ackermann_axle1_wheelbase_m_, ackermann_axle2_wheelbase_m_,
    wheel_tread_m_, enable_ackermann_wheel_angles_);
  const auto kinematic_velocity = agv_vehicle_model::fitWheelKinematics(
    vx, axle_positions_m_, wheel_tread_m_, wheel_steers);
  const double kinematic_vy = kinematic_velocity.lateral;
  omega_ = kinematic_velocity.yaw_rate;

  const double vx_for_beta = std::copysign(std::max(std::abs(vx), 0.05), vx == 0.0 ? 1.0 : vx);
  beta_ = std::atan2(kinematic_vy, vx_for_beta);
  v_ = std::hypot(vx, kinematic_vy);
  kappa_ = omega_ / std::max(std::abs(vx), 0.05);

  const bool use_lateral_dynamics = enable_lateral_dynamics_ && std::abs(vx) > 0.3;
  double vy = kinematic_vy;
  double wz = omega_;
  double vy_rate = 0.0;
  double wz_rate = 0.0;

  if (use_lateral_dynamics) {
    const double vx_for_slip = std::copysign(std::max(std::abs(vx), 0.5), vx);
    std::array<double, 4> dynamic_normal_loads_n = axle_normal_loads_n_;
    const double front_group_x =
      0.5 * (axle_positions_m_[0] + axle_positions_m_[1]);
    const double rear_group_x =
      0.5 * (axle_positions_m_[2] + axle_positions_m_[3]);
    const double effective_wheelbase = std::max(0.1, front_group_x - rear_group_x);
    const double longitudinal_load_transfer_n =
      vehicle_mass_kg_ * vx_rate * center_of_gravity_height_m_ /
      effective_wheelbase;
    const double front_static_load_n =
      axle_normal_loads_n_[0] + axle_normal_loads_n_[1];
    const double rear_static_load_n =
      axle_normal_loads_n_[2] + axle_normal_loads_n_[3];
    const double front_dynamic_load_n =
      std::max(1.0, front_static_load_n - longitudinal_load_transfer_n);
    const double rear_dynamic_load_n =
      std::max(1.0, rear_static_load_n + longitudinal_load_transfer_n);
    dynamic_normal_loads_n[0] =
      front_dynamic_load_n * axle_normal_loads_n_[0] / front_static_load_n;
    dynamic_normal_loads_n[1] =
      front_dynamic_load_n * axle_normal_loads_n_[1] / front_static_load_n;
    dynamic_normal_loads_n[2] =
      rear_dynamic_load_n * axle_normal_loads_n_[2] / rear_static_load_n;
    dynamic_normal_loads_n[3] =
      rear_dynamic_load_n * axle_normal_loads_n_[3] / rear_static_load_n;
    const double driven_normal_load_n = all_wheel_drive_ ?
      std::accumulate(dynamic_normal_loads_n.begin(), dynamic_normal_loads_n.end(), 0.0) :
      dynamic_normal_loads_n[2] + dynamic_normal_loads_n[3];
    const double longitudinal_force_n = tire_friction_coefficient_ > 0.0 ?
      std::min(
        std::abs(effective_longitudinal_mass_kg_ * dynamic_ax),
        tire_friction_coefficient_ * driven_normal_load_n) :
      std::abs(effective_longitudinal_mass_kg_ * dynamic_ax);

    double total_lateral_force_n = 0.0;
    double total_yaw_moment_nm = 0.0;
    const double half_tread = 0.5 * wheel_tread_m_;
    for (size_t wheel_index = 0; wheel_index < wheel_steers.size(); ++wheel_index) {
      const size_t axle_index = wheel_index / 2;
      const double wheel_x = axle_positions_m_[axle_index];
      const double wheel_y = wheel_index % 2 == 0 ? half_tread : -half_tread;
      const double wheel_normal_load_n = 0.5 * dynamic_normal_loads_n[axle_index];
      const bool driven_wheel = all_wheel_drive_ || axle_index >= 2;
      const double longitudinal_wheel_force_n =
        driven_wheel && driven_normal_load_n > 1.0 ?
        longitudinal_force_n * wheel_normal_load_n / driven_normal_load_n : 0.0;
      const auto wheel_force = agv_vehicle_model::calculateWheelLateralForce(
        wheel_x, wheel_y, wheel_steers[wheel_index], vx_for_slip, dynamic_vy,
        dynamic_wz, 0.5 * axle_cornering_stiffness_n_rad_[axle_index],
        wheel_normal_load_n, tire_friction_coefficient_, longitudinal_wheel_force_n,
        use_smooth_tire_saturation_);
      total_lateral_force_n += wheel_force.body_lateral_force;
      total_yaw_moment_nm += wheel_force.yaw_moment;
    }

    vy = dynamic_vy;
    wz = dynamic_wz;
    vy_rate = total_lateral_force_n / vehicle_mass_kg_ - vx * dynamic_wz;
    wz_rate = total_yaw_moment_nm / yaw_inertia_kgm2_;
    omega_ = wz;
  } else {
    constexpr double lateral_tracking_time_constant = 0.2;
    vy_rate = (kinematic_vy - dynamic_vy) / lateral_tracking_time_constant;
    wz_rate = (omega_ - dynamic_wz) / lateral_tracking_time_constant;
  }

  // Debug logging for 4WS kinematics (commented out - high frequency)
  // static int log_counter = 0;
  // if (++log_counter >= 30) {  // Log every ~1 second at 30Hz
  //   log_counter = 0;
  //   // Detect 4WS mode: if front and rear have same sign = crab, opposite sign = q-turn
  //   bool is_4ws_active = std::abs(steer_f) > 0.01 || std::abs(steer_r) > 0.01;
  //   if (is_4ws_active) {
  //     const char* mode = "NORMAL";
  //     if (std::abs(steer_r) > 0.01) {
  //       if (steer_f * steer_r > 0) {
  //         mode = "CRAB/OBLIQUE";  // Same sign = crab motion
  //       } else {
  //         mode = "Q-TURN";  // Opposite sign = reduced turning radius
  //       }
  //     }
  //     std::cerr << "[SimModel2Axis] " << mode
  //               << ": steer_f=" << steer_f * 180.0 / M_PI << " deg"
  //               << ", steer_r=" << steer_r * 180.0 / M_PI << " deg"
  //               << ", beta=" << beta_ * 180.0 / M_PI << " deg"
  //               << ", kappa=" << kappa_
  //               << ", omega=" << omega_ << " rad/s"
  //               << ", vx=" << vx << " m/s"
  //               << ", vy=" << vy << " m/s\n";
  //   }
  // }

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vx * cos(yaw) - vy * sin(yaw);
  d_state(IDX::Y) = vx * sin(yaw) + vy * cos(yaw);
  d_state(IDX::YAW) = wz;
  d_state(IDX::VX) = vx_rate;
  d_state(IDX::VY) = vy_rate;
  d_state(IDX::WZ) = wz_rate;
  d_state(IDX::STEER_F) = steer_rate_f;
  d_state(IDX::STEER_R) = steer_rate_r;
  d_state(IDX::AX) = ax_rate;

  return d_state;
}

}  // namespace autoware::simulator::simple_planning_simulator
