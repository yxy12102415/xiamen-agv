#include "simple_planning_simulator/vehicle_model/sim_model_delay_steer_vel_two_axis.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
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
  double center_of_gravity_height_m, double rolling_resistance_coefficient,
  double aerodynamic_drag_coefficient, double frontal_area_m2, double air_density_kg_m3,
  double wheel_radius_m, double wheel_inertia_kgm2, int wheel_count,
  double dt, double vx_delay, double vx_time_constant,
  double drive_delay, double brake_delay, double drive_time_constant, double brake_time_constant,
  double steer_delay, double steer_time_constant, double steer_dead_band, double steer_bias,
  bool all_wheel_drive)
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
  all_wheel_drive_(all_wheel_drive),
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
  steer_bias_(steer_bias)
{
  initializeInputQueue(dt);
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

  // do not use deadzone_delta_steer (Steer IF does not exist in this model)
  updateRungeKutta(dt, delayed_input);
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
  const double requested_net_ax =
    sat(use_ax_command ? delay_input_ax : velocity_error_ax, max_drive_accel_, -max_brake_decel_);
  const double resistance_force_n = resistanceForce(vx, requested_net_ax);
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

  // Two steer inputs are interpreted as axle 1 and axle 2 steering.
  // Axles 3 and 4 are fixed for the current "12-axle steering" vehicle mode.
  const std::array<double, 4> axle_steers{{steer_f, steer_r, 0.0, 0.0}};

  double sum_x = 0.0;
  double sum_xx = 0.0;
  double sum_y = 0.0;
  double sum_xy = 0.0;
  for (size_t axle_index = 0; axle_index < axle_positions_m_.size(); ++axle_index) {
    const double axle_x = axle_positions_m_[axle_index];
    const double desired_lateral_velocity = vx * std::tan(axle_steers[axle_index]);
    sum_x += axle_x;
    sum_xx += axle_x * axle_x;
    sum_y += desired_lateral_velocity;
    sum_xy += axle_x * desired_lateral_velocity;
  }
  const double axle_count = static_cast<double>(axle_positions_m_.size());
  const double fit_denominator = axle_count * sum_xx - sum_x * sum_x;
  const double kinematic_vy =
    std::abs(fit_denominator) > 1.0e-6 ?
    (sum_y * sum_xx - sum_x * sum_xy) / fit_denominator : 0.0;
  omega_ =
    std::abs(fit_denominator) > 1.0e-6 ?
    (axle_count * sum_xy - sum_x * sum_y) / fit_denominator : 0.0;

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
    for (size_t axle_index = 0; axle_index < axle_positions_m_.size(); ++axle_index) {
      const double axle_x = axle_positions_m_[axle_index];
      const double axle_steer = axle_steers[axle_index];
      const double slip_angle =
        axle_steer - std::atan2(dynamic_vy + axle_x * dynamic_wz, vx_for_slip);
      double lateral_force_n =
        axle_cornering_stiffness_n_rad_[axle_index] * slip_angle;

      if (tire_friction_coefficient_ > 0.0) {
        const bool driven_axle = all_wheel_drive_ || axle_index >= 2;
        const double longitudinal_axle_force_n =
          driven_axle && driven_normal_load_n > 1.0 ?
          longitudinal_force_n * dynamic_normal_loads_n[axle_index] / driven_normal_load_n :
          0.0;
        const double lateral_force_limit_n = std::sqrt(std::max(
          0.0,
          std::pow(
            tire_friction_coefficient_ * dynamic_normal_loads_n[axle_index], 2.0) -
          std::pow(longitudinal_axle_force_n, 2.0)));
        lateral_force_n =
          sat(lateral_force_n, lateral_force_limit_n, -lateral_force_limit_n);
      }

      const double body_lateral_force_n = lateral_force_n * std::cos(axle_steer);
      total_lateral_force_n += body_lateral_force_n;
      total_yaw_moment_nm += axle_x * body_lateral_force_n;
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
