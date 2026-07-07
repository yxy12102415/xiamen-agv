#ifndef SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_DELAY_STEER_VEL_TWO_AXIS_HPP_
#define SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_DELAY_STEER_VEL_TWO_AXIS_HPP_

#include "autoware/simple_planning_simulator/vehicle_model/sim_model_interface.hpp"

#include <Eigen/Core>
#include <Eigen/LU>

#include <array>
#include <deque>

namespace autoware::simulator::simple_planning_simulator
{

class SimModelDelaySteerVelTwoAxis : public SimModelInterface
{
public:
  /**
   * @brief constructor
   * @param [in] vx_lim velocity limit [m/s]
   * @param [in] steer_lim_front front steering limit [rad]
   * @param [in] steer_lim_rear rear steering limit [rad]
   * @param [in] vx_rate_lim acceleration limit [m/ss]
   * @param [in] steer_rate_lim steering angular velocity limit [rad/ss]
   * @param [in] wheelbase vehicle wheelbase length [m]
   * @param [in] dt delta time information to set input buffer for delay
   * @param [in] vx_delay time delay for velocity command [s]
   * @param [in] vx_time_constant time constant for 1D model of velocity dynamics
   * @param [in] steer_delay time delay for steering command [s]
   * @param [in] steer_time_constant time constant for 1D model of steering dynamics
   * @param [in] steer_dead_band dead band for steering angle [rad]
   * @param [in] steer_bias steering bias [rad]
   */
  SimModelDelaySteerVelTwoAxis(
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
    bool all_wheel_drive);

  /**
   * @brief destructor
   */
  ~SimModelDelaySteerVelTwoAxis() = default;

  /**
   * @brief get vehicle position x
   */
  double getX() override;

  /**
   * @brief get vehicle position y
   */
  double getY() override;

  /**
   * @brief get vehicle angle yaw
   */
  double getYaw() override;

  /**
   * @brief get vehicle longitudinal velocity
   */
  double getVx() override;

  /**
   * @brief get vehicle lateral velocity
   */
  double getVy() override;

  /**
   * @brief get vehicle longitudinal acceleration
   */
  double getAx() override;

  /**
   * @brief get vehicle angular-velocity wz
   */
  double getWz() override;

  /**
   * @brief get vehicle steering angle
   */
  double getSteer() override;
  double getSteerF();
  double getSteerR();

  /**
   * @brief update vehicle states
   * @param [in] dt delta time [s]
   */
  void update(const double & dt) override;

  /**
   * @brief calculate derivative of states with delay steering model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  Eigen::VectorXd calcModel(const Eigen::VectorXd & state, const Eigen::VectorXd & input) override;

private:
  const double MIN_TIME_CONSTANT;  //!< @brief minimum time constant

  enum IDX {
    X = 0,
    Y,
    YAW,
    VX,
    VY,
    WZ,
    STEER_F,
    STEER_R,
    AX,
  };
  enum IDX_U {
    VX_DES = 0,
    AX_DES,
    USE_AX_DES,
    STEER_F_DES,
    STEER_R_DES,
  };

  const double vx_lim_;          //!< @brief velocity limit
  const double max_drive_accel_;     //!< @brief acceleration limit
  const double max_brake_decel_;     //!< @brief deceleration limit
  const double steer_lim_front_;  //!< @brief front steering limit [rad]
  const double steer_lim_rear_;   //!< @brief rear steering limit [rad]
  const double steer_rate_lim_;  //!< @brief steering angular velocity limit [rad/s]
  const double wheelbase_f;      //!< @brief vehicle wheelbase length [m]
  const double wheelbase_r;
  const double vehicle_mass_kg_;
  const double yaw_inertia_kgm2_;
  const double front_cornering_stiffness_n_rad_;
  const double rear_cornering_stiffness_n_rad_;
  const double front_normal_load_n_;
  const double rear_normal_load_n_;
  const std::array<double, 4> axle_positions_m_;
  const std::array<double, 4> axle_cornering_stiffness_n_rad_;
  const std::array<double, 4> axle_normal_loads_n_;
  const double tire_friction_coefficient_;
  const double center_of_gravity_height_m_;
  const double rolling_resistance_coefficient_;
  const double aerodynamic_drag_coefficient_;
  const double frontal_area_m2_;
  const double air_density_kg_m3_;
  const double wheel_radius_m_;
  const double wheel_inertia_kgm2_;
  const int wheel_count_;
  const double effective_longitudinal_mass_kg_;
  const double drive_delay_;
  const double brake_delay_;
  const double drive_time_constant_;
  const double brake_time_constant_;
  const bool all_wheel_drive_;
  const bool enable_lateral_dynamics_;
  double prev_vx_ = 0.0;

  double beta_ = 0.0;
  double kappa_ = 0.0;
  double v_ = 0.0;
  double omega_ = 0.0;

  std::deque<double> vx_input_queue_;       //!< @brief buffer for velocity command
  std::deque<double> drive_ax_input_queue_;
  std::deque<double> brake_ax_input_queue_;
  std::deque<double> drive_use_ax_input_queue_;
  std::deque<double> brake_use_ax_input_queue_;
  std::deque<double> steer_f_input_queue_;  //!< @brief buffer for angular velocity command
  std::deque<double> steer_r_input_queue_;
  const double vx_delay_;  //!< @brief time delay for velocity command [s]
  const double vx_time_constant_;
  const double steer_delay_;  //!< @brief time delay for angular-velocity command [s]
  const double steer_time_constant_;
  const double steer_dead_band_;  //!< @brief dead band for steering angle [rad]
  const double steer_bias_;       //!< @brief steering angle bias [rad]

  void initializeInputQueue(const double & dt);
  double resistanceForce(const double vx, const double requested_accel) const;
};

}  // namespace autoware::simulator::simple_planning_simulator

#endif  // SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_DELAY_STEER_VEL_HPP_
