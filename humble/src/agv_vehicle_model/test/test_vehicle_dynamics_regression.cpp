#include "simple_planning_simulator/vehicle_model/sim_model_delay_steer_vel_two_axis.hpp"

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

using autoware::simulator::simple_planning_simulator::SimModelDelaySteerVelTwoAxis;

namespace
{

std::unique_ptr<SimModelDelaySteerVelTwoAxis> makeModel()
{
  const std::array<double, 4> axle_positions{{4.65, 3.0, -3.0, -4.65}};
  const std::array<double, 4> axle_stiffness{{200000.0, 200000.0, 200000.0, 200000.0}};
  const std::array<double, 4> axle_loads{{56407.5, 56407.5, 56407.5, 56407.5}};
  return std::make_unique<SimModelDelaySteerVelTwoAxis>(
    5.0, 0.5229795759057247, 0.45056049764376704, 0.35, 4.0, 0.261799,
    4.65, 4.65, 23000.0, 432363.7513, 400000.0, 400000.0, 112815.0,
    112815.0, axle_positions, axle_stiffness, axle_loads, 0.8, 2.381384,
    6.116611371515338, 3.5527407651152627, true, 0.8, 0.012, 1.10,
    7.936695, 1.225, 0.66, 67.0824, 8, 0.025, 0.05, 0.2, 0.05, 0.05,
    0.18, 0.18, 0.02, 0.05, 11188.8, 280.0, 0.02, 0.05, 0.005236,
    0.003491, 0.0, true, true);
}

bool near(const double actual, const double expected, const double tolerance, const std::string & name)
{
  if (std::abs(actual - expected) <= tolerance) {
    return true;
  }
  std::cerr << name << ": expected " << expected << ", got " << actual << '\n';
  return false;
}

}  // namespace

int main()
{
  auto model = makeModel();
  Eigen::VectorXd input = Eigen::VectorXd::Zero(5);
  input(0) = 2.0;
  Eigen::VectorXd straight_state = Eigen::VectorXd::Zero(9);
  straight_state(3) = 2.0;
  const auto straight_derivative = model->calcModel(straight_state, input);
  bool ok = true;
  ok &= near(straight_derivative(4), 0.0, 1.0e-12, "straight lateral acceleration");
  ok &= near(straight_derivative(5), 0.0, 1.0e-12, "straight yaw acceleration");

  Eigen::VectorXd left_state = straight_state;
  left_state(4) = 0.15;
  left_state(5) = 0.05;
  left_state(6) = 0.20;
  left_state(7) = 0.12;
  Eigen::VectorXd right_state = left_state;
  right_state(4) *= -1.0;
  right_state(5) *= -1.0;
  right_state(6) *= -1.0;
  right_state(7) *= -1.0;
  const auto left_derivative = model->calcModel(left_state, input);
  const auto right_derivative = model->calcModel(right_state, input);
  ok &= near(
    left_derivative(4), -right_derivative(4), 1.0e-9,
    "mirrored lateral acceleration");
  ok &= near(left_derivative(5), -right_derivative(5), 1.0e-9, "mirrored yaw acceleration");
  ok &= std::isfinite(left_derivative(4)) && std::isfinite(left_derivative(5));

  Eigen::VectorXd saturated_state = straight_state;
  saturated_state(6) = 0.5229795759057247;
  saturated_state(7) = 0.45056049764376704;
  const auto saturated_derivative = model->calcModel(saturated_state, input);
  constexpr double friction_acceleration_limit = 0.8 * 9.81;
  ok &= std::abs(saturated_derivative(4)) <= friction_acceleration_limit + 1.0e-6;
  ok &= std::isfinite(saturated_derivative(5));

  if (!ok) {
    std::cerr << "Eight-wheel vehicle dynamics regression failed\n";
    return 1;
  }
  std::cout << "Eight-wheel vehicle dynamics regression passed\n";
  return 0;
}
