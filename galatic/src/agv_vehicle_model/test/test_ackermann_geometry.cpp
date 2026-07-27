#include "agv_vehicle_model/steering_geometry.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <string>

namespace
{

constexpr double kPi = 3.14159265358979323846;

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
  constexpr double tread = 2.381384;
  constexpr double axle1_equivalent_wheelbase = 6.116611371515338;
  constexpr double axle2_equivalent_wheelbase = 3.5527407651152627;
  constexpr double axle1_center_limit = 0.5229795759057247;
  constexpr double axle2_center_limit = 0.45056049764376704;
  bool ok = true;

  const auto axle1 = agv_vehicle_model::ackermannWheelAngles(
    axle1_center_limit, axle1_equivalent_wheelbase, tread);
  ok &= near(axle1.left * 180.0 / kPi, 33.0, 1.0e-6, "axle1 inner angle");
  ok &= near(axle1.right * 180.0 / kPi, 27.4, 1.0e-6, "axle1 outer angle");

  const auto axle2 = agv_vehicle_model::ackermannWheelAngles(
    axle2_center_limit, axle2_equivalent_wheelbase, tread);
  ok &= near(axle2.left * 180.0 / kPi, 30.0, 1.0e-6, "axle2 inner angle");
  ok &= near(axle2.right * 180.0 / kPi, 22.6, 1.0e-6, "axle2 outer angle");

  const auto right_turn = agv_vehicle_model::frontTwoAxleWheelSteers(
    -axle1_center_limit, -axle2_center_limit, axle1_equivalent_wheelbase,
    axle2_equivalent_wheelbase, tread);
  ok &= near(right_turn[0], -axle1.right, 1.0e-12, "right-turn axle1 left");
  ok &= near(right_turn[1], -axle1.left, 1.0e-12, "right-turn axle1 right");
  ok &= near(right_turn[2], -axle2.right, 1.0e-12, "right-turn axle2 left");
  ok &= near(right_turn[3], -axle2.left, 1.0e-12, "right-turn axle2 right");
  for (std::size_t index = 4; index < right_turn.size(); ++index) {
    ok &= near(right_turn[index], 0.0, 0.0, "fixed rear wheel");
  }

  const std::array<double, 4> axle_positions{{4.65, 3.0, -3.0, -4.65}};
  const std::array<double, 8> straight_angles{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  const auto straight = agv_vehicle_model::fitWheelKinematics(
    3.0, axle_positions, tread, straight_angles);
  ok &= near(straight.lateral, 0.0, 1.0e-12, "straight lateral velocity");
  ok &= near(straight.yaw_rate, 0.0, 1.0e-12, "straight yaw rate");

  const auto left_angles = agv_vehicle_model::frontTwoAxleWheelSteers(
    0.2, 0.12, axle1_equivalent_wheelbase, axle2_equivalent_wheelbase, tread);
  const auto left = agv_vehicle_model::fitWheelKinematics(3.0, axle_positions, tread, left_angles);
  const auto mirrored_angles = agv_vehicle_model::frontTwoAxleWheelSteers(
    -0.2, -0.12, axle1_equivalent_wheelbase, axle2_equivalent_wheelbase, tread);
  const auto mirrored = agv_vehicle_model::fitWheelKinematics(
    3.0, axle_positions, tread, mirrored_angles);
  ok &= near(left.lateral, -mirrored.lateral, 1.0e-12, "mirrored lateral velocity");
  ok &= near(left.yaw_rate, -mirrored.yaw_rate, 1.0e-12, "mirrored yaw rate");
  ok &= left.yaw_rate > 0.0;

  const auto saturated_force = agv_vehicle_model::calculateWheelLateralForce(
    4.65, 0.5 * tread, 0.5, 2.0, 0.0, 0.0, 1.0e9, 10000.0, 0.8, 3000.0, false);
  const double expected_force_limit = std::sqrt(8000.0 * 8000.0 - 3000.0 * 3000.0);
  ok &= near(
    std::abs(saturated_force.wheel_lateral_force), expected_force_limit, 1.0e-6,
    "per-wheel friction-circle limit");

  if (!ok) {
    return 1;
  }
  std::cout << "Ackermann geometry and per-wheel tire force regression passed\n";
  return 0;
}
