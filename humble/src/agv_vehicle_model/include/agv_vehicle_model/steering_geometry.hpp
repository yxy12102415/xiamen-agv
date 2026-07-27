#ifndef AGV_VEHICLE_MODEL__STEERING_GEOMETRY_HPP_
#define AGV_VEHICLE_MODEL__STEERING_GEOMETRY_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace agv_vehicle_model
{

struct WheelSteerPair
{
  double left;
  double right;
};

struct PlanarVelocity
{
  double lateral;
  double yaw_rate;
};

struct WheelLateralForce
{
  double slip_angle;
  double wheel_lateral_force;
  double body_lateral_force;
  double yaw_moment;
};

inline WheelSteerPair ackermannWheelAngles(
  const double axle_center_steer, const double equivalent_wheelbase,
  const double wheel_tread)
{
  const double abs_steer = std::abs(axle_center_steer);
  if (abs_steer < 1.0e-9 || equivalent_wheelbase <= 1.0e-9 || wheel_tread <= 1.0e-9) {
    return {axle_center_steer, axle_center_steer};
  }

  const double center_radius = equivalent_wheelbase / std::tan(abs_steer);
  const double half_tread = 0.5 * wheel_tread;
  const double inner_radius = std::max(1.0e-6, center_radius - half_tread);
  const double outer_radius = center_radius + half_tread;
  const double inner_steer = std::atan(equivalent_wheelbase / inner_radius);
  const double outer_steer = std::atan(equivalent_wheelbase / outer_radius);

  if (axle_center_steer > 0.0) {
    return {inner_steer, outer_steer};
  }
  return {-outer_steer, -inner_steer};
}

inline std::array<double, 8> frontTwoAxleWheelSteers(
  const double axle1_center_steer, const double axle2_center_steer,
  const double axle1_equivalent_wheelbase, const double axle2_equivalent_wheelbase,
  const double wheel_tread, const bool enable_ackermann = true)
{
  const auto axle1 = enable_ackermann ?
    ackermannWheelAngles(axle1_center_steer, axle1_equivalent_wheelbase, wheel_tread) :
    WheelSteerPair{axle1_center_steer, axle1_center_steer};
  const auto axle2 = enable_ackermann ?
    ackermannWheelAngles(axle2_center_steer, axle2_equivalent_wheelbase, wheel_tread) :
    WheelSteerPair{axle2_center_steer, axle2_center_steer};

  return {{axle1.left, axle1.right, axle2.left, axle2.right, 0.0, 0.0, 0.0, 0.0}};
}

inline PlanarVelocity fitWheelKinematics(
  const double longitudinal_velocity, const std::array<double, 4> & axle_positions,
  const double wheel_tread, const std::array<double, 8> & wheel_steers)
{
  double sum_a1 = 0.0;
  double sum_a1_a1 = 0.0;
  double sum_b = 0.0;
  double sum_a1_b = 0.0;
  const double half_tread = 0.5 * wheel_tread;

  for (std::size_t wheel_index = 0; wheel_index < wheel_steers.size(); ++wheel_index) {
    const std::size_t axle_index = wheel_index / 2;
    const double wheel_y = wheel_index % 2 == 0 ? half_tread : -half_tread;
    const double steer_tangent = std::tan(wheel_steers[wheel_index]);
    const double a1 = axle_positions[axle_index] + wheel_y * steer_tangent;
    const double b = longitudinal_velocity * steer_tangent;
    sum_a1 += a1;
    sum_a1_a1 += a1 * a1;
    sum_b += b;
    sum_a1_b += a1 * b;
  }

  const double count = static_cast<double>(wheel_steers.size());
  const double denominator = count * sum_a1_a1 - sum_a1 * sum_a1;
  if (std::abs(denominator) <= 1.0e-9) {
    return {0.0, 0.0};
  }
  return {
    (sum_b * sum_a1_a1 - sum_a1 * sum_a1_b) / denominator,
    (count * sum_a1_b - sum_a1 * sum_b) / denominator,
  };
}

inline WheelLateralForce calculateWheelLateralForce(
  const double wheel_x, const double wheel_y, const double wheel_steer,
  const double longitudinal_velocity, const double lateral_velocity,
  const double yaw_rate, const double cornering_stiffness,
  const double normal_load, const double friction_coefficient,
  const double longitudinal_force, const bool smooth_saturation)
{
  const double local_longitudinal_velocity = longitudinal_velocity - yaw_rate * wheel_y;
  const double velocity_sign_reference = std::abs(local_longitudinal_velocity) > 1.0e-9 ?
    local_longitudinal_velocity : (longitudinal_velocity == 0.0 ? 1.0 : longitudinal_velocity);
  const double slip_longitudinal_velocity = std::copysign(
    std::max(std::abs(local_longitudinal_velocity), 0.5), velocity_sign_reference);
  const double local_lateral_velocity = lateral_velocity + yaw_rate * wheel_x;
  const double slip_angle =
    wheel_steer - std::atan2(local_lateral_velocity, slip_longitudinal_velocity);
  double lateral_force = cornering_stiffness * slip_angle;

  if (friction_coefficient > 0.0) {
    const double friction_limit = friction_coefficient * std::max(0.0, normal_load);
    const double lateral_limit = std::sqrt(std::max(
      0.0, friction_limit * friction_limit - longitudinal_force * longitudinal_force));
    if (smooth_saturation && lateral_limit > 1.0) {
      lateral_force = lateral_limit * std::tanh(lateral_force / lateral_limit);
    } else {
      lateral_force = std::clamp(lateral_force, -lateral_limit, lateral_limit);
    }
  }

  const double body_lateral_force = lateral_force * std::cos(wheel_steer);
  const double yaw_moment = lateral_force * (
    wheel_x * std::cos(wheel_steer) + wheel_y * std::sin(wheel_steer));
  return {slip_angle, lateral_force, body_lateral_force, yaw_moment};
}

}  // namespace agv_vehicle_model

#endif  // AGV_VEHICLE_MODEL__STEERING_GEOMETRY_HPP_
