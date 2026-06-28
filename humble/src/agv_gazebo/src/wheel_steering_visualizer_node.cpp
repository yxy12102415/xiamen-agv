#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#ifdef AGV_USE_IGN_TRANSPORT_FOR_POSE
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/transport/Node.hh>
#endif
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#ifdef AGV_USE_ROS_IGN_INTERFACES
#include <ros_ign_interfaces/msg/entity.hpp>
#include <ros_ign_interfaces/srv/set_entity_pose.hpp>
#else
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#endif
#include <std_msgs/msg/float32.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace
{
#ifdef AGV_USE_ROS_IGN_INTERFACES
namespace gazebo_interfaces = ros_ign_interfaces;
#else
namespace gazebo_interfaces = ros_gz_interfaces;
#endif

struct Wheel
{
  const char * name;
  double x;
  double y;
  bool front;
};

geometry_msgs::msg::Quaternion yaw_to_quaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double yaw_from_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

#ifdef AGV_USE_IGN_TRANSPORT_FOR_POSE
ignition::msgs::Pose to_ign_pose(
  const std::string & entity_name, const geometry_msgs::msg::Pose & pose)
{
  ignition::msgs::Pose out;
  out.set_name(entity_name);
  out.mutable_position()->set_x(pose.position.x);
  out.mutable_position()->set_y(pose.position.y);
  out.mutable_position()->set_z(pose.position.z);
  out.mutable_orientation()->set_x(pose.orientation.x);
  out.mutable_orientation()->set_y(pose.orientation.y);
  out.mutable_orientation()->set_z(pose.orientation.z);
  out.mutable_orientation()->set_w(pose.orientation.w);
  return out;
}
#endif
}  // namespace

class WheelSteeringVisualizerNode : public rclcpp::Node
{
public:
  WheelSteeringVisualizerNode() : Node("wheel_steering_visualizer")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/localization/kinematic_state");
    front_steer_topic_ = declare_parameter<std::string>("front_steer_topic", "/agv/status/front_steer");
    rear_steer_topic_ = declare_parameter<std::string>("rear_steer_topic", "/agv/status/rear_steer");
    service_name_ =
      declare_parameter<std::string>("service_name", "/world/xiamen_empty/set_pose");
    wheelbase_ = declare_parameter<double>("wheelbase", 2.0);
    wheel_tread_ = declare_parameter<double>("wheel_tread", 1.36);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.66);
    front_x_ = declare_parameter<double>("front_x", wheelbase_ * 0.5);
    rear_x_ = declare_parameter<double>("rear_x", -wheelbase_ * 0.5);
    left_y_ = declare_parameter<double>("left_y", wheel_tread_ * 0.5);
    right_y_ = declare_parameter<double>("right_y", -wheel_tread_ * 0.5);
    wheel_z_ = declare_parameter<double>("wheel_z", wheel_radius_);
    update_rate_ = declare_parameter<double>("update_rate", 30.0);

#ifndef AGV_USE_IGN_TRANSPORT_FOR_POSE
    client_ = create_client<gazebo_interfaces::srv::SetEntityPose>(service_name_);
#endif
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS{1},
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        odom_pose_ = msg->pose.pose;
        have_odom_ = true;
      });
    sub_front_steer_ = create_subscription<std_msgs::msg::Float32>(
      front_steer_topic_, rclcpp::QoS{10},
      [this](const std_msgs::msg::Float32::ConstSharedPtr msg) { front_steer_ = msg->data; });
    sub_rear_steer_ = create_subscription<std_msgs::msg::Float32>(
      rear_steer_topic_, rclcpp::QoS{10},
      [this](const std_msgs::msg::Float32::ConstSharedPtr msg) { rear_steer_ = msg->data; });

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, update_rate_))),
      std::bind(&WheelSteeringVisualizerNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(), "Updating Gazebo wheel steering visuals from %s, %s, %s",
      odom_topic_.c_str(), front_steer_topic_.c_str(), rear_steer_topic_.c_str());
  }

private:
  void on_timer()
  {
    if (!have_odom_) {
      return;
    }

    const std::array<Wheel, 4> wheels{{
      {"ego_agv_front_left_wheel", front_x_, left_y_, true},
      {"ego_agv_front_right_wheel", front_x_, right_y_, true},
      {"ego_agv_rear_left_wheel", rear_x_, left_y_, false},
      {"ego_agv_rear_right_wheel", rear_x_, right_y_, false},
    }};

    const double base_yaw = yaw_from_pose(odom_pose_);
    for (const auto & wheel : wheels) {
      const double steer = wheel.front ? front_steer_ : rear_steer_;
      geometry_msgs::msg::Pose pose;
      pose.position.x =
        odom_pose_.position.x + std::cos(base_yaw) * wheel.x - std::sin(base_yaw) * wheel.y;
      pose.position.y =
        odom_pose_.position.y + std::sin(base_yaw) * wheel.x + std::cos(base_yaw) * wheel.y;
      pose.position.z = odom_pose_.position.z + wheel_z_;
      pose.orientation = yaw_to_quaternion(base_yaw + steer);
      send_pose(wheel.name, pose);
    }
  }

  void send_pose(const std::string & entity_name, const geometry_msgs::msg::Pose & pose)
  {
#ifdef AGV_USE_IGN_TRANSPORT_FOR_POSE
    const auto request = to_ign_pose(entity_name, pose);
    ignition::msgs::Boolean response;
    bool result = false;
    constexpr unsigned int timeout_ms = 5;
    const bool executed = ign_node_.Request(service_name_, request, timeout_ms, response, result);
    if (!executed || !result || !response.data()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignition rejected wheel pose update for %s",
        entity_name.c_str());
    }
#else
    if (!client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for Gazebo set_pose service: %s",
        service_name_.c_str());
      return;
    }

    auto request = std::make_shared<gazebo_interfaces::srv::SetEntityPose::Request>();
    request->entity.name = entity_name;
    request->entity.type = gazebo_interfaces::msg::Entity::MODEL;
    request->pose = pose;
    client_->async_send_request(request);
#endif
  }

  std::string odom_topic_;
  std::string front_steer_topic_;
  std::string rear_steer_topic_;
  std::string service_name_;
  double wheelbase_;
  double wheel_tread_;
  double wheel_radius_;
  double front_x_;
  double rear_x_;
  double left_y_;
  double right_y_;
  double wheel_z_;
  double update_rate_;
  double front_steer_ = 0.0;
  double rear_steer_ = 0.0;
  bool have_odom_ = false;
  geometry_msgs::msg::Pose odom_pose_;

#ifdef AGV_USE_IGN_TRANSPORT_FOR_POSE
  ignition::transport::Node ign_node_;
#endif
  rclcpp::Client<gazebo_interfaces::srv::SetEntityPose>::SharedPtr client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_front_steer_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_rear_steer_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelSteeringVisualizerNode>());
  rclcpp::shutdown();
  return 0;
}
