#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace
{
geometry_msgs::msg::Quaternion yaw_to_quaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}
}  // namespace

class AgvStandaloneSimulatorNode : public rclcpp::Node
{
public:
  AgvStandaloneSimulatorNode() : Node("agv_two_axis_simulator")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
    initial_x_ = declare_parameter<double>("initial_x", 9.9991);
    initial_y_ = declare_parameter<double>("initial_y", 28.0013);
    initial_yaw_ = declare_parameter<double>("initial_yaw", -1.5708);
    dt_ = declare_parameter<double>("dt", 0.025);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", rclcpp::QoS{1});
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(0.001, dt_))),
      std::bind(&AgvStandaloneSimulatorNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(), "Galactic standalone AGV pose publisher started at x=%.3f y=%.3f yaw=%.3f",
      initial_x_, initial_y_, initial_yaw_);
  }

private:
  void on_timer()
  {
    const auto stamp = now();
    const auto orientation = yaw_to_quaternion(initial_yaw_);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = frame_id_;
    transform.child_frame_id = child_frame_id_;
    transform.transform.translation.x = initial_x_;
    transform.transform.translation.y = initial_y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = orientation;

    tf_broadcaster_->sendTransform(transform);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = initial_x_;
    odom.pose.pose.position.y = initial_y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = orientation;
    pub_odom_->publish(odom);
  }

  std::string frame_id_;
  std::string child_frame_id_;
  double initial_x_;
  double initial_y_;
  double initial_yaw_;
  double dt_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgvStandaloneSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
