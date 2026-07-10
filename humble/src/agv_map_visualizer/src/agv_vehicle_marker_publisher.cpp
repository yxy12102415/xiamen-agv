#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace
{
using nav_msgs::msg::Odometry;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

std_msgs::msg::ColorRGBA color(const float r, const float g, const float b, const float a)
{
  std_msgs::msg::ColorRGBA out;
  out.r = r;
  out.g = g;
  out.b = b;
  out.a = a;
  return out;
}

geometry_msgs::msg::Quaternion quaternion_from_rpy(
  const double roll, const double pitch, const double yaw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

geometry_msgs::msg::Quaternion multiply(
  const geometry_msgs::msg::Quaternion & a, const geometry_msgs::msg::Quaternion & b)
{
  geometry_msgs::msg::Quaternion q;
  q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return q;
}

geometry_msgs::msg::Point rotate_point(
  const geometry_msgs::msg::Quaternion & q, const double x, const double y, const double z)
{
  geometry_msgs::msg::Point point;
  const double xx = q.x * q.x;
  const double yy = q.y * q.y;
  const double zz = q.z * q.z;
  const double xy = q.x * q.y;
  const double xz = q.x * q.z;
  const double yz = q.y * q.z;
  const double wx = q.w * q.x;
  const double wy = q.w * q.y;
  const double wz = q.w * q.z;

  point.x = (1.0 - 2.0 * (yy + zz)) * x + 2.0 * (xy - wz) * y + 2.0 * (xz + wy) * z;
  point.y = 2.0 * (xy + wz) * x + (1.0 - 2.0 * (xx + zz)) * y + 2.0 * (yz - wx) * z;
  point.z = 2.0 * (xz - wy) * x + 2.0 * (yz + wx) * y + (1.0 - 2.0 * (xx + yy)) * z;
  return point;
}

struct SensorVisual
{
  const char * ns;
  int id;
  int type;
  double x;
  double y;
  double z;
  double yaw;
  double size_x;
  double size_y;
  double size_z;
  std_msgs::msg::ColorRGBA color;
};

struct WheelVisual
{
  int id;
  double x;
  double y;
  double z;
};

Marker sensor_marker(
  const std::string & frame_id, const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & base,
  const SensorVisual & sensor)
{
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = sensor.ns;
  marker.id = sensor.id;
  marker.type = sensor.type;
  marker.action = Marker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);

  const auto offset = rotate_point(base.orientation, sensor.x, sensor.y, sensor.z);
  marker.pose.position.x = base.position.x + offset.x;
  marker.pose.position.y = base.position.y + offset.y;
  marker.pose.position.z = base.position.z + offset.z;
  marker.pose.orientation = multiply(base.orientation, quaternion_from_rpy(0.0, 0.0, sensor.yaw));
  marker.scale.x = sensor.size_x;
  marker.scale.y = sensor.size_y;
  marker.scale.z = sensor.size_z;
  marker.color = sensor.color;
  return marker;
}

Marker wheel_marker(
  const std::string & frame_id, const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & base,
  const WheelVisual & wheel)
{
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "agv_wheels";
  marker.id = wheel.id;
  marker.type = Marker::CYLINDER;
  marker.action = Marker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);

  const auto offset = rotate_point(base.orientation, wheel.x, wheel.y, wheel.z);
  marker.pose.position.x = base.position.x + offset.x;
  marker.pose.position.y = base.position.y + offset.y;
  marker.pose.position.z = base.position.z + offset.z;
  marker.pose.orientation = multiply(base.orientation, quaternion_from_rpy(1.5708, 0.0, 0.0));
  marker.scale.x = 1.32;
  marker.scale.y = 1.32;
  marker.scale.z = 0.42;
  marker.color = color(0.015F, 0.015F, 0.015F, 1.0F);
  return marker;
}

}  // namespace

class AgvVehicleMarkerPublisher : public rclcpp::Node
{
public:
  AgvVehicleMarkerPublisher() : Node("agv_vehicle_marker_publisher")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/localization/kinematic_state");
    output_topic_ =
      declare_parameter<std::string>("output_topic", "/visualization/agv_vehicle_marker");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    visual_z_offset_ = declare_parameter<double>("visual_z_offset", 0.0);
    pub_marker_ = create_publisher<MarkerArray>(output_topic_, rclcpp::QoS{1});
    sub_odom_ = create_subscription<Odometry>(
      input_topic_, rclcpp::QoS{10},
      std::bind(&AgvVehicleMarkerPublisher::on_odometry, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Publishing AGV primitive marker: %s -> %s", input_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  void on_odometry(const Odometry::ConstSharedPtr msg)
  {
    const auto stamp = now();
    const auto frame = msg->header.frame_id.empty() ? frame_id_ : msg->header.frame_id;
    auto pose = msg->pose.pose;
    pose.position.z += visual_z_offset_;

    MarkerArray markers;

    const std::vector<SensorVisual> body_parts = {
      {"agv_body", 0, Marker::CUBE, 0.0, 0.0, 1.28, 0.0, 14.69, 2.95, 0.50,
       color(0.02F, 0.46F, 0.56F, 0.95F)},
      {"agv_body", 1, Marker::CUBE, 5.95, 0.0, 1.75, 0.0, 2.10, 2.85, 1.15,
       color(0.018F, 0.38F, 0.46F, 0.95F)},
      {"agv_body", 2, Marker::CUBE, -5.95, 0.0, 1.75, 0.0, 2.10, 2.85, 1.15,
       color(0.018F, 0.38F, 0.46F, 0.95F)},
      {"agv_body", 3, Marker::CUBE, 0.0, 0.0, 1.82, 0.0, 7.60, 0.55, 0.36,
       color(0.018F, 0.34F, 0.42F, 0.95F)},
    };
    for (const auto & body_part : body_parts) {
      markers.markers.push_back(sensor_marker(frame, stamp, pose, body_part));
    }

    const std::vector<WheelVisual> wheels = {
      {40, 4.479714, 1.62, 0.66},
      {41, 4.479714, -1.62, 0.66},
      {42, 2.596727, 1.62, 0.66},
      {43, 2.596727, -1.62, 0.66},
      {44, -2.570058, 1.62, 0.66},
      {45, -2.570058, -1.62, 0.66},
      {46, -4.479289, 1.62, 0.66},
      {47, -4.479289, -1.62, 0.66},
    };
    for (const auto & wheel : wheels) {
      markers.markers.push_back(wheel_marker(frame, stamp, pose, wheel));
    }

    const std::vector<SensorVisual> sensors = {
      {"agv_lidars", 10, Marker::CYLINDER, 7.346, 1.560, 1.25, 0.785398, 0.18, 0.18, 0.12,
       color(1.0F, 0.95F, 0.0F, 1.0F)},
      {"agv_lidars", 11, Marker::CYLINDER, 7.346, -1.560, 1.25, -0.785398, 0.18, 0.18, 0.12,
       color(1.0F, 0.0F, 0.0F, 1.0F)},
      {"agv_lidars", 12, Marker::CYLINDER, -7.346, -1.560, 1.25, -2.356194, 0.18, 0.18, 0.12,
       color(1.0F, 0.95F, 0.0F, 1.0F)},
      {"agv_lidars", 13, Marker::CYLINDER, -7.346, 1.560, 1.25, 2.356194, 0.18, 0.18, 0.12,
       color(1.0F, 0.0F, 0.0F, 1.0F)},
      {"agv_lidars", 14, Marker::CYLINDER, 4.55, 1.52, 2.45, 0.0, 0.22, 0.22, 0.16,
       color(1.0F, 0.95F, 0.0F, 1.0F)},
      {"agv_lidars", 15, Marker::CYLINDER, -4.55, -1.52, 2.45, 3.141593, 0.22, 0.22, 0.16,
       color(1.0F, 0.95F, 0.0F, 1.0F)},
      {"agv_cameras", 20, Marker::CUBE, 7.65, 0.0, 1.35, 0.0, 0.16, 0.36, 0.22,
       color(0.0F, 0.95F, 1.0F, 1.0F)},
      {"agv_cameras", 21, Marker::CUBE, -7.65, 0.0, 1.35, 3.141593, 0.16, 0.36, 0.22,
       color(0.0F, 0.95F, 1.0F, 1.0F)},
      {"agv_cameras", 22, Marker::CUBE, 3.75, 1.65, 1.35, 1.570796, 0.16, 0.36, 0.22,
       color(0.0F, 0.95F, 1.0F, 1.0F)},
      {"agv_cameras", 23, Marker::CUBE, -3.75, 1.65, 1.35, 1.570796, 0.16, 0.36, 0.22,
       color(0.0F, 0.95F, 1.0F, 1.0F)},
      {"agv_cameras", 24, Marker::CUBE, 3.75, -1.65, 1.35, -1.570796, 0.16, 0.36, 0.22,
       color(0.0F, 0.95F, 1.0F, 1.0F)},
      {"agv_cameras", 25, Marker::CUBE, -3.75, -1.65, 1.35, -1.570796, 0.16, 0.36, 0.22,
       color(0.0F, 0.95F, 1.0F, 1.0F)},
      {"agv_gnss", 30, Marker::CUBE, 5.30, 0.0, 2.45, 0.0, 0.24, 0.24, 0.12,
       color(0.55F, 0.2F, 1.0F, 1.0F)},
      {"agv_gnss", 31, Marker::CUBE, -5.30, 0.0, 2.45, 0.0, 0.24, 0.24, 0.12,
       color(0.55F, 0.2F, 1.0F, 1.0F)},
    };
    for (const auto & sensor : sensors) {
      markers.markers.push_back(sensor_marker(frame, stamp, pose, sensor));
    }
    pub_marker_->publish(markers);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double visual_z_offset_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgvVehicleMarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}
