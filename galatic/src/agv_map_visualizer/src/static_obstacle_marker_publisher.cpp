#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace
{
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

struct SceneMesh
{
  std::string name;
  std::string resource;
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  double scale;
};

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

std_msgs::msg::ColorRGBA color(const float r, const float g, const float b, const float a)
{
  std_msgs::msg::ColorRGBA out;
  out.r = r;
  out.g = g;
  out.b = b;
  out.a = a;
  return out;
}
}  // namespace

class StaticObstacleMarkerPublisher : public rclcpp::Node
{
public:
  StaticObstacleMarkerPublisher() : Node("static_obstacle_marker_publisher")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    output_topic_ =
      declare_parameter<std::string>("output_topic", "/visualization/static_scene_marker");

    const auto qos = rclcpp::QoS{1}.transient_local().reliable();
    pub_ = create_publisher<MarkerArray>(output_topic_, qos);
    timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&StaticObstacleMarkerPublisher::publish_markers, this));

    RCLCPP_INFO(get_logger(), "Publishing static scene markers on %s", output_topic_.c_str());
  }

private:
  void publish_markers()
  {
    const std::string sts_mesh =
      "package://agv_gazebo/models/quay_crane_sts_single/meshes/sts_single.obj";
    const std::vector<SceneMesh> meshes{
      {"quay_crane_sts_1", sts_mesh, 52.0, -40.0, 0.033, 1.5708, 0.0, 3.1416, 1.0},
      {"quay_crane_sts_2", sts_mesh, 148.0, -40.0, 0.033, 1.5708, 0.0, 3.1416, 1.0},
      {"quay_crane_sts_3", sts_mesh, 244.0, -40.0, 0.033, 1.5708, 0.0, 3.1416, 1.0},
      {"quay_crane_sts_4", sts_mesh, 340.0, -40.0, 0.033, 1.5708, 0.0, 3.1416, 1.0},
    };

    MarkerArray markers;
    const auto stamp = now();
    int32_t id = 0;
    for (const auto & mesh : meshes) {
      Marker marker;
      marker.header.frame_id = frame_id_;
      marker.header.stamp = stamp;
      marker.ns = "static_scene";
      marker.id = id++;
      marker.type = Marker::MESH_RESOURCE;
      marker.action = Marker::ADD;
      marker.pose.position.x = mesh.x;
      marker.pose.position.y = mesh.y;
      marker.pose.position.z = mesh.z;
      marker.pose.orientation = quaternion_from_rpy(mesh.roll, mesh.pitch, mesh.yaw);
      marker.scale.x = mesh.scale;
      marker.scale.y = mesh.scale;
      marker.scale.z = mesh.scale;
      marker.color = color(0.92F, 0.48F, 0.10F, 1.0F);
      marker.mesh_resource = mesh.resource;
      marker.mesh_use_embedded_materials = true;
      markers.markers.push_back(marker);
    }

    pub_->publish(markers);
  }

  std::string frame_id_;
  std::string output_topic_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticObstacleMarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}
