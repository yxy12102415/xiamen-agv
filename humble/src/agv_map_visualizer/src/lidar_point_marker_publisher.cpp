#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
using sensor_msgs::msg::PointCloud2;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using namespace std::chrono_literals;

struct CloudConfig
{
  std::string topic;
  std::string ns;
  float r;
  float g;
  float b;
};

std_msgs::msg::ColorRGBA color(const float r, const float g, const float b, const float a)
{
  std_msgs::msg::ColorRGBA out;
  out.r = r;
  out.g = g;
  out.b = b;
  out.a = a;
  return out;
}

float read_float32(const std::vector<uint8_t> & data, const size_t offset)
{
  float value = 0.0F;
  std::memcpy(&value, data.data() + offset, sizeof(float));
  return value;
}
}  // namespace

class LidarPointMarkerPublisher : public rclcpp::Node
{
public:
  LidarPointMarkerPublisher() : Node("lidar_point_marker_publisher")
  {
    output_topic_ =
      declare_parameter<std::string>("output_topic", "/visualization/lidar_point_markers");
    sample_step_ = declare_parameter<int>("sample_step", 6);
    max_points_ = declare_parameter<int>("max_points_per_cloud", 12000);
    point_size_ = declare_parameter<double>("point_size", 0.045);
    z_offset_ = declare_parameter<double>("z_offset", 0.0);
    use_cloud_stamp_ = declare_parameter<bool>("use_cloud_stamp", false);
    marker_lifetime_sec_ = declare_parameter<double>("marker_lifetime_sec", 0.25);

    pub_ = create_publisher<MarkerArray>(output_topic_, rclcpp::QoS{1});
    timer_ = create_wall_timer(100ms, [this]() { publish_markers(); });

    const std::vector<CloudConfig> clouds{
      {"/sensing/lidar/fl_lidar_1/pointcloud", "fl_lidar_1_points", 1.0F, 0.95F, 0.0F},
      {"/sensing/lidar/fr_lidar_1/pointcloud", "fr_lidar_1_points", 1.0F, 0.0F, 0.0F},
      {"/sensing/lidar/rl_lidar_1/pointcloud", "rl_lidar_1_points", 1.0F, 0.95F, 0.0F},
      {"/sensing/lidar/rr_lidar_1/pointcloud", "rr_lidar_1_points", 1.0F, 0.0F, 0.0F},
      {"/sensing/lidar/fl_lidar_2/pointcloud", "fl_lidar_2_points", 1.0F, 0.95F, 0.0F},
      {"/sensing/lidar/rl_lidar_2/pointcloud", "rl_lidar_2_points", 1.0F, 0.95F, 0.0F},
    };

    int32_t id = 0;
    for (const auto & cloud : clouds) {
      configs_.emplace(cloud.topic, cloud);
      ids_.emplace(cloud.topic, id++);
      subs_.push_back(create_subscription<PointCloud2>(
        cloud.topic, rclcpp::SensorDataQoS().keep_last(1),
        [this, topic = cloud.topic](const PointCloud2::ConstSharedPtr msg) {
          update_marker(topic, *msg);
        }));
    }

    RCLCPP_INFO(get_logger(), "Publishing sampled lidar point markers on %s", output_topic_.c_str());
  }

private:
  void update_marker(const std::string & topic, const PointCloud2 & msg)
  {
    std::unordered_map<std::string, uint32_t> offsets;
    for (const auto & field : msg.fields) {
      offsets[field.name] = field.offset;
    }
    if (!offsets.count("x") || !offsets.count("y") || !offsets.count("z") || msg.point_step == 0) {
      return;
    }

    const auto & config = configs_.at(topic);
    Marker marker;
    marker.header.frame_id = msg.header.frame_id;
    if (use_cloud_stamp_) {
      marker.header.stamp = msg.header.stamp;
    } else {
      marker.header.stamp.sec = 0;
      marker.header.stamp.nanosec = 0;
    }
    marker.ns = config.ns;
    marker.id = ids_.at(topic);
    marker.type = Marker::SPHERE_LIST;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = point_size_;
    marker.scale.y = point_size_;
    marker.scale.z = point_size_;
    marker.color = color(config.r, config.g, config.b, 0.95F);
    marker.lifetime = rclcpp::Duration::from_seconds(std::max(0.05, marker_lifetime_sec_));

    const size_t point_count = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
    const size_t step = static_cast<size_t>(std::max(1, sample_step_));
    marker.points.reserve(static_cast<size_t>(max_points_));
    for (size_t i = 0; i < point_count && marker.points.size() < static_cast<size_t>(max_points_);
         i += step) {
      const size_t base = i * msg.point_step;
      if (base + offsets["z"] + sizeof(float) > msg.data.size()) {
        break;
      }
      geometry_msgs::msg::Point point;
      point.x = read_float32(msg.data, base + offsets["x"]);
      point.y = read_float32(msg.data, base + offsets["y"]);
      point.z = read_float32(msg.data, base + offsets["z"]) + z_offset_;
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        marker.points.push_back(point);
      }
    }

    if (marker.points.empty()) {
      return;
    }

    latest_markers_[topic] = std::move(marker);
    publish_markers();
  }

  void publish_markers()
  {
    if (latest_markers_.empty()) {
      return;
    }

    MarkerArray markers;
    markers.markers.reserve(latest_markers_.size());
    for (auto & [topic, marker] : latest_markers_) {
      (void)topic;
      if (!use_cloud_stamp_) {
        marker.header.stamp.sec = 0;
        marker.header.stamp.nanosec = 0;
      }
      markers.markers.push_back(marker);
    }
    pub_->publish(markers);
  }

  std::string output_topic_;
  int sample_step_;
  int max_points_;
  double point_size_;
  double z_offset_;
  bool use_cloud_stamp_;
  double marker_lifetime_sec_;
  std::unordered_map<std::string, CloudConfig> configs_;
  std::unordered_map<std::string, int32_t> ids_;
  std::unordered_map<std::string, Marker> latest_markers_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Subscription<PointCloud2>::SharedPtr> subs_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarPointMarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}
