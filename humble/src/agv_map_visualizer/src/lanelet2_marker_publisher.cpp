#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <string>

namespace
{
using autoware_map_msgs::msg::LaneletMapBin;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

std_msgs::msg::ColorRGBA make_color(const float r, const float g, const float b, const float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

Marker make_line_marker(
  const std::string & frame_id, const rclcpp::Time & stamp, const std::string & ns,
  const int32_t id, const double width, const std_msgs::msg::ColorRGBA & color)
{
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = Marker::LINE_LIST;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = width;
  marker.color = color;
  marker.lifetime = rclcpp::Duration::from_seconds(0.0);
  return marker;
}

template <class LineStringT>
void append_segments(Marker & marker, const LineStringT & line_string)
{
  if (line_string.size() < 2) {
    return;
  }

  for (auto point = line_string.begin(); std::next(point) != line_string.end(); ++point) {
    marker.points.push_back(
      autoware::experimental::lanelet2_utils::to_ros(point->basicPoint()));
    marker.points.push_back(
      autoware::experimental::lanelet2_utils::to_ros(std::next(point)->basicPoint()));
  }
}
}  // namespace

class Lanelet2MarkerPublisher : public rclcpp::Node
{
public:
  Lanelet2MarkerPublisher() : Node("lanelet2_marker_publisher")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/map/vector_map");
    output_topic_ = declare_parameter<std::string>("output_topic", "/map/vector_map_marker");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    boundary_width_ = declare_parameter<double>("boundary_width", 0.18);
    centerline_width_ = declare_parameter<double>("centerline_width", 0.08);

    pub_markers_ = create_publisher<MarkerArray>(output_topic_, rclcpp::QoS{1}.transient_local());
    sub_map_ = create_subscription<LaneletMapBin>(
      input_topic_, rclcpp::QoS{1}.transient_local(),
      std::bind(&Lanelet2MarkerPublisher::on_map, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Publishing Lanelet2 RViz markers: %s -> %s", input_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  void on_map(const LaneletMapBin::ConstSharedPtr msg)
  {
    lanelet::LaneletMapConstPtr map;
    try {
      map = autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to deserialize LaneletMapBin: %s", error.what());
      return;
    }

    const auto stamp = now();
    const auto marker_frame = msg->header.frame_id.empty() ? frame_id_ : msg->header.frame_id;

    Marker boundaries = make_line_marker(
      marker_frame, stamp, "lanelet_boundaries", 0, boundary_width_,
      make_color(0.25F, 0.85F, 1.0F, 0.95F));
    Marker centerlines = make_line_marker(
      marker_frame, stamp, "lanelet_centerlines", 1, centerline_width_,
      make_color(1.0F, 0.85F, 0.15F, 0.95F));

    for (const auto & lanelet : map->laneletLayer) {
      append_segments(boundaries, lanelet.leftBound());
      append_segments(boundaries, lanelet.rightBound());
      append_segments(centerlines, lanelet.centerline());
    }

    MarkerArray markers;
    markers.markers.push_back(boundaries);
    markers.markers.push_back(centerlines);
    pub_markers_->publish(markers);

    RCLCPP_INFO(
      get_logger(), "Published %zu lanelet markers from %zu lanelets", markers.markers.size(),
      map->laneletLayer.size());
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double boundary_width_;
  double centerline_width_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Lanelet2MarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}
