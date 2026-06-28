#include <ignition/msgs/pointcloud_packed.pb.h>
#include <ignition/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace
{
uint8_t to_ros_datatype(const ignition::msgs::PointCloudPacked::Field::DataType datatype)
{
  using IgnField = ignition::msgs::PointCloudPacked::Field;
  using RosField = sensor_msgs::msg::PointField;

  switch (datatype) {
    case IgnField::INT8:
      return RosField::INT8;
    case IgnField::UINT8:
      return RosField::UINT8;
    case IgnField::INT16:
      return RosField::INT16;
    case IgnField::UINT16:
      return RosField::UINT16;
    case IgnField::INT32:
      return RosField::INT32;
    case IgnField::UINT32:
      return RosField::UINT32;
    case IgnField::FLOAT32:
      return RosField::FLOAT32;
    case IgnField::FLOAT64:
      return RosField::FLOAT64;
    default:
      return 0;
  }
}

std::string frame_id_from_header(
  const ignition::msgs::Header & header, const std::string & fallback_frame_id)
{
  for (const auto & data : header.data()) {
    if (data.key() == "frame_id" && data.value_size() > 0) {
      return data.value(0);
    }
  }
  return fallback_frame_id;
}
}  // namespace

class IgnPointCloudBridgeNode : public rclcpp::Node
{
public:
  IgnPointCloudBridgeNode() : Node("ign_pointcloud_bridge")
  {
    ign_topic_ = declare_parameter<std::string>("ign_topic", "/sensing/lidar/points");
    ros_topic_ = declare_parameter<std::string>("ros_topic", "/sim/lidar/points_raw");
    fallback_frame_id_ = declare_parameter<std::string>("fallback_frame_id", "base_link");

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(ros_topic_, rclcpp::QoS{10});
    const auto callback = [this](
                            const char * data, const size_t size,
                            const ignition::transport::MessageInfo &) {
        ignition::msgs::PointCloudPacked ign_msg;
        if (!ign_msg.ParseFromArray(data, static_cast<int>(size))) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Failed to parse Ignition PointCloudPacked from %s", ign_topic_.c_str());
          return;
        }
        on_pointcloud(ign_msg);
      };

    if (!ign_node_.SubscribeRaw(ign_topic_, callback)) {
      throw std::runtime_error("Failed to subscribe to Ignition topic: " + ign_topic_);
    }

    RCLCPP_INFO(
      get_logger(), "Bridging Ignition PointCloudPacked %s -> ROS PointCloud2 %s",
      ign_topic_.c_str(), ros_topic_.c_str());
  }

private:
  void on_pointcloud(const ignition::msgs::PointCloudPacked & ign_msg)
  {
    sensor_msgs::msg::PointCloud2 ros_msg;
    if (ign_msg.has_header() && ign_msg.header().has_stamp()) {
      ros_msg.header.stamp.sec = static_cast<int32_t>(ign_msg.header().stamp().sec());
      ros_msg.header.stamp.nanosec = static_cast<uint32_t>(ign_msg.header().stamp().nsec());
      ros_msg.header.frame_id = frame_id_from_header(ign_msg.header(), fallback_frame_id_);
    } else {
      ros_msg.header.stamp = now();
      ros_msg.header.frame_id = fallback_frame_id_;
    }

    ros_msg.height = ign_msg.height();
    ros_msg.width = ign_msg.width();
    ros_msg.is_bigendian = ign_msg.is_bigendian();
    ros_msg.point_step = ign_msg.point_step();
    ros_msg.row_step = ign_msg.row_step();
    ros_msg.is_dense = ign_msg.is_dense();
    ros_msg.fields.reserve(static_cast<size_t>(ign_msg.field_size()));

    for (const auto & ign_field : ign_msg.field()) {
      sensor_msgs::msg::PointField ros_field;
      ros_field.name = ign_field.name();
      ros_field.offset = ign_field.offset();
      ros_field.datatype = to_ros_datatype(ign_field.datatype());
      ros_field.count = ign_field.count();
      if (ros_field.datatype == 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Skipping PointCloud2 field %s with unknown datatype",
          ros_field.name.c_str());
        continue;
      }
      ros_msg.fields.push_back(ros_field);
    }

    const auto & data = ign_msg.data();
    ros_msg.data.assign(data.begin(), data.end());
    pub_->publish(ros_msg);
    ++publish_count_;
    if (publish_count_ % 100 == 1) {
      RCLCPP_INFO(
        get_logger(), "Published %zu PointCloud2 messages on %s", publish_count_,
        ros_topic_.c_str());
    }
  }

  std::string ign_topic_;
  std::string ros_topic_;
  std::string fallback_frame_id_;
  ignition::transport::Node ign_node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  size_t publish_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IgnPointCloudBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
