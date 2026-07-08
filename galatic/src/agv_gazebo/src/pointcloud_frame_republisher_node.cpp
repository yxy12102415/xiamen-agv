#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>

class PointCloudFrameRepublisherNode : public rclcpp::Node
{
public:
  PointCloudFrameRepublisherNode() : Node("pointcloud_frame_republisher")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/input/points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/output/points");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    z_offset_ = declare_parameter<double>("z_offset", 0.0);
    use_latest_tf_ = declare_parameter<bool>("use_latest_tf", true);

    const auto input_qos = rclcpp::SensorDataQoS().keep_last(1);
    const auto output_qos = rclcpp::SensorDataQoS().keep_last(1);
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, output_qos);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, input_qos,
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        auto out = *msg;
        out.header.frame_id = frame_id_;
        if (use_latest_tf_) {
          out.header.stamp.sec = 0;
          out.header.stamp.nanosec = 0;
        }
        apply_z_offset(out);
        pub_->publish(out);
        ++publish_count_;
        if (publish_count_ % 100 == 1) {
          RCLCPP_INFO(
            get_logger(), "Republished %zu PointCloud2 messages on %s", publish_count_,
            output_topic_.c_str());
        }
      });

    RCLCPP_INFO(
      get_logger(), "Republishing PointCloud2 %s -> %s with frame_id=%s", input_topic_.c_str(),
      output_topic_.c_str(), frame_id_.c_str());
  }

private:
  void apply_z_offset(sensor_msgs::msg::PointCloud2 & msg) const
  {
    if (std::abs(z_offset_) < 1.0e-6 || msg.point_step == 0) {
      return;
    }

    std::unordered_map<std::string, uint32_t> offsets;
    for (const auto & field : msg.fields) {
      offsets[field.name] = field.offset;
    }
    if (!offsets.count("z")) {
      return;
    }

    const auto z_offset = offsets.at("z");
    const auto point_count = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
    for (size_t i = 0; i < point_count; ++i) {
      const size_t base = i * msg.point_step + z_offset;
      if (base + sizeof(float) > msg.data.size()) {
        break;
      }
      float z = 0.0F;
      std::memcpy(&z, msg.data.data() + base, sizeof(float));
      z += static_cast<float>(z_offset_);
      std::memcpy(msg.data.data() + base, &z, sizeof(float));
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double z_offset_;
  bool use_latest_tf_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  size_t publish_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFrameRepublisherNode>());
  rclcpp::shutdown();
  return 0;
}
