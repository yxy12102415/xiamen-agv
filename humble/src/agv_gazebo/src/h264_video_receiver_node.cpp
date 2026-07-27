#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
using SteadyClock = std::chrono::steady_clock;

std::vector<char *> exec_arguments(std::vector<std::string> & arguments)
{
  std::vector<char *> result;
  result.reserve(arguments.size() + 1);
  for (auto & argument : arguments) {
    result.push_back(argument.data());
  }
  result.push_back(nullptr);
  return result;
}
}  // namespace

class H264VideoReceiver : public rclcpp::Node
{
public:
  H264VideoReceiver()
  : Node("h264_video_receiver")
  {
    camera_name_ = declare_parameter<std::string>("camera_name", "front");
    output_prefix_ = declare_parameter<std::string>("output_prefix", "/remote/video");
    decoder_input_port_ = declare_parameter<int>("decoder_input_port", 15200);
    width_ = declare_parameter<int>("width", 960);
    height_ = declare_parameter<int>("height", 540);
    frame_rate_ = declare_parameter<double>("frame_rate", 15.0);
    configured_delay_ms_ = declare_parameter<double>("configured_delay_ms", 50.0);
    stale_after_s_ = declare_parameter<double>("stale_after_s", 1.0);
    const double diagnostic_rate = declare_parameter<double>("diagnostic_rate", 5.0);

    if (width_ <= 0 || height_ <= 0 || decoder_input_port_ <= 0) {
      throw std::invalid_argument("invalid H.264 receiver dimensions or UDP port");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      output_prefix_ + "/" + camera_name_ + "/image", qos);
    diagnostic_pub_ = create_publisher<std_msgs::msg::String>(
      "/remote/network/video_state", 10);

    start_decoder();
    diagnostic_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, diagnostic_rate))),
      std::bind(&H264VideoReceiver::publish_diagnostic, this));
    RCLCPP_INFO(
      get_logger(), "C++ H.264 receiver [%s] listening on UDP %d at %dx%d",
      camera_name_.c_str(), decoder_input_port_, width_, height_);
  }

  ~H264VideoReceiver() override
  {
    stop_decoder();
  }

private:
  void start_decoder()
  {
    decoder_stopped_.store(false);
    int pipe_fds[2];
    if (pipe(pipe_fds) != 0) {
      throw std::runtime_error("failed to create FFmpeg output pipe");
    }

    const std::string input_url =
      "udp://127.0.0.1:" + std::to_string(decoder_input_port_) +
      "?fifo_size=2000000&overrun_nonfatal=1";
    std::vector<std::string> arguments = {
      "ffmpeg", "-hide_banner", "-loglevel", "error",
      "-fflags", "nobuffer+discardcorrupt+flush_packets",
      "-flags", "low_delay", "-max_delay", "0",
      "-probesize", "32768", "-analyzeduration", "0", "-fpsprobesize", "0",
      "-threads", "1", "-f", "h264", "-i", input_url, "-an",
      "-flush_packets", "1", "-f", "rawvideo", "-pix_fmt", "bgr24",
      "-vsync", "0", "pipe:1"
    };

    decoder_pid_ = fork();
    if (decoder_pid_ == 0) {
      dup2(pipe_fds[1], STDOUT_FILENO);
      const int null_fd = open("/dev/null", O_WRONLY);
      if (null_fd >= 0) {
        dup2(null_fd, STDERR_FILENO);
        close(null_fd);
      }
      close(pipe_fds[0]);
      close(pipe_fds[1]);
      auto argv = exec_arguments(arguments);
      execvp(argv[0], argv.data());
      _exit(127);
    }
    if (decoder_pid_ < 0) {
      close(pipe_fds[0]);
      close(pipe_fds[1]);
      throw std::runtime_error("failed to fork FFmpeg decoder");
    }

    close(pipe_fds[1]);
    decoder_output_fd_ = pipe_fds[0];
    running_.store(true);
    reader_thread_ = std::thread(&H264VideoReceiver::reader_loop, this);
  }

  bool read_frame(std::vector<uint8_t> & destination)
  {
    size_t offset = 0;
    while (running_.load() && offset < destination.size()) {
      const ssize_t count = read(
        decoder_output_fd_, destination.data() + offset, destination.size() - offset);
      if (count > 0) {
        offset += static_cast<size_t>(count);
        continue;
      }
      if (count < 0 && errno == EINTR) {
        continue;
      }
      return false;
    }
    return offset == destination.size();
  }

  void reader_loop()
  {
    const size_t frame_size = static_cast<size_t>(width_) * height_ * 3U;
    while (running_.load()) {
      auto message = std::make_unique<sensor_msgs::msg::Image>();
      message->data.resize(frame_size);
      if (!read_frame(message->data)) {
        break;
      }
      message->header.stamp = now();
      message->header.frame_id = camera_name_ + "_camera_remote";
      message->height = static_cast<uint32_t>(height_);
      message->width = static_cast<uint32_t>(width_);
      message->encoding = "bgr8";
      message->is_bigendian = false;
      message->step = static_cast<uint32_t>(width_ * 3);
      image_pub_->publish(std::move(message));
      decoded_count_.fetch_add(1);
      last_frame_ns_.store(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          SteadyClock::now().time_since_epoch()).count());
    }
    decoder_stopped_.store(running_.load());
  }

  void publish_diagnostic()
  {
    const auto now_steady = SteadyClock::now();
    if (
      decoder_stopped_.load() &&
      std::chrono::duration<double>(now_steady - last_decoder_restart_).count() >= 1.0)
    {
      RCLCPP_WARN(
        get_logger(), "H.264 decoder [%s] stopped; restarting UDP receiver on port %d",
        camera_name_.c_str(), decoder_input_port_);
      stop_decoder();
      start_decoder();
      last_decoder_restart_ = now_steady;
    }
    const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      now_steady.time_since_epoch()).count();
    const int64_t last_ns = last_frame_ns_.load();
    const double age_s = last_ns > 0 ? (now_ns - last_ns) * 1.0e-9 : -1.0;
    const bool live = age_s >= 0.0 && age_s <= stale_after_s_;
    const uint64_t count = decoded_count_.load();
    const double elapsed = std::max(
      1.0e-6, std::chrono::duration<double>(now_steady - previous_metric_time_).count());
    const double output_fps = (count - previous_decoded_count_) / elapsed;
    previous_decoded_count_ = count;
    previous_metric_time_ = now_steady;
    // The configured value models the one-way network delay. Do not add a
    // whole frame period here; that made the console report 116.7 ms for a
    // 50 ms link before any actual decode or display latency was measured.
    const double estimated_delivery_ms = configured_delay_ms_;

    std::ostringstream stream;
    stream << "{\"link\":\"" << (live ? "LIVE" : "WAITING")
           << "\",\"transport\":\"H264/UDP/ANNEXB\",\"role\":\"receiver\""
           << ",\"configured_delay_ms\":" << configured_delay_ms_
           << ",\"cameras\":{\"" << camera_name_ << "\":{";
    stream << "\"state\":\""
           << (live ? "LIVE" : (last_ns > 0 ? "STALE" : "WAITING")) << "\""
           << ",\"output_fps\":" << output_fps
           << ",\"last_frame_age_ms\":" << (age_s < 0.0 ? -1.0 : age_s * 1000.0)
           << ",\"last_delivery_delay_ms\":" << estimated_delivery_ms
           << ",\"estimated_end_to_end_ms\":" << estimated_delivery_ms
           << ",\"decoder_stopped\":" << (decoder_stopped_.load() ? "true" : "false")
           << "}}}";
    std_msgs::msg::String message;
    message.data = stream.str();
    diagnostic_pub_->publish(message);
  }

  void stop_decoder()
  {
    if (!running_.exchange(false)) {
      return;
    }
    if (decoder_pid_ > 0) {
      kill(decoder_pid_, SIGTERM);
    }
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
    if (decoder_output_fd_ >= 0) {
      close(decoder_output_fd_);
      decoder_output_fd_ = -1;
    }
    if (decoder_pid_ > 0) {
      int status = 0;
      waitpid(decoder_pid_, &status, 0);
      decoder_pid_ = -1;
    }
  }

  std::string camera_name_;
  std::string output_prefix_;
  int decoder_input_port_{15200};
  int width_{960};
  int height_{540};
  double frame_rate_{15.0};
  double configured_delay_ms_{50.0};
  double stale_after_s_{1.0};
  pid_t decoder_pid_{-1};
  int decoder_output_fd_{-1};
  std::atomic<bool> running_{false};
  std::atomic<bool> decoder_stopped_{false};
  std::atomic<uint64_t> decoded_count_{0};
  std::atomic<int64_t> last_frame_ns_{0};
  uint64_t previous_decoded_count_{0};
  SteadyClock::time_point previous_metric_time_{SteadyClock::now()};
  SteadyClock::time_point last_decoder_restart_{SteadyClock::now()};
  std::thread reader_thread_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_pub_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<H264VideoReceiver>());
  rclcpp::shutdown();
  return 0;
}
