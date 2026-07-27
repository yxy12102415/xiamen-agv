#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <ignition/msgs/image.pb.h>
#include <ignition/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

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

bool write_all(const int fd, const uint8_t * data, const size_t size)
{
  size_t offset = 0;
  while (offset < size) {
    const ssize_t written = ::write(fd, data + offset, size - offset);
    if (written > 0) {
      offset += static_cast<size_t>(written);
      continue;
    }
    if (written < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}
}  // namespace

class DirectH264VideoSender : public rclcpp::Node
{
public:
  DirectH264VideoSender()
  : Node("direct_h264_video_sender")
  {
    camera_name_ = declare_parameter<std::string>("camera_name", "front");
    ignition_topic_ = declare_parameter<std::string>(
      "ignition_topic", "/sensing/camera/front/image");
    udp_host_ = declare_parameter<std::string>("udp_host", "127.0.0.1");
    udp_port_ = declare_parameter<int>("udp_port", 15100);
    encoder_ = declare_parameter<std::string>("encoder", "h264_nvenc");
    ffmpeg_path_ = declare_parameter<std::string>("ffmpeg_path", "ffmpeg");
    frame_rate_ = declare_parameter<double>("frame_rate", 15.0);
    bitrate_kbps_ = declare_parameter<int>("bitrate_kbps", 1500);
    keyframe_interval_ = declare_parameter<int>("keyframe_interval", 5);
    output_width_ = declare_parameter<int>("output_width", 0);
    output_height_ = declare_parameter<int>("output_height", 0);

    diagnostic_pub_ = create_publisher<std_msgs::msg::String>(
      "/remote/network/video_state", rclcpp::QoS(10));
    diagnostic_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&DirectH264VideoSender::publish_diagnostic, this));

    const std::function<void(
      const ignition::msgs::Image &,
      const ignition::transport::MessageInfo &)> callback = [this](
      const ignition::msgs::Image & message,
      const ignition::transport::MessageInfo &) {
        on_image(message);
      };
    if (!ign_node_.Subscribe<ignition::msgs::Image>(ignition_topic_, callback)) {
      throw std::runtime_error(
        "Failed to subscribe to Ignition camera topic: " + ignition_topic_);
    }

    writer_thread_ = std::thread(&DirectH264VideoSender::writer_loop, this);
    RCLCPP_INFO(
      get_logger(),
      "Direct Gazebo H.264 sender [%s]: %s -> UDP %s:%d using %s",
      camera_name_.c_str(), ignition_topic_.c_str(), udp_host_.c_str(), udp_port_,
      encoder_.c_str());
  }

  ~DirectH264VideoSender() override
  {
    stopping_.store(true);
    frame_cv_.notify_all();
    if (writer_thread_.joinable()) {
      writer_thread_.join();
    }
    stop_encoder();
  }

private:
  void on_image(const ignition::msgs::Image & message)
  {
    const auto format = message.pixel_format_type();
    std::string pixel_format;
    if (format == ignition::msgs::RGB_INT8) {
      pixel_format = "rgb24";
    } else if (format == ignition::msgs::BGR_INT8) {
      pixel_format = "bgr24";
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unsupported pixel format %d on %s", static_cast<int>(format), ignition_topic_.c_str());
      return;
    }

    const auto & data = message.data();
    const size_t expected_size =
      static_cast<size_t>(message.width()) * static_cast<size_t>(message.height()) * 3U;
    if (message.width() == 0 || message.height() == 0 || data.size() < expected_size) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      width_ = static_cast<int>(message.width());
      height_ = static_cast<int>(message.height());
      pixel_format_ = pixel_format;
      latest_frame_.assign(
        reinterpret_cast<const uint8_t *>(data.data()),
        reinterpret_cast<const uint8_t *>(data.data()) + expected_size);
      frame_sequence_++;
      last_input_ns_.store(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          SteadyClock::now().time_since_epoch()).count());
      input_count_++;
    }
    frame_cv_.notify_one();
  }

  void start_encoder(const int width, const int height, const std::string & pixel_format)
  {
    if (encoder_pid_ > 0) {
      return;
    }

    int pipe_fds[2];
    if (pipe(pipe_fds) != 0) {
      throw std::runtime_error("failed to create FFmpeg input pipe");
    }

    const int keyframe_interval = std::max(1, keyframe_interval_);
    std::vector<std::string> arguments = {
      ffmpeg_path_, "-hide_banner", "-loglevel", "error",
      "-f", "rawvideo", "-pix_fmt", pixel_format,
      "-s", std::to_string(width) + "x" + std::to_string(height),
      "-r", std::to_string(frame_rate_), "-i", "pipe:0", "-an"
    };
    if (output_width_ > 0 && output_height_ > 0 &&
      (output_width_ != width || output_height_ != height))
    {
      arguments.insert(
        arguments.end(),
        {
          "-vf",
          "scale=" + std::to_string(output_width_) + ":" +
          std::to_string(output_height_) + ":flags=fast_bilinear"
        });
    }
    if (encoder_ == "h264_nvenc") {
      arguments.insert(
        arguments.end(),
        {
          "-c:v", "h264_nvenc", "-preset", "p1", "-tune", "ull",
          "-rc", "cbr", "-zerolatency", "1", "-delay", "0",
          "-rc-lookahead", "0", "-surfaces", "1", "-forced-idr", "1", "-aud", "1"
        });
    } else {
      arguments.insert(
        arguments.end(),
        {
          "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
          "-x264-params",
          "scenecut=0:keyint=" + std::to_string(keyframe_interval) +
          ":min-keyint=" + std::to_string(keyframe_interval) + ":repeat-headers=1"
        });
    }
    arguments.insert(
      arguments.end(),
      {
        "-b:v", std::to_string(bitrate_kbps_) + "k",
        "-maxrate", std::to_string(bitrate_kbps_) + "k",
        "-bufsize", std::to_string(std::max(250, bitrate_kbps_ / 2)) + "k",
        "-g", std::to_string(keyframe_interval), "-bf", "0",
        "-vsync", "0", "-flush_packets", "1",
        "-bsf:v", "dump_extra=freq=keyframe",
        "-f", "h264",
        "udp://" + udp_host_ + ":" + std::to_string(udp_port_) + "?pkt_size=1316"
      });

    encoder_pid_ = fork();
    if (encoder_pid_ == 0) {
      dup2(pipe_fds[0], STDIN_FILENO);
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
    if (encoder_pid_ < 0) {
      close(pipe_fds[0]);
      close(pipe_fds[1]);
      throw std::runtime_error("failed to fork FFmpeg encoder");
    }

    close(pipe_fds[0]);
    encoder_stdin_fd_ = pipe_fds[1];
    encoder_width_ = width;
    encoder_height_ = height;
    encoder_pixel_format_ = pixel_format;
    encoded_count_ = 0;
    RCLCPP_INFO(
      get_logger(), "FFmpeg encoder started for %s at %dx%d", camera_name_.c_str(), width, height);
  }

  void stop_encoder()
  {
    const int fd = encoder_stdin_fd_;
    encoder_stdin_fd_ = -1;
    if (fd >= 0) {
      close(fd);
    }
    if (encoder_pid_ > 0) {
      kill(encoder_pid_, SIGTERM);
      int status = 0;
      waitpid(encoder_pid_, &status, 0);
      encoder_pid_ = -1;
    }
  }

  void writer_loop()
  {
    uint64_t consumed_sequence = 0;
    while (!stopping_.load()) {
      std::vector<uint8_t> frame;
      int width = 0;
      int height = 0;
      std::string pixel_format;
      {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        frame_cv_.wait_for(lock, std::chrono::milliseconds(100), [this, &consumed_sequence] {
          return stopping_.load() || frame_sequence_ > consumed_sequence;
        });
        if (stopping_.load()) {
          break;
        }
        if (frame_sequence_ == consumed_sequence) {
          continue;
        }
        consumed_sequence = frame_sequence_;
        frame = latest_frame_;
        width = width_;
        height = height_;
        pixel_format = pixel_format_;
      }

      try {
        if (encoder_pid_ <= 0) {
          start_encoder(width, height, pixel_format);
        }
        if (
          width != encoder_width_ || height != encoder_height_ ||
          pixel_format != encoder_pixel_format_)
        {
          stop_encoder();
          start_encoder(width, height, pixel_format);
        }
        if (!write_all(encoder_stdin_fd_, frame.data(), frame.size())) {
          stop_encoder();
        } else {
          encoded_count_++;
        }
      } catch (const std::exception & error) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "H.264 sender [%s] failed: %s",
          camera_name_.c_str(), error.what());
        stop_encoder();
      }
    }
  }

  void publish_diagnostic()
  {
    const int64_t last_ns = last_input_ns_.load();
    const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      SteadyClock::now().time_since_epoch()).count();
    const double age_ms = last_ns > 0 ? (now_ns - last_ns) * 1.0e-6 : -1.0;
    const bool live = age_ms >= 0.0 && age_ms < 1000.0;
    std_msgs::msg::String message;
    message.data =
      "{\"link\":\"" + std::string(live ? "LIVE" : "WAITING") +
      "\",\"transport\":\"H264/UDP/ANNEXB\",\"role\":\"sender\"," +
      "\"cameras\":{\"" + camera_name_ + "\":{" +
      "\"state\":\"" + std::string(live ? "LIVE" : "WAITING") +
      "\",\"encoder\":\"" + encoder_ + "\",\"input_fps\":0.0}}}";
    diagnostic_pub_->publish(message);
  }

  std::string camera_name_;
  std::string ignition_topic_;
  std::string udp_host_;
  int udp_port_{15100};
  std::string encoder_;
  std::string ffmpeg_path_;
  double frame_rate_{15.0};
  int bitrate_kbps_{1500};
  int keyframe_interval_{5};
  int output_width_{0};
  int output_height_{0};

  ignition::transport::Node ign_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_pub_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;

  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  std::vector<uint8_t> latest_frame_;
  std::string pixel_format_;
  int width_{0};
  int height_{0};
  uint64_t frame_sequence_{0};
  uint64_t input_count_{0};
  uint64_t encoded_count_{0};
  std::atomic<int64_t> last_input_ns_{0};
  std::atomic<bool> stopping_{false};
  std::thread writer_thread_;

  pid_t encoder_pid_{-1};
  int encoder_stdin_fd_{-1};
  int encoder_width_{0};
  int encoder_height_{0};
  std::string encoder_pixel_format_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectH264VideoSender>());
  rclcpp::shutdown();
  return 0;
}
