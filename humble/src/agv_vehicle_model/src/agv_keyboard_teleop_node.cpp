#include <autoware_control_msgs/msg/control.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>
#include <termios.h>
#include <unistd.h>

using autoware_control_msgs::msg::Control;
using namespace std::chrono_literals;

class TerminalRawMode
{
public:
  TerminalRawMode()
  {
    if (!isatty(STDIN_FILENO)) {
      return;
    }
    if (tcgetattr(STDIN_FILENO, &original_) != 0) {
      return;
    }
    termios raw = original_;
    raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
      enabled_ = true;
    }
  }

  ~TerminalRawMode()
  {
    if (enabled_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    }
  }

private:
  termios original_{};
  bool enabled_{false};
};

class AgvKeyboardTeleopNode : public rclcpp::Node
{
public:
  AgvKeyboardTeleopNode() : Node("agv_keyboard_teleop")
  {
    output_topic_ = declare_parameter<std::string>("output_topic", "/control/command/control_cmd");
    velocity_step_ = declare_parameter<double>("velocity_step", 0.2);
    steer_step_ = declare_parameter<double>("steer_step", 0.05);
    max_velocity_ = declare_parameter<double>("max_velocity", 2.0);
    max_steer_ = declare_parameter<double>("max_steer", 0.6);
    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);

    pub_control_ = create_publisher<Control>(output_topic_, rclcpp::QoS{1});
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&AgvKeyboardTeleopNode::on_timer, this));

    print_help();
  }

private:
  void print_help() const
  {
    std::puts("");
    std::puts("AGV keyboard teleop");
    std::puts("-------------------");
    std::puts("w/s : increase/decrease speed");
    std::puts("a/d : steer left/right");
    std::puts("space or x : stop");
    std::puts("c : center steering");
    std::puts("q : quit");
    std::puts("");
  }

  void on_timer()
  {
    read_keys();
    publish_control();
  }

  void read_keys()
  {
    char c = 0;
    while (::read(STDIN_FILENO, &c, 1) == 1) {
      switch (c) {
        case 'w':
        case 'W':
          velocity_ = std::min(velocity_ + velocity_step_, max_velocity_);
          break;
        case 's':
        case 'S':
          velocity_ = std::max(velocity_ - velocity_step_, -max_velocity_);
          break;
        case 'a':
        case 'A':
          steer_ = std::min(steer_ + steer_step_, max_steer_);
          break;
        case 'd':
        case 'D':
          steer_ = std::max(steer_ - steer_step_, -max_steer_);
          break;
        case 'c':
        case 'C':
          steer_ = 0.0;
          break;
        case ' ':
        case 'x':
        case 'X':
          velocity_ = 0.0;
          steer_ = 0.0;
          break;
        case 'q':
        case 'Q':
          rclcpp::shutdown();
          return;
      }
      print_status();
    }
  }

  void publish_control()
  {
    Control msg;
    msg.stamp = now();
    msg.longitudinal.velocity = static_cast<float>(velocity_);
    msg.longitudinal.acceleration = 0.0F;
    msg.lateral.steering_tire_angle = static_cast<float>(steer_);
    msg.lateral.steering_tire_rotation_rate = 0.0F;
    pub_control_->publish(msg);
  }

  void print_status() const
  {
    std::printf("\rvelocity: %+0.2f m/s  steer: %+0.2f rad     ", velocity_, steer_);
    std::fflush(stdout);
  }

  std::string output_topic_;
  double velocity_step_{0.2};
  double steer_step_{0.05};
  double max_velocity_{2.0};
  double max_steer_{0.6};
  double publish_rate_{20.0};
  double velocity_{0.0};
  double steer_{0.0};

  rclcpp::Publisher<Control>::SharedPtr pub_control_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  TerminalRawMode raw_mode;
  rclcpp::spin(std::make_shared<AgvKeyboardTeleopNode>());
  rclcpp::shutdown();
  std::puts("");
  return 0;
}
