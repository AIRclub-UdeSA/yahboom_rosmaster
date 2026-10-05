// Prototype (scratch branch only): republish /clock at a lower rate.
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

class ClockThrottle : public rclcpp::Node
{
public:
  ClockThrottle()
  : Node("clock_throttle")
  {
    const double rate_hz = declare_parameter("rate_hz", 200.0);
    period_ns_ = static_cast<int64_t>(1e9 / rate_hz);
    pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock_throttled", rclcpp::ClockQoS());
    sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS(),
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
        const int64_t now = static_cast<int64_t>(msg->clock.sec) * 1000000000LL + msg->clock.nanosec;
        // Forward the first message at or after each grid point; follow a reset.
        if (now < last_ || now >= next_) {
          pub_->publish(*msg);
          last_ = now;
          next_ = (now / period_ns_ + 1) * period_ns_;
        }
      });
  }

private:
  int64_t period_ns_{5000000};
  int64_t last_{0};
  int64_t next_{0};
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockThrottle>());
  rclcpp::shutdown();
  return 0;
}
