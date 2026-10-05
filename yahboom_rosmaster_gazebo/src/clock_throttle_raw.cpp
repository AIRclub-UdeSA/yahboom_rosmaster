// Prototype (scratch branch only): republish /clock at a lower rate without
// deserializing it. The CDR payload is a 4-byte encapsulation header, then
// int32 sec and uint32 nanosec; only those 8 bytes are read, and the original
// serialized message is forwarded as it is.
#include <cstdint>
#include <cstring>
#include <memory>

#include "rclcpp/rclcpp.hpp"

class ClockThrottleRaw : public rclcpp::Node
{
public:
  ClockThrottleRaw()
  : Node("clock_throttle")
  {
    const double rate_hz = declare_parameter("rate_hz", 200.0);
    period_ns_ = static_cast<int64_t>(1e9 / rate_hz);
    pub_ = create_generic_publisher("/clock_throttled", "rosgraph_msgs/msg/Clock", rclcpp::ClockQoS());
    sub_ = create_generic_subscription(
      "/clock", "rosgraph_msgs/msg/Clock", rclcpp::ClockQoS(),
      [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        const auto & rcl_msg = msg->get_rcl_serialized_message();
        if (rcl_msg.buffer_length < 12) {
          return;
        }
        // The encapsulation header's second byte says little (1) or big (0) endian.
        const bool little = rcl_msg.buffer[1] == 1;
        auto read = [&](size_t offset) {
            uint32_t v;
            std::memcpy(&v, rcl_msg.buffer + offset, 4);
            return little ? v : __builtin_bswap32(v);
          };
        const int64_t now = static_cast<int64_t>(static_cast<int32_t>(read(4))) * 1000000000LL + read(8);
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
  rclcpp::GenericPublisher::SharedPtr pub_;
  rclcpp::GenericSubscription::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockThrottleRaw>());
  rclcpp::shutdown();
  return 0;
}
