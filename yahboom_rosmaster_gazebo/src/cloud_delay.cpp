// Prototype (scratch branch only): hold each cloud until sim time reaches
// stamp + latency_s, then publish it. The clock comes from /clock directly (the
// node itself runs on wall time), so no timer per cloud is needed.
#include <deque>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using sensor_msgs::msg::PointCloud2;

class CloudDelay : public rclcpp::Node
{
public:
  CloudDelay()
  : Node("cloud_delay")
  {
    latency_ns_ = static_cast<int64_t>(declare_parameter("latency_s", 0.05) * 1e9);
    const auto input = declare_parameter("input_topic", std::string("/internal/cam_1/depth/color/points"));
    const auto output = declare_parameter("output_topic", std::string("/cam_1/depth/color/points"));
    pub_ = create_publisher<PointCloud2>(output, rclcpp::SensorDataQoS());
    cloud_sub_ = create_subscription<PointCloud2>(
      input, rclcpp::SensorDataQoS(),
      [this](PointCloud2::UniquePtr msg) {
        const int64_t due = stamp_ns(*msg) + latency_ns_;
        if (latency_ns_ <= 0 || sim_now_ >= due) {
          pub_->publish(std::move(msg));
          return;
        }
        held_.emplace_back(due, std::move(msg));
      });
    clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS(),
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
        sim_now_ = static_cast<int64_t>(msg->clock.sec) * 1000000000LL + msg->clock.nanosec;
        while (!held_.empty() && held_.front().first <= sim_now_) {
          pub_->publish(std::move(held_.front().second));
          held_.pop_front();
        }
      });
  }

private:
  static int64_t stamp_ns(const PointCloud2 & m)
  {
    return static_cast<int64_t>(m.header.stamp.sec) * 1000000000LL + m.header.stamp.nanosec;
  }

  int64_t latency_ns_{50000000};
  int64_t sim_now_{0};
  std::deque<std::pair<int64_t, PointCloud2::UniquePtr>> held_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudDelay>());
  rclcpp::shutdown();
  return 0;
}
