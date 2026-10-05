// Publish ignition.msgs.Image frames on Gazebo transport topics at a fixed
// rate, standing in for the simulator's camera. SIGUSR1 stops publishing but
// keeps the process (and its topics) alive, like pausing the world.
//
//   gz_image_publisher <rate_hz> <width> <height> <topic> [<topic> ...]
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include <ignition/msgs/image.pb.h>
#include <ignition/transport/Node.hh>

static std::atomic<bool> g_paused{false};
static std::atomic<bool> g_quit{false};

int main(int argc, char ** argv)
{
  if (argc < 5) {
    return 2;
  }
  const double rate = std::atof(argv[1]);
  const int width = std::atoi(argv[2]);
  const int height = std::atoi(argv[3]);
  std::signal(SIGUSR1, [](int) {g_paused = true;});
  std::signal(SIGTERM, [](int) {g_quit = true;});
  std::signal(SIGINT, [](int) {g_quit = true;});
  ignition::transport::Node node;
  std::vector<ignition::transport::Node::Publisher> pubs;
  for (int i = 4; i < argc; ++i) {
    pubs.push_back(node.Advertise<ignition::msgs::Image>(argv[i]));
  }
  ignition::msgs::Image msg;
  msg.set_width(width);
  msg.set_height(height);
  msg.set_pixel_format_type(ignition::msgs::PixelFormatType::RGB_INT8);
  msg.set_step(width * 3);
  msg.set_data(std::string(static_cast<size_t>(width) * height * 3, '\x7f'));
  const auto period = std::chrono::duration<double>(1.0 / rate);
  auto next = std::chrono::steady_clock::now();
  while (!g_quit) {
    if (!g_paused) {
      for (auto & pub : pubs) {
        pub.Publish(msg);
      }
    }
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next);
  }
  return 0;
}
