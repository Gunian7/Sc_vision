#ifndef IO__SUBSCRIBE2NAV_HPP
#define IO__SUBSCRIBE2NAV_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <vector>
#include <optional>

#include "tools/thread_safe_queue.hpp"
#include "communicate_2025/msg/autoaim.hpp"

namespace io
{

struct AutoaimData {
    float pitch;
    float high_gimbal_yaw;
    uint8_t enemy_team_color;
    uint8_t mode;
    uint8_t rune_flag;
    float low_gimbal_yaw;
};

class Subscribe2Nav : public rclcpp::Node
{
public:
  Subscribe2Nav();

  ~Subscribe2Nav();

  void start();

  std::optional<AutoaimData> get_autoaim_data();

private:
  void autoaim_callback(const communicate_2025::msg::Autoaim::SharedPtr msg);

  tools::ThreadSafeQueue<AutoaimData> autoaim_queue_;
  rclcpp::Subscription<communicate_2025::msg::Autoaim>::SharedPtr autoaim_subscription_;
};

}  // namespace io

#endif  // IO__SUBSCRIBE2NAV_HPP