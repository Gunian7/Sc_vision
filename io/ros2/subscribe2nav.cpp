#include "subscribe2nav.hpp"

#include <sstream>
#include <vector>

namespace io
{

Subscribe2Nav::Subscribe2Nav()
: Node("nav_subscriber"),
  autoaim_queue_(1)
{
  // 创建订阅器，订阅 /communicate/autoaim 话题
  autoaim_subscription_ = this->create_subscription<communicate_2025::msg::Autoaim>(
    "/communicate/autoaim", 10,
    std::bind(&Subscribe2Nav::autoaim_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "nav_subscriber node initialized, subscribing to /communicate/autoaim");
}

Subscribe2Nav::~Subscribe2Nav()
{
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node shutting down.");
}

void Subscribe2Nav::autoaim_callback(const communicate_2025::msg::Autoaim::SharedPtr msg)
{
  AutoaimData data;
  data.pitch = msg->pitch;
  data.high_gimbal_yaw = msg->high_gimbal_yaw;
  data.enemy_team_color = msg->enemy_team_color;
  data.mode = msg->mode;
  data.rune_flag = msg->rune_flag;
  data.low_gimbal_yaw = msg->low_gimbal_yaw;

  autoaim_queue_.clear();
  autoaim_queue_.push(data);

  RCLCPP_DEBUG(this->get_logger(), "Received autoaim data: pitch=%.2f, high_yaw=%.2f, color=%d, mode=%d", 
               data.pitch, data.high_gimbal_yaw, data.enemy_team_color, data.mode);
}

void Subscribe2Nav::start()
{
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

std::optional<AutoaimData> Subscribe2Nav::get_autoaim_data()
{
  if (autoaim_queue_.empty()) {
    return std::nullopt;
  }
  
  AutoaimData data;
  autoaim_queue_.back(data);
  
  return data;
}

}  // namespace io