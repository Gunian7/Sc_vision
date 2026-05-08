#ifndef IO__HERO_ROS_BOARD_HERO_ROS_COMMAND_PUB_HPP
#define IO__HERO_ROS_BOARD_HERO_ROS_COMMAND_PUB_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "hero_interfaces/msg/serial_info.hpp"
#include "io/command.hpp"

namespace io
{

/**
 * 启用时向 ROS 发布 SerialInfo（默认入 hero.yaml 的 hero_command_topic）。
 * 若运行 hero_common 的 hero_link_node，可将该话题配置为其 vision 入口以汇入 /shoot_info。
 */
class HeroRosCommandPublisher
{
public:
  explicit HeroRosCommandPublisher(const std::string &config_path);

  bool enabled() const { return enabled_; }

  void publish(const Command &cmd);

private:
  bool enabled_{false};
  std::string topic_{"/communicate/vision_shoot_serial"};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<hero_interfaces::msg::SerialInfo>::SharedPtr pub_;
};

} // namespace io

#endif
