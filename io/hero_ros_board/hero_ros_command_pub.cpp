#include "io/hero_ros_board/hero_ros_command_pub.hpp"

#include <yaml-cpp/yaml.h>
#include "rclcpp/logging.hpp"

namespace io
{

HeroRosCommandPublisher::HeroRosCommandPublisher(const std::string &config_path)
{
  try
  {
    const auto y = YAML::LoadFile(config_path);
    if (y["hero_command_publish_enable"])
    {
      enabled_ = y["hero_command_publish_enable"].as<bool>();
    }
    if (y["hero_command_topic"])
    {
      topic_ = y["hero_command_topic"].as<std::string>();
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_WARN(rclcpp::get_logger("hero_ros_command_pub"), "yaml load failed: %s", e.what());
    enabled_ = false;
    return;
  }

  if (!enabled_)
  {
    return;
  }
  if (!rclcpp::ok())
  {
    RCLCPP_WARN(rclcpp::get_logger("hero_ros_command_pub"), "rclcpp not ok, disable command publish");
    enabled_ = false;
    return;
  }

  node_ = std::make_shared<rclcpp::Node>("hero_ros_command_pub");
  pub_ = node_->create_publisher<hero_interfaces::msg::SerialInfo>(topic_, rclcpp::QoS(10).reliable());
  RCLCPP_INFO(node_->get_logger(),
              "SerialInfo publish topic=%s（可选：与 hero_common hero_link_node 的 vision_serial_topic 对齐）",
              topic_.c_str());
}

void HeroRosCommandPublisher::publish(const Command &cmd)
{
  if (!enabled_ || !pub_)
  {
    return;
  }

  hero_interfaces::msg::SerialInfo msg;
  msg.yaw = static_cast<float>(cmd.yaw);
  msg.pitch = static_cast<float>(cmd.pitch);
  msg.vel_yaw = static_cast<float>(cmd.yaw_vel);
  msg.vel_pitch = static_cast<float>(cmd.pitch_vel);
  constexpr signed char kYes = static_cast<signed char>('1');
  constexpr signed char kNo = static_cast<signed char>('0');
  msg.is_find.data = cmd.control ? kYes : kNo;
  msg.is_shoot.data = cmd.shoot ? kYes : kNo;
  pub_->publish(msg);
}

} // namespace io
