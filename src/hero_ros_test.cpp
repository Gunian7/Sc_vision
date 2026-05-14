/**
 * 模仿自瞄侧仅订阅 ROS：读取 communicate Autoaim（或 yaml 中 hero_autoaim_topic），打日志。
 * 用法: hero_ros_test [config.yaml]
 * 未传 config 时话题默认为 /communicate/autoaim。
 */
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "hero_interfaces/msg/autoaim.hpp"
#include "io/hero_config_path.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::string topic = "/communicate/autoaim";
  if (argc >= 2) {
    const std::string yaml_path =
        io::resolve_config_path_next_to_build(argv[1], argv[0]);
    try {
      auto yaml = YAML::LoadFile(yaml_path);
      if (yaml["hero_autoaim_topic"]) {
        topic = yaml["hero_autoaim_topic"].as<std::string>();
      }
    } catch (const std::exception& e) {
      std::cerr << "[hero_ros_test] yaml load failed: " << e.what() << "\n";
      rclcpp::shutdown();
      return 1;
    }
  }

  auto node = std::make_shared<rclcpp::Node>("hero_ros_test");
  RCLCPP_INFO(node->get_logger(), "subscribe Autoaim topic: %s", topic.c_str());

  auto sub = node->create_subscription<hero_interfaces::msg::Autoaim>(
      topic,
      rclcpp::SensorDataQoS(),
      [node](const hero_interfaces::msg::Autoaim::SharedPtr msg) {
        RCLCPP_INFO(
            node->get_logger(),
            "Autoaim stamp=%u.%09u frame=%s | pitch=%.5f high_gimbal_yaw=%.5f | "
            "enemy_color=%u mode=%u rune=%u low_gimbal_yaw=%.5f",
            msg->header.stamp.sec,
            msg->header.stamp.nanosec,
            msg->header.frame_id.c_str(),
            msg->pitch,
            msg->high_gimbal_yaw,
            static_cast<unsigned>(msg->enemy_team_color),
            static_cast<unsigned>(msg->mode),
            static_cast<unsigned>(msg->rune_flag),
            msg->low_gimbal_yaw);
      });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
