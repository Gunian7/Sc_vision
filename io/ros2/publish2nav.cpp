#include "publish2nav.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <thread>

#include "tools/logger.hpp"

namespace io
{

Publish2Nav::Publish2Nav() : Node("auto_aim_target_pos_publisher")
{
  publisher_ = this->create_publisher<communicate_2025::msg::SerialInfo>("/shoot_info", 10);
  publisher_ekf_w_ = this->create_publisher<communicate_2025::msg::EKF>("/ekf_w", 10);

  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node initialized.");
}

Publish2Nav::~Publish2Nav()
{
  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node shutting down.");
}

void Publish2Nav::send_data(const Eigen::Vector4d & target_pos)
{
  // 创建消息
  auto message = std::make_shared<communicate_2025::msg::SerialInfo>();

  // 将 Eigen::Vector3d 数据转换为字符串并存储在消息中
  // 将 Eigen::Vector4d 数据转换为消息字段
  message->yaw = static_cast<float>(target_pos[0]);
  message->pitch = static_cast<float>(target_pos[1]);
  
  // is_find 是 std_msgs/Char 类型，需要将double转换为char
  // 通常使用 0/1 来表示 false/true
  // message->is_find.data = static_cast<uint8_t>(0);
  message->is_find.data = static_cast<uint8_t>(target_pos[3] != 0.0 ? 1 : 0);
 
  // 发布消息
  publisher_->publish(*message);
 
  // // 发布消息
  // publisher_->publish(*message);

  // RCLCPP_INFO(
  //   this->get_logger(), "auto_aim_target_pos_publisher node sent message: '%s'",
  //   message->data.c_str());
}

void Publish2Nav::send_ekf_w(const Eigen::VectorXd & ekf_w,const int last_id)
{
  // 创建消息
  auto message = std::make_shared<communicate_2025::msg::EKF>();

  // 将 double 数据存储在消息中
  message->x = static_cast<float>(ekf_w[0]);
  message->vx = static_cast<float>(ekf_w[1]);
  message->y = static_cast<float>(ekf_w[2]);
  message->vy = static_cast<float>(ekf_w[3]);
  message->z = static_cast<float>(ekf_w[4]);
  message->vz = static_cast<float>(ekf_w[5]);
  message->a = static_cast<float>(ekf_w[6]);
  message->w = static_cast<float>(ekf_w[7]);
  message->r = static_cast<float>(ekf_w[8]);
  message->l = static_cast<float>(ekf_w[9]);
  message->h = static_cast<float>(ekf_w[10]);
  message->last_id = last_id;

   // 发布消息
  publisher_ekf_w_->publish(*message);
}

void Publish2Nav::start()
{
  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

}  // namespace io
