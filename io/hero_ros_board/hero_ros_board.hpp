#ifndef IO__HERO_ROS_BOARD_HERO_ROS_BOARD_HPP
#define IO__HERO_ROS_BOARD_HERO_ROS_BOARD_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "hero_interfaces/msg/autoaim.hpp"
#include "io/hero_ros_board/detail/thread_safe_queue.hpp"
#include "io/mcu_mode.hpp"

namespace io
{

struct HeroBoardParsed
{
  rclcpp::Time ros_stamp{};
  float high_gimbal_yaw{0.F};
  float pitch{0.F};
  float vtx_pitch{0.F};
  uint8_t enemy_team_color{0};
  uint8_t mode_u8{0};
  uint8_t rune_flag{0};
  float low_gimbal_yaw{0.F};
  double bullet_speed{0.};
  Eigen::Quaterniond gimbal_q{Eigen::Quaterniond::Identity()};
  std::chrono::steady_clock::time_point received_steady{};
  bool has_sample{false};
};

/**
 * Sc_vision Hero 栈专用：用 ROS Autoaim 替代串口 CBoard，读取姿态与弹速。
 * 不依赖仓库 hero_common；整栈可与可选包 hero_common（hero_link_node）配合使用。
 */
class HeroRosBoard
{
public:
  explicit HeroRosBoard(const std::string &config_path);
  ~HeroRosBoard();

  HeroRosBoard(const HeroRosBoard &) = delete;
  HeroRosBoard &operator=(const HeroRosBoard &) = delete;

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  Mode mode{idle};
  double bullet_speed{21.0};

  HeroBoardParsed snapshot() const;

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  void on_autoaim(const hero_interfaces::msg::Autoaim::SharedPtr msg);
  void spin_loop();
  void maybe_warn_autoaim_rx_stall();
  void simulate_no_mcu_loop();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<hero_interfaces::msg::Autoaim>::SharedPtr sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  detail::ThreadSafeQueue<IMUData> queue_;
  IMUData data_ahead_;
  IMUData data_behind_;

  bool phoenix_angles_in_degrees_{false};
  double imu_yaw_offset_rad_{0.};
  double imu_pitch_offset_rad_{0.};
  bool use_default_bullet_speed_{false};
  double default_bullet_speed_{21.0};
  bool debug_log_{false};
  std::chrono::steady_clock::time_point last_debug_log_time_;

  mutable std::mutex snapshot_mtx_;
  HeroBoardParsed last_parsed_;

  std::atomic<bool> quit_{false};
  std::thread spin_thread_;

  bool simulate_no_mcu_{false};

  std::string autoaim_topic_;
  mutable std::mutex rx_watchdog_mtx_;
  std::chrono::steady_clock::time_point last_autoaim_rx_{};
  std::chrono::steady_clock::time_point last_autoaim_warn_{};
  std::chrono::steady_clock::time_point last_autoaim_watchdog_tick_{};
  bool suppress_autoaim_rx_stall_log_{false};
};

} // namespace io

#endif
