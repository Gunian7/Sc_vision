#include "io/hero_ros_board/hero_ros_board.hpp"

#include <cmath>
#include <utility>

#include <yaml-cpp/yaml.h>
#include "rclcpp/logging.hpp"

using namespace std::chrono_literals;

namespace io
{

HeroRosBoard::HeroRosBoard(const std::string &config_path)
    : queue_(5000), last_debug_log_time_(std::chrono::steady_clock::now())
{
  auto yaml = YAML::LoadFile(config_path);

  if (yaml["bullet_speed"])
  {
    default_bullet_speed_ = yaml["bullet_speed"].as<double>();
    bullet_speed = default_bullet_speed_;
  }
  if (yaml["use_default_bullet_speed"])
  {
    use_default_bullet_speed_ = yaml["use_default_bullet_speed"].as<bool>();
  }
  if (yaml["phoenix_angle_unit"])
  {
    auto unit = yaml["phoenix_angle_unit"].as<std::string>();
    for (auto &c : unit)
    {
      c = static_cast<char>(std::tolower(c));
    }
    phoenix_angles_in_degrees_ = (unit == "deg" || unit == "degree" || unit == "degrees");
  }
  if (yaml["imu_yaw_offset_deg"])
  {
    imu_yaw_offset_rad_ = yaml["imu_yaw_offset_deg"].as<double>() * M_PI / 180.0;
  }
  if (yaml["imu_pitch_offset_deg"])
  {
    imu_pitch_offset_rad_ = yaml["imu_pitch_offset_deg"].as<double>() * M_PI / 180.0;
  }
  if (yaml["imu_yaw_offset_rad"])
  {
    imu_yaw_offset_rad_ = yaml["imu_yaw_offset_rad"].as<double>();
  }
  if (yaml["imu_pitch_offset_rad"])
  {
    imu_pitch_offset_rad_ = yaml["imu_pitch_offset_rad"].as<double>();
  }
  if (yaml["cboard_debug_log"])
  {
    debug_log_ = yaml["cboard_debug_log"].as<bool>();
  }
  if (yaml["hero_board_simulate_no_mcu"])
  {
    simulate_no_mcu_ = yaml["hero_board_simulate_no_mcu"].as<bool>();
  }
  if (yaml["hero_ros_suppress_rx_stall_log"])
  {
    suppress_autoaim_rx_stall_log_ = yaml["hero_ros_suppress_rx_stall_log"].as<bool>();
  }

  data_ahead_ = {Eigen::Quaterniond::Identity(), std::chrono::steady_clock::now()};
  data_behind_ = data_ahead_;

  if (simulate_no_mcu_)
  {
    mode = auto_aim;
    bullet_speed = default_bullet_speed_;
    {
      std::lock_guard<std::mutex> lock(snapshot_mtx_);
      last_parsed_.has_sample = true;
      last_parsed_.high_gimbal_yaw = 0.F;
      last_parsed_.pitch = 0.F;
      last_parsed_.vtx_pitch = 0.F;
      last_parsed_.low_gimbal_yaw = static_cast<float>(default_bullet_speed_);
      last_parsed_.bullet_speed = default_bullet_speed_;
      last_parsed_.gimbal_q = Eigen::Quaterniond::Identity();
      last_parsed_.received_steady = std::chrono::steady_clock::now();
    }
    spin_thread_ = std::thread([this] { simulate_no_mcu_loop(); });
    RCLCPP_INFO(rclcpp::get_logger("hero_ros_board"),
                "hero_board_simulate_no_mcu=true: 不订阅 Autoaim，本地恒等姿态 + auto_aim");
    return;
  }

  autoaim_topic_ = "/communicate/autoaim";
  if (yaml["hero_autoaim_topic"])
  {
    autoaim_topic_ = yaml["hero_autoaim_topic"].as<std::string>();
  }

  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  const auto t_rx = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(rx_watchdog_mtx_);
    last_autoaim_rx_ = t_rx;
    last_autoaim_warn_ = t_rx - 86400s;
    last_autoaim_watchdog_tick_ = t_rx;
  }

  node_ = std::make_shared<rclcpp::Node>("hero_ros_board");
  sub_ = node_->create_subscription<communicate_26::msg::Autoaim>(
      autoaim_topic_, rclcpp::SensorDataQoS(),
      [this](communicate_26::msg::Autoaim::SharedPtr m) { on_autoaim(std::move(m)); });

  executor_.add_node(node_);
  spin_thread_ = std::thread([this] { spin_loop(); });

  RCLCPP_INFO(node_->get_logger(), "subscribe %s", autoaim_topic_.c_str());
}

HeroRosBoard::~HeroRosBoard()
{
  quit_.store(true);
  if (spin_thread_.joinable())
  {
    spin_thread_.join();
  }
  if (node_)
  {
    executor_.remove_node(node_);
  }
}

void HeroRosBoard::spin_loop()
{
  while (!quit_.load())
  {
    if (rclcpp::ok())
    {
      executor_.spin_some();
    }
    bool tick_watchdog = false;
    {
      const auto now = std::chrono::steady_clock::now();
      std::lock_guard<std::mutex> lock(rx_watchdog_mtx_);
      if (now - last_autoaim_watchdog_tick_ >= 500ms)
      {
        last_autoaim_watchdog_tick_ = now;
        tick_watchdog = true;
      }
    }
    if (tick_watchdog)
    {
      maybe_warn_autoaim_rx_stall();
    }
    std::this_thread::sleep_for(500us);
  }
}

void HeroRosBoard::maybe_warn_autoaim_rx_stall()
{
  if (suppress_autoaim_rx_stall_log_)
  {
    return;
  }
  constexpr auto kIdleWarn = 1500ms;
  constexpr auto kRepeatWarn = 4000ms;

  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(rx_watchdog_mtx_);
  if (now - last_autoaim_rx_ < kIdleWarn)
  {
    return;
  }
  if (now - last_autoaim_warn_ < kRepeatWarn)
  {
    return;
  }
  last_autoaim_warn_ = now;
  const double idle_s = std::chrono::duration<double>(now - last_autoaim_rx_).count();
  if (node_)
  {
    RCLCPP_WARN(node_->get_logger(),
                "已超过 %.2fs 未收到 Autoaim topic=%s（检查 communicate / 话题名 / QoS）", idle_s,
                autoaim_topic_.c_str());
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("hero_ros_board"),
                "已超过 %.2fs 未收到 Autoaim topic=%s（检查 communicate / 话题名 / QoS）", idle_s,
                autoaim_topic_.c_str());
  }
}

void HeroRosBoard::simulate_no_mcu_loop()
{
  while (!quit_.load())
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    queue_.push({q, steady_now});
    {
      std::lock_guard<std::mutex> lock(snapshot_mtx_);
      last_parsed_.has_sample = true;
      last_parsed_.gimbal_q = q;
      last_parsed_.received_steady = steady_now;
      last_parsed_.bullet_speed = bullet_speed;
    }
    mode = auto_aim;
    std::this_thread::sleep_for(1ms);
  }
}

void HeroRosBoard::on_autoaim(const communicate_26::msg::Autoaim::SharedPtr msg)
{
  if (!msg)
  {
    return;
  }

  const auto steady_now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(rx_watchdog_mtx_);
    last_autoaim_rx_ = steady_now;
  }

  if (msg->mode < MODES.size())
  {
    mode = normalize_mcu_mode_for_auto_aim(static_cast<Mode>(msg->mode));
  }
  else if (debug_log_)
  {
    RCLCPP_WARN(node_ ? node_->get_logger() : rclcpp::get_logger("hero_ros_board"),
                "Invalid mode from topic: %d (keep %s)", static_cast<int>(msg->mode),
                MODES[mode].c_str());
  }

  const double raw_v = static_cast<double>(msg->low_gimbal_yaw);
  if (use_default_bullet_speed_)
  {
    bullet_speed = default_bullet_speed_;
  }
  else if (std::isfinite(raw_v) && raw_v >= 1.0)
  {
    bullet_speed = raw_v;
  }
  else if (debug_log_)
  {
    RCLCPP_WARN(node_ ? node_->get_logger() : rclcpp::get_logger("hero_ros_board"),
                "Invalid bullet speed raw=%.3f, keep %.3f", raw_v, bullet_speed);
  }

  double yaw = static_cast<double>(msg->high_gimbal_yaw);
  double pitch = static_cast<double>(msg->pitch);
  const double raw_yaw = yaw;
  const double raw_pitch = pitch;

  if (phoenix_angles_in_degrees_)
  {
    constexpr double kDeg2Rad = M_PI / 180.0;
    yaw *= kDeg2Rad;
    pitch *= kDeg2Rad;
  }
  yaw += imu_yaw_offset_rad_;
  pitch += imu_pitch_offset_rad_;

  if (!std::isfinite(yaw) || !std::isfinite(pitch) || std::abs(yaw) > 1e4 || std::abs(pitch) > 1e4)
  {
    RCLCPP_ERROR(node_ ? node_->get_logger() : rclcpp::get_logger("hero_ros_board"),
                 "Invalid angles, skip IMU sample: yaw=%g pitch=%g", yaw, pitch);
    return;
  }

  const Eigen::AngleAxisd yaw_aa(yaw, Eigen::Vector3d::UnitZ());
  const Eigen::AngleAxisd pitch_aa(pitch, Eigen::Vector3d::UnitY());
  const Eigen::Quaterniond q = (yaw_aa * pitch_aa).normalized();
  queue_.push({q, steady_now});

  {
    std::lock_guard<std::mutex> lock(snapshot_mtx_);
    last_parsed_.ros_stamp = rclcpp::Time(msg->header.stamp);
    last_parsed_.high_gimbal_yaw = msg->high_gimbal_yaw;
    last_parsed_.pitch = msg->pitch;
    last_parsed_.vtx_pitch = msg->vtx_pitch;
    last_parsed_.enemy_team_color = msg->enemy_team_color;
    last_parsed_.mode_u8 = msg->mode;
    last_parsed_.rune_flag = msg->rune_flag;
    last_parsed_.low_gimbal_yaw = msg->low_gimbal_yaw;
    last_parsed_.bullet_speed = bullet_speed;
    last_parsed_.gimbal_q = q;
    last_parsed_.received_steady = steady_now;
    last_parsed_.has_sample = true;
  }

  if (debug_log_)
  {
    const auto dt = std::chrono::duration<double>(steady_now - last_debug_log_time_).count();
    if (dt > 0.5)
    {
      RCLCPP_INFO(node_ ? node_->get_logger() : rclcpp::get_logger("hero_ros_board"),
                  "raw(yaw=%.3f, pitch=%.3f, vtx=%.3f, v=%.3f) unit=%s | mode=%s", raw_yaw, raw_pitch,
                  static_cast<double>(msg->vtx_pitch), raw_v, phoenix_angles_in_degrees_ ? "deg" : "rad",
                  MODES[mode].c_str());
      last_debug_log_time_ = steady_now;
    }
  }
}

Eigen::Quaterniond HeroRosBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp)
  {
    data_ahead_ = data_behind_;
  }

  while (true)
  {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp)
    {
      break;
    }
    data_ahead_ = data_behind_;
  }

  const Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  const Eigen::Quaterniond q_b = data_behind_.q.normalized();
  const auto t_a = data_ahead_.timestamp;
  const auto t_b = data_behind_.timestamp;
  const std::chrono::duration<double> t_ab = t_b - t_a;
  const std::chrono::duration<double> t_ac = timestamp - t_a;
  if (t_ab.count() <= 0.)
  {
    return q_b;
  }
  const double k = (t_ac / t_ab);
  return q_a.slerp(k, q_b).normalized();
}

HeroBoardParsed HeroRosBoard::snapshot() const
{
  std::lock_guard<std::mutex> lock(snapshot_mtx_);
  return last_parsed_;
}

} // namespace io
