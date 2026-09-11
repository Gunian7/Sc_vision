#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
Aimer::Aimer(const std::string & config_path)
: left_yaw_offset_(std::nullopt),
  right_yaw_offset_(std::nullopt),
  fsm_enable_(true),
  fsm_controller_(),
  fsm_state_(AutoAimFsm::AIM_SINGLE_ARMOR)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;        // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;    // degree to rad
  comming_angle_ = yaml["comming_angle"].as<double>() / 57.3;  // degree to rad
  leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3;  // degree to rad
  comming_angle_high_ = comming_angle_;
  leaving_angle_high_ = leaving_angle_;
  comming_end_angle_ = comming_angle_high_;
  leaving_end_angle_ = leaving_angle_high_;
  outpost_comming_angle_ = 70 / 57.3;
  outpost_leaving_angle_ = 30 / 57.3;
  high_speed_delay_time_ = yaml["high_speed_delay_time"].as<double>();
  low_speed_delay_time_ = yaml["low_speed_delay_time"].as<double>();
  outpost_delay_time_ = high_speed_delay_time_;
  if (yaml["outpost_delay_time"].IsDefined()) {
    outpost_delay_time_ = yaml["outpost_delay_time"].as<double>();
  }
  decision_speed_ = yaml["decision_speed"].as<double>();
  use_center_aim_when_high_speed_ = true;
  if (yaml["use_center_aim_when_high_speed"].IsDefined()) {
    use_center_aim_when_high_speed_ = yaml["use_center_aim_when_high_speed"].as<bool>();
  }
  pre_aim_max_delta_angle_ = 60.0 / 57.3;
  if (yaml["pre_aim_max_delta_angle"].IsDefined()) {
    pre_aim_max_delta_angle_ = yaml["pre_aim_max_delta_angle"].as<double>() / 57.3;
  }
  speed_angle_ = decision_speed_;
  speed_angle_max_ = speed_angle_;
  if (yaml["comming_angle_high"].IsDefined()) {
    comming_angle_high_ = yaml["comming_angle_high"].as<double>() / 57.3;
  }
  if (yaml["leaving_angle_high"].IsDefined()) {
    leaving_angle_high_ = yaml["leaving_angle_high"].as<double>() / 57.3;
  }
  if (yaml["outpost_comming_angle"].IsDefined()) {
    outpost_comming_angle_ = yaml["outpost_comming_angle"].as<double>() / 57.3;
  }
  if (yaml["outpost_leaving_angle"].IsDefined()) {
    outpost_leaving_angle_ = yaml["outpost_leaving_angle"].as<double>() / 57.3;
  }
  if (yaml["comming_end_angle"].IsDefined()) {
    comming_end_angle_ = yaml["comming_end_angle"].as<double>() / 57.3;
  }
  if (yaml["leaving_end_angle"].IsDefined()) {
    leaving_end_angle_ = yaml["leaving_end_angle"].as<double>() / 57.3;
  }
  if (yaml["speed_angle"].IsDefined()) {
    speed_angle_ = yaml["speed_angle"].as<double>();
  }
  if (yaml["speed_angle_max"].IsDefined()) {
    speed_angle_max_ = yaml["speed_angle_max"].as<double>();
  } else {
    speed_angle_max_ = speed_angle_;
  }
  if (yaml["left_yaw_offset"].IsDefined() && yaml["right_yaw_offset"].IsDefined()) {
    left_yaw_offset_ = yaml["left_yaw_offset"].as<double>() / 57.3;    // degree to rad
    right_yaw_offset_ = yaml["right_yaw_offset"].as<double>() / 57.3;  // degree to rad
    tools::logger()->info("[Aimer] successfully loading shootmode");
  }

  if (yaml["auto_aim_fsm"].IsDefined()) {
    const auto fsm_yaml = yaml["auto_aim_fsm"];
    if (fsm_yaml["enable"].IsDefined()) {
      fsm_enable_ = fsm_yaml["enable"].as<bool>();
    }
    fsm_controller_ = AutoAimFsmController(fsm_yaml);
  }
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now)
{
  if (targets.empty()) return {false, false, 0, 0, 0, 0, 0, 0};
  auto target = targets.front();

  auto ekf = target.ekf();
  double delay_time;
  if (target.name == ArmorName::outpost) {
    delay_time = outpost_delay_time_;
  } else {
  delay_time =
    std::abs(target.imm_w()) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;
  }

  // tools::logger()->info(
  //   "[Aimer] w={:.3f} rad/s, delay={:.3f}s, bullet_speed={:.2f} m/s (threshold={:.3f})",
  //   target.ekf_x()[7], delay_time, bullet_speed, decision_speed_);

  if (bullet_speed <= 20) bullet_speed = 20;

  // 考虑detecor和tracker所消耗的时间，此外假设aimer的用时可忽略不计
  auto future = timestamp;
  if (to_now) {
    double dt;
    dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  else {
    auto dt = 0.005 + delay_time;  //detector-aimer耗时0.005+发弹延时0.1
    // tools::logger()->info("dt is {:.4f} second", dt);
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  // FSM 每帧只推进一次：choose_aim_point 在下面的弹道迭代里会被调用多次
  {
    const double spin_w = (std::abs(target.imm_w()) > 1e-6) ? target.imm_w() : target.ekf_x()[7];
    if (fsm_enable_) {
      fsm_controller_.update(spin_w, target.jumped);
      fsm_state_ = fsm_controller_.state();
    } else {
      fsm_state_ = AutoAimFsm::AIM_SINGLE_ARMOR;
    }
  }

  auto aim_point0 = choose_aim_point(target, 0.0);
  debug_aim_point = aim_point0;
  if (!aim_point0.valid) {
    // tools::logger()->debug("Invalid aim_point0.");
    return {false, false, 0, 0, 0, 0, 0, 0};
  }

  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (trajectory0.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
    debug_aim_point.valid = false;
    return {false, false, 0, 0, 0, 0, 0, 0};
  }

  // 迭代求解飞行时间 (最多10次，收敛条件：相邻两次fly_time差 <0.001)
  bool converged = false;
  double prev_fly_time = trajectory0.fly_time;
  tools::Trajectory current_traj = trajectory0;
  std::vector<Target> iteration_target(10, target);  // 创建10个目标副本用于迭代预测
  Target final_target = target;

  for (int iter = 0; iter < 10; ++iter) {
    // 预测目标在 future + prev_fly_time 时刻的位置
    auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
    iteration_target[iter].predict(predict_time);

    // 计算瞄准点
    auto aim_point = choose_aim_point(iteration_target[iter], prev_fly_time);
    debug_aim_point = aim_point;
    final_target = iteration_target[iter];
    if (!aim_point.valid) {
      return {false, false, 0, 0, 0, 0, 0, 0};
    }

    // 计算新弹道
    Eigen::Vector3d xyz = aim_point.xyza.head(3);
    double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
    current_traj = tools::Trajectory(bullet_speed, d, xyz.z());

    // 检查弹道是否可解
    if (current_traj.unsolvable) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, z={:.2f}", iter + 1,
        bullet_speed, d, xyz.z());
      debug_aim_point.valid = false;
      return {false, false, 0, 0, 0, 0, 0, 0};
    }

    // 检查收敛条件
    if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) {
      converged = true;
      break;
    }
    prev_fly_time = current_traj.fly_time;
  }

  // 计算最终角度：yaw可选车心，pitch始终来自装甲板弹道
  Eigen::Vector3d armor_xyz_for_pitch = debug_aim_point.xyza.head(3);
  auto final_ekf_x = final_target.ekf_x();
  const bool center_mode = fsm_enable_
                             ? (fsm_state_ == AutoAimFsm::AIM_WHOLE_CAR_CENTER)
                             : (std::abs(target.imm_w()) > decision_speed_);
  double yaw;
  if (use_center_aim_when_high_speed_ && center_mode) {
    yaw = std::atan2(final_ekf_x[2], final_ekf_x[0]) + yaw_offset_;
  } else {
    yaw = std::atan2(armor_xyz_for_pitch.y(), armor_xyz_for_pitch.x()) + yaw_offset_;
  }
  double pitch = -(current_traj.pitch + pitch_offset_);
  return {true, false, yaw, pitch, 0, 0, 0, 0};
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  io::ShootMode shoot_mode, bool to_now)
{
  double yaw_offset;
  if (shoot_mode == io::left_shoot && left_yaw_offset_.has_value()) {
    yaw_offset = left_yaw_offset_.value();
  } else if (shoot_mode == io::right_shoot && right_yaw_offset_.has_value()) {
    yaw_offset = right_yaw_offset_.value();
  } else {
    yaw_offset = yaw_offset_;
  }

  auto command = aim(targets, timestamp, bullet_speed, to_now);
  command.yaw = command.yaw - yaw_offset_ + yaw_offset;

  return command;
}

AimPoint Aimer::choose_aim_point(const Target & target, double fly_time)
{
  Eigen::VectorXd ekf_x = target.ekf_x();
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  auto armor_num = armor_xyza_list.size();
  if (armor_num == 0) {
    return {false, Eigen::Vector4d::Zero()};
  }

  const double spin_w = (std::abs(target.imm_w()) > 1e-6) ? target.imm_w() : ekf_x[7];

  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped) return {true, armor_xyza_list[0]};

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;
  std::vector<double> effective_delta_angle_list;
  delta_angle_list.reserve(armor_num);
  effective_delta_angle_list.reserve(armor_num);

  for (std::size_t i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
    auto effective_delta = tools::limit_rad(delta_angle + spin_w * fly_time);
    effective_delta_angle_list.emplace_back(effective_delta);
  }

  auto pick_best_by_min_delta = [&](const std::vector<int> & id_list) -> int {
    int best_id = -1;
    double best_abs_delta = std::numeric_limits<double>::max();
    for (int id : id_list) {
      if (id < 0 || id >= static_cast<int>(armor_num)) {
        continue;
      }
      const auto abs_delta = std::abs(effective_delta_angle_list[id]);
      if (abs_delta < best_abs_delta) {
        best_abs_delta = abs_delta;
        best_id = id;
      }
    }
    return best_id;
  };

  if (!fsm_enable_) {
    if (std::abs(target.ekf_x()[8]) <= 2 && target.name != ArmorName::outpost) {
      std::vector<int> id_list;
      for (std::size_t i = 0; i < armor_num; i++) {
        if (std::abs(effective_delta_angle_list[i]) > pre_aim_max_delta_angle_) continue;
        id_list.push_back(static_cast<int>(i));
      }

      if (id_list.empty()) {
        tools::logger()->warn("Empty id list!");
        return {false, armor_xyza_list[0]};
      }

      if (id_list.size() > 1) {
        int id0 = id_list[0], id1 = id_list[1];

        if (lock_id_ != id0 && lock_id_ != id1)
          lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

        return {true, armor_xyza_list[lock_id_]};
      }

      lock_id_ = -1;
      return {true, armor_xyza_list[id_list[0]]};
    }

    double coming_angle, leaving_angle;
    if (target.name == ArmorName::outpost) {
      coming_angle = outpost_comming_angle_;
      leaving_angle = outpost_leaving_angle_;
    } else {
      const double abs_w = std::abs(spin_w);
      if (abs_w < speed_angle_) {
        coming_angle = comming_angle_;
        leaving_angle = leaving_angle_;
      } else if (abs_w >= speed_angle_max_) {
        coming_angle = comming_end_angle_;
        leaving_angle = leaving_end_angle_;
      } else {
        const double denom = speed_angle_max_ - speed_angle_;
        const double t = denom > 1e-6 ? (abs_w - speed_angle_) / denom : 1.0;
        coming_angle = comming_angle_high_ + t * (comming_end_angle_ - comming_angle_high_);
        leaving_angle = leaving_angle_high_ + t * (leaving_end_angle_ - leaving_angle_high_);
      }
    }

    for (std::size_t i = 0; i < armor_num; i++) {
      if (std::abs(effective_delta_angle_list[i]) > pre_aim_max_delta_angle_) continue;
      if (std::abs(effective_delta_angle_list[i]) > coming_angle) continue;
      if (spin_w > 0 && effective_delta_angle_list[i] < leaving_angle) return {true, armor_xyza_list[i]};
      if (spin_w < 0 && effective_delta_angle_list[i] > -leaving_angle) return {true, armor_xyza_list[i]};
    }

    return {false, armor_xyza_list[0]};
  }

  if (fsm_state_ == AutoAimFsm::AIM_SINGLE_ARMOR && target.name != ArmorName::outpost) {
    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;
    for (std::size_t i = 0; i < armor_num; i++) {
      if (std::abs(effective_delta_angle_list[i]) > pre_aim_max_delta_angle_) continue;
      id_list.push_back(static_cast<int>(i));
    }
    // 绝无可能
    if (id_list.empty()) {
      tools::logger()->warn("Empty id list!");
      return {false, armor_xyza_list[0]};
    }

    // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];

      // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

      return {true, armor_xyza_list[lock_id_]};
    }

    // 只有一个装甲板在可射击范围内时，退出锁定模式
    lock_id_ = -1;
    return {true, armor_xyza_list[id_list[0]]};
  }

  lock_id_ = -1;

  if (fsm_state_ == AutoAimFsm::AIM_WHOLE_CAR_PAIR && target.name != ArmorName::outpost) {
    std::vector<int> pair_ids;
    if (armor_num >= 4) {
      if (ekf_x[10] > 0) {
        pair_ids = {1, 3};
      } else {
        pair_ids = {0, 2};
      }
      int selected_id = pick_best_by_min_delta(pair_ids);
      if (selected_id >= 0) {
        return {true, armor_xyza_list[selected_id]};
      }
    }
  }

  if (fsm_state_ == AutoAimFsm::AIM_WHOLE_CAR_CENTER) {
    std::vector<int> all_ids;
    all_ids.reserve(armor_num);
    for (std::size_t i = 0; i < armor_num; ++i) {
      all_ids.push_back(static_cast<int>(i));
    }
    int selected_id = pick_best_by_min_delta(all_ids);
    if (selected_id >= 0) {
      return {true, armor_xyza_list[selected_id]};
    }
  }

  double coming_angle, leaving_angle;
  if (target.name == ArmorName::outpost) {
    coming_angle = outpost_comming_angle_;
    leaving_angle = outpost_leaving_angle_;
  } else {
    const double abs_w = std::abs(spin_w);
    if (abs_w < speed_angle_) {
      coming_angle = comming_angle_;
      leaving_angle = leaving_angle_;
    } else if (abs_w >= speed_angle_max_) {
      coming_angle = comming_end_angle_;
      leaving_angle = leaving_end_angle_;
    } else {
      const double denom = speed_angle_max_ - speed_angle_;
      const double t = denom > 1e-6 ? (abs_w - speed_angle_) / denom : 1.0;
      coming_angle = comming_angle_high_ + t * (comming_end_angle_ - comming_angle_high_);
      leaving_angle = leaving_angle_high_ + t * (leaving_end_angle_ - leaving_angle_high_);
    }
  }

  // 在小陀螺时，一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
  for (std::size_t i = 0; i < armor_num; i++) {
    if (std::abs(effective_delta_angle_list[i]) > pre_aim_max_delta_angle_) continue;
    if (std::abs(effective_delta_angle_list[i]) > coming_angle) continue;
      if (spin_w > 0 && effective_delta_angle_list[i] < leaving_angle) return {true, armor_xyza_list[i]};
      if (spin_w < 0 && effective_delta_angle_list[i] > -leaving_angle) return {true, armor_xyza_list[i]};
  }

  std::vector<int> all_ids;
  all_ids.reserve(armor_num);
  for (std::size_t i = 0; i < armor_num; ++i) {
    all_ids.push_back(static_cast<int>(i));
  }
  int selected_id = pick_best_by_min_delta(all_ids);
  if (selected_id >= 0) {
    return {true, armor_xyza_list[selected_id]};
  }
  return {false, armor_xyza_list[0]};
}

}  // namespace auto_aim
