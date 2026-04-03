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
: left_yaw_offset_(std::nullopt), right_yaw_offset_(std::nullopt)
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
  decision_speed_ = yaml["decision_speed"].as<double>();
  use_center_aim_when_high_speed_ = true;
  if (yaml["use_center_aim_when_high_speed"].IsDefined()) {
    use_center_aim_when_high_speed_ = yaml["use_center_aim_when_high_speed"].as<bool>();
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
  jump_pitch_up_ = 0.0;
  jump_pitch_down_ = 0.0;
  jump_pitch_up_duration_ = 0.0;
  jump_pitch_down_duration_ = 0.0;
  if (yaml["jump_pitch_up"].IsDefined()) {
    jump_pitch_up_ = yaml["jump_pitch_up"].as<double>() / 57.3;
  }
  if (yaml["jump_pitch_down"].IsDefined()) {
    jump_pitch_down_ = yaml["jump_pitch_down"].as<double>() / 57.3;
  }
  if (yaml["jump_pitch_up_duration"].IsDefined()) {
    jump_pitch_up_duration_ = yaml["jump_pitch_up_duration"].as<double>();
  }
  if (yaml["jump_pitch_down_duration"].IsDefined()) {
    jump_pitch_down_duration_ = yaml["jump_pitch_down_duration"].as<double>();
  }
  if (yaml["left_yaw_offset"].IsDefined() && yaml["right_yaw_offset"].IsDefined()) {
    left_yaw_offset_ = yaml["left_yaw_offset"].as<double>() / 57.3;    // degree to rad
    right_yaw_offset_ = yaml["right_yaw_offset"].as<double>() / 57.3;  // degree to rad
    tools::logger()->info("[Aimer] successfully loading shootmode");
  }
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now)
{
  if (targets.empty()) return {false, false, 0, 0, 0, 0, 0, 0};
  auto target = targets.front();

  auto ekf = target.ekf();
  double delay_time =
    std::abs(target.ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

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

  auto aim_point0 = choose_aim_point(target);
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
    auto aim_point = choose_aim_point(iteration_target[iter]);
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
  double yaw;
  if (use_center_aim_when_high_speed_ && std::abs(final_ekf_x[7]) > decision_speed_) {
    yaw = std::atan2(final_ekf_x[2], final_ekf_x[0]) + yaw_offset_;
  } else {
    yaw = std::atan2(armor_xyz_for_pitch.y(), armor_xyz_for_pitch.x()) + yaw_offset_;
  }
  double jump_correction = 0.0;
  if (target.has_jump_time() && std::abs(target.ekf_x()[7]) >= decision_speed_) {
    auto age = std::chrono::duration<double>(timestamp - target.last_jump_time()).count();
    auto dir = target.last_jump_dir();
    if (dir < 0 && jump_pitch_up_duration_ > 0.0 && age >= 0.0 && age <= jump_pitch_up_duration_) {
      jump_correction = jump_pitch_up_;
    }
    if (dir > 0 && jump_pitch_down_duration_ > 0.0 && age >= 0.0 && age <= jump_pitch_down_duration_) {
      jump_correction = -jump_pitch_down_;
    }
  }

  double pitch = -(current_traj.pitch + pitch_offset_ + jump_correction);
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

AimPoint Aimer::choose_aim_point(const Target & target)
{
  Eigen::VectorXd ekf_x = target.ekf_x();
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  auto armor_num = armor_xyza_list.size();
  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped) return {true, armor_xyza_list[0]};

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  // 不考虑小陀螺
  if (std::abs(target.ekf_x()[8]) <= 2 && target.name != ArmorName::outpost) {
    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);
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

  double coming_angle, leaving_angle;
  if (target.name == ArmorName::outpost) {
    coming_angle = outpost_comming_angle_;
    leaving_angle = outpost_leaving_angle_;
  } else {
    const double abs_w = std::abs(ekf_x[7]);
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
  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > coming_angle) continue;
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle) return {true, armor_xyza_list[i]};
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle) return {true, armor_xyza_list[i]};
  }

  return {false, armor_xyza_list[0]};
}

}  // namespace auto_aim