#ifndef AUTO_AIM__HERO_COORDINATE_HPP
#define AUTO_AIM__HERO_COORDINATE_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <stdexcept>
#include <string_view>

namespace auto_aim
{
namespace hero_coord
{

/**
 * Hero 侧坐标工具：把「标定一次不变的量」和「每帧要更新的量」分开，用 lookup_transform
 * 统一查任意两坐标系之间的刚体变换。
*
 * 约定坐标系（右手系）：
 * - world：与 IMU/下位机姿态融合对齐的参考系（由 IMU 四元数 + R_gimbal2imubody 得到 R_gimbal2world）。
 * - gimbal：云台参考系原点；相机相对其平移 t_camera2gimbal，再经关节旋转 × 名义 R_camera2gimbal。
 * - camera：相机光心系，与 OpenCV solvePnP 物体点在相机系一致。
 *
 * lookup_transform(target, source) 返回 T，使得 p_target = T * p_source（齐次或 Isometry 作用在齐次坐标上）。
 */
class HeroCoordinateFrames
{
public:
  static constexpr std::string_view kWorld = "world";
  static constexpr std::string_view kGimbal = "gimbal";
  static constexpr std::string_view kCamera = "camera";

  /**
   * @param R_gimbal2imubody  YAML R_gimbal2imubody，与 Solver 一致
   * @param R_camera2gimbal_nominal  关节角为 0 时的相机→云台旋转（YAML R_camera2gimbal）
   * @param t_camera2gimbal  相机原点在云台系下平移（YAML t_camera2gimbal）
   * @param joint_pitch_axis_gimbal  关节绕轴（云台系下单位向量，未单位化时会归一化）
   */
  HeroCoordinateFrames(
    const Eigen::Matrix3d & R_gimbal2imubody, const Eigen::Matrix3d & R_camera2gimbal_nominal,
    const Eigen::Vector3d & t_camera2gimbal, const Eigen::Vector3d & joint_pitch_axis_gimbal)
  : R_gimbal2imubody_(R_gimbal2imubody),
    R_camera2gimbal_nominal_(R_camera2gimbal_nominal),
    t_camera2gimbal_(t_camera2gimbal),
    joint_axis_gimbal_(joint_pitch_axis_gimbal)
  {
    if (joint_axis_gimbal_.norm() < 1e-9) {
      joint_axis_gimbal_ = Eigen::Vector3d(0.0, 1.0, 0.0);
    } else {
      joint_axis_gimbal_.normalize();
    }
  }

  /** 动态：IMU 四元数（与 CBoard / Solver 同源）。 */
  void set_imu_quaternion(const Eigen::Quaterniond & q)
  {
    q_imu_ = q.normalized();
  }

  /** 动态：相机相对云台的关节 pitch（弧度），绕构造时给定的轴。 */
  void set_joint_pitch_rad(double rad) { joint_pitch_rad_ = rad; }

  /** 只读：当前 R_gimbal2world（与 Solver::set_R_gimbal2world 后内部一致）。 */
  Eigen::Matrix3d R_gimbal2world() const { return R_gimbal2world_from_imu(q_imu_, R_gimbal2imubody_); }

  /** 只读：当前有效相机→云台旋转（含关节）。 */
  Eigen::Matrix3d R_camera2gimbal_effective() const
  {
    return R_camera2gimbal_with_joint(
      R_camera2gimbal_nominal_, joint_pitch_rad_, joint_axis_gimbal_);
  }

  /**
   * p_target = lookup_transform(target, source) * p_source
   * @throws std::invalid_argument 未实现的 frame 对
   */
  Eigen::Isometry3d lookup_transform(std::string_view target, std::string_view source) const
  {
    if (source == target) {
      return Eigen::Isometry3d::Identity();
    }

    // 先构造边：gimbal<-world, camera<-gimbal，再组合
    const Eigen::Matrix3d Rgw = R_gimbal2world();

    Eigen::Isometry3d T_world_gimbal;
    T_world_gimbal.linear() = Rgw;
    T_world_gimbal.translation() = Eigen::Vector3d::Zero();

    Eigen::Isometry3d T_gimbal_camera;
    const Eigen::Matrix3d R_c2g = R_camera2gimbal_effective();
    T_gimbal_camera.linear() = R_c2g;
    T_gimbal_camera.translation() = t_camera2gimbal_;

    Eigen::Isometry3d T_world_camera = T_world_gimbal * T_gimbal_camera;

    auto inv = [](const Eigen::Isometry3d & t) { return t.inverse(); };

    if (target == kWorld && source == kGimbal) {
      return T_world_gimbal;
    }
    if (target == kGimbal && source == kWorld) {
      return inv(T_world_gimbal);
    }
    if (target == kGimbal && source == kCamera) {
      return T_gimbal_camera;
    }
    if (target == kCamera && source == kGimbal) {
      return inv(T_gimbal_camera);
    }
    if (target == kWorld && source == kCamera) {
      return T_world_camera;
    }
    if (target == kCamera && source == kWorld) {
      return inv(T_world_camera);
    }

    throw std::invalid_argument("hero_coord::lookup_transform: unknown frame pair");
  }

  /** 用变换将点从 source 表达到 target（与 lookup_transform 一致）。 */
  Eigen::Vector3d transform_point(
    std::string_view target, std::string_view source, const Eigen::Vector3d & p_source) const
  {
    return lookup_transform(target, source) * p_source;
  }

private:
  // —— 静态（标定）：长期不变 ——
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_nominal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Vector3d joint_axis_gimbal_;

  // —— 动态：每帧更新 ——
  Eigen::Quaterniond q_imu_{Eigen::Quaterniond::Identity()};
  double joint_pitch_rad_{0.0};

  static Eigen::Matrix3d R_gimbal2world_from_imu(
    const Eigen::Quaterniond & q_imu, const Eigen::Matrix3d & R_gimbal2imubody)
  {
    Eigen::Matrix3d R_imubody2imuabs = q_imu.toRotationMatrix();
    return R_gimbal2imubody.transpose() * R_imubody2imuabs * R_gimbal2imubody;
  }

  static Eigen::Matrix3d R_camera2gimbal_with_joint(
    const Eigen::Matrix3d & R_nominal, double joint_pitch_rad, const Eigen::Vector3d & axis_unit)
  {
    Eigen::AngleAxisd aa(joint_pitch_rad, axis_unit);
    return aa.toRotationMatrix() * R_nominal;
  }
};

// —— 无状态工具（与 Solver 公式一致，不经过 HeroCoordinateFrames 时仍可用）——

inline Eigen::Matrix3d R_gimbal2world_from_imu(
  const Eigen::Quaterniond & q_imu, const Eigen::Matrix3d & R_gimbal2imubody)
{
  Eigen::Matrix3d R_imubody2imuabs = q_imu.toRotationMatrix();
  return R_gimbal2imubody.transpose() * R_imubody2imuabs * R_gimbal2imubody;
}

inline Eigen::Matrix3d R_camera2gimbal_with_joint(
  const Eigen::Matrix3d & R_nominal, double joint_pitch_rad, const Eigen::Vector3d & axis_unit)
{
  Eigen::AngleAxisd aa(joint_pitch_rad, axis_unit);
  return aa.toRotationMatrix() * R_nominal;
}

inline Eigen::Vector3d point_camera_to_gimbal(
  const Eigen::Vector3d & p_cam, const Eigen::Matrix3d & R_cam2gimbal,
  const Eigen::Vector3d & t_cam2gimbal)
{
  return R_cam2gimbal * p_cam + t_cam2gimbal;
}

inline Eigen::Vector3d point_gimbal_to_world(
  const Eigen::Vector3d & p_gimbal, const Eigen::Matrix3d & R_gimbal2world)
{
  return R_gimbal2world * p_gimbal;
}

inline Eigen::Vector3d point_camera_to_world(
  const Eigen::Vector3d & p_cam, const Eigen::Matrix3d & R_cam2gimbal,
  const Eigen::Vector3d & t_cam2gimbal, const Eigen::Matrix3d & R_gimbal2world)
{
  return point_gimbal_to_world(point_camera_to_gimbal(p_cam, R_cam2gimbal, t_cam2gimbal), R_gimbal2world);
}

}  // namespace hero_coord
}  // namespace auto_aim

#endif
