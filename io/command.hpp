#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io
{
struct Command
{
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  double yaw_vel;
  double pitch_vel;
  double horizon_distance = 0;  //无人机专有
  double distance = 0;
};

/** 无目标：角速度、距离字段为零（配合 -Wmissing-field-initializers 使用）。 */
inline Command neutral_command() noexcept
{
  return Command{false, false, 0., 0., 0., 0., 0., 0.};
}

inline Command tracking_command(double yaw_rad, double pitch_rad) noexcept
{
  return Command{true, false, yaw_rad, pitch_rad, 0., 0., 0., 0.};
}

}  // namespace io

#endif  // IO__COMMAND_HPP