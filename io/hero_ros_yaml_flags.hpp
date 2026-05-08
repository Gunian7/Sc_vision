#ifndef IO__HERO_ROS_YAML_FLAGS_HPP
#define IO__HERO_ROS_YAML_FLAGS_HPP

#include <string>

#include <yaml-cpp/yaml.h>

namespace io
{

inline bool yaml_hero_ros_suppress_rx_stall_log(const std::string &config_path)
{
  try
  {
    const auto yaml = YAML::LoadFile(config_path);
    if (yaml["hero_ros_suppress_rx_stall_log"])
    {
      return yaml["hero_ros_suppress_rx_stall_log"].as<bool>();
    }
  }
  catch (...)
  {
  }
  return false;
}

} // namespace io

#endif
