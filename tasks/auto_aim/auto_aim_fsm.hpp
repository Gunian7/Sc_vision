#ifndef AUTO_AIM__AUTO_AIM_FSM_HPP
#define AUTO_AIM__AUTO_AIM_FSM_HPP

#include <cmath>
#include <string>

#include <yaml-cpp/yaml.h>

namespace auto_aim
{

enum class AutoAimFsm : int
{
  AIM_SINGLE_ARMOR = 0,
  AIM_WHOLE_CAR_ARMOR = 1,
  AIM_WHOLE_CAR_PAIR = 2,
  AIM_WHOLE_CAR_CENTER = 3,
};

inline std::string auto_aim_fsm_to_string(AutoAimFsm state)
{
  switch (state) {
    case AutoAimFsm::AIM_SINGLE_ARMOR:
      return "AIM_SINGLE_ARMOR";
    case AutoAimFsm::AIM_WHOLE_CAR_ARMOR:
      return "AIM_WHOLE_CAR_ARMOR";
    case AutoAimFsm::AIM_WHOLE_CAR_PAIR:
      return "AIM_WHOLE_CAR_PAIR";
    case AutoAimFsm::AIM_WHOLE_CAR_CENTER:
      return "AIM_WHOLE_CAR_CENTER";
    default:
      return "AIM_SINGLE_ARMOR";
  }
}

class AutoAimFsmController
{
public:
  struct Params
  {
    int transfer_thresh = 8;
    double single_whole_up = 2.5;
    double single_whole_down = 1.8;
    double whole_pair_up = 5.5;
    double whole_pair_down = 4.5;
    double pair_center_up = 8.0;
    double pair_center_down = 6.5;

    void load(const YAML::Node & node)
    {
      if (!node || !node.IsMap()) {
        return;
      }
      if (node["transfer_thresh"].IsDefined()) {
        transfer_thresh = node["transfer_thresh"].as<int>();
      }
      if (node["single_whole_up"].IsDefined()) {
        single_whole_up = node["single_whole_up"].as<double>();
      }
      if (node["single_whole_down"].IsDefined()) {
        single_whole_down = node["single_whole_down"].as<double>();
      }
      if (node["whole_pair_up"].IsDefined()) {
        whole_pair_up = node["whole_pair_up"].as<double>();
      }
      if (node["whole_pair_down"].IsDefined()) {
        whole_pair_down = node["whole_pair_down"].as<double>();
      }
      if (node["pair_center_up"].IsDefined()) {
        pair_center_up = node["pair_center_up"].as<double>();
      }
      if (node["pair_center_down"].IsDefined()) {
        pair_center_down = node["pair_center_down"].as<double>();
      }
    }
  };

  AutoAimFsmController() = default;

  explicit AutoAimFsmController(const YAML::Node & node)
  {
    params_.load(node);
  }

  AutoAimFsm state() const
  {
    return fsm_state_;
  }

  void reset()
  {
    fsm_state_ = AutoAimFsm::AIM_SINGLE_ARMOR;
    overflow_count_ = 0;
  }

  void update(double angular_velocity, bool target_jumped)
  {
    if (!target_jumped) {
      reset();
      return;
    }

    const double abs_w = std::abs(angular_velocity);

    switch (fsm_state_) {
      case AutoAimFsm::AIM_SINGLE_ARMOR: {
        overflow_count_ = (abs_w > params_.single_whole_up) ? (overflow_count_ + 1) : 0;
        if (overflow_count_ > params_.transfer_thresh) {
          fsm_state_ = AutoAimFsm::AIM_WHOLE_CAR_ARMOR;
          overflow_count_ = 0;
        }
        break;
      }

      case AutoAimFsm::AIM_WHOLE_CAR_ARMOR: {
        if (abs_w > params_.whole_pair_up) {
          ++overflow_count_;
        } else if (abs_w < params_.single_whole_down) {
          --overflow_count_;
        } else {
          overflow_count_ = 0;
        }

        if (std::abs(overflow_count_) > params_.transfer_thresh) {
          fsm_state_ =
            (overflow_count_ > 0) ? AutoAimFsm::AIM_WHOLE_CAR_PAIR : AutoAimFsm::AIM_SINGLE_ARMOR;
          overflow_count_ = 0;
        }
        break;
      }

      case AutoAimFsm::AIM_WHOLE_CAR_PAIR: {
        if (abs_w > params_.pair_center_up) {
          ++overflow_count_;
        } else if (abs_w < params_.whole_pair_down) {
          --overflow_count_;
        } else {
          overflow_count_ = 0;
        }

        if (std::abs(overflow_count_) > params_.transfer_thresh) {
          fsm_state_ =
            (overflow_count_ > 0) ? AutoAimFsm::AIM_WHOLE_CAR_CENTER : AutoAimFsm::AIM_WHOLE_CAR_ARMOR;
          overflow_count_ = 0;
        }
        break;
      }

      case AutoAimFsm::AIM_WHOLE_CAR_CENTER: {
        overflow_count_ = (abs_w < params_.pair_center_down) ? (overflow_count_ + 1) : 0;
        if (overflow_count_ > params_.transfer_thresh) {
          fsm_state_ = AutoAimFsm::AIM_WHOLE_CAR_PAIR;
          overflow_count_ = 0;
        }
        break;
      }

      default: {
        reset();
        break;
      }
    }
  }

private:
  Params params_;
  AutoAimFsm fsm_state_ = AutoAimFsm::AIM_SINGLE_ARMOR;
  int overflow_count_ = 0;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AUTO_AIM_FSM_HPP
