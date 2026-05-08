#ifndef IO__MCU_MODE_HPP
#define IO__MCU_MODE_HPP

#include <string>
#include <vector>

namespace io {

/** 与下位机/communicate 协议一致；其中 outpost 在 IO 层会并入 auto_aim（见 normalize_mcu_mode_for_auto_aim）。 */
enum Mode { idle, auto_aim, small_buff, big_buff, outpost };
inline const std::vector<std::string> MODES = { "idle", "auto_aim", "small_buff", "big_buff", "outpost" };

/** 自瞄（含前哨）统一为 auto_aim；前哨战行为由识别器给出的 ArmorName::outpost 决定，不由单独 MCU 模式分支。 */
inline Mode normalize_mcu_mode_for_auto_aim(Mode m)
{
  if (m == outpost) {
    return auto_aim;
  }
  return m;
}

} // namespace io

#endif
