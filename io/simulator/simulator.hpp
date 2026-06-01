#ifndef IO__SIMULATOR_HPP
#define IO__SIMULATOR_HPP

#include <chrono>
#include <cstdint>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <string>

#include "io/camera.hpp"
#include "io/command.hpp"

namespace io
{

/**
 * @brief Shared memory layout offsets (/simulator_frame).
 *
 * offset  size   field
 * 0       4      width  (uint32_t)
 * 4       4      height (uint32_t)
 * 8       8      timestamp_ns (int64_t, Unix epoch ns)
 * 16      4      gimbal_yaw   (float, rad)
 * 20      4      gimbal_pitch (float, rad)
 * 24      4      gimbal_roll  (float, rad)
 * 28      4      bullet_speed (float, m/s)
 * 32      4      mode         (int32_t: 0=idle,1=auto_aim,2=small_buff,3=big_buff,4=outpost)
 * 36      N      pixel data (RGB8, row-major, width*height*3 bytes)
 */
struct ShmLayout {
    static constexpr std::size_t OFFSET_TIMESTAMP_NS = 8;
    static constexpr std::size_t OFFSET_GIMBAL_YAW   = 16;
    static constexpr std::size_t OFFSET_GIMBAL_PITCH  = 20;
    static constexpr std::size_t OFFSET_GIMBAL_ROLL   = 24;
    static constexpr std::size_t OFFSET_BULLET_SPEED  = 28;
    static constexpr std::size_t OFFSET_MODE          = 32;
    static constexpr std::size_t OFFSET_PIXEL_DATA    = 36;
};

/**
 * @brief A camera that reads frames from POSIX shared memory written by the
 *        RoboMaster Simulator (daedalus).
 *
 * Shared memory layout (/simulator_frame):
 *   See ShmLayout above.
 *
 * Synchronisation uses a named semaphore (/simulator_sem).
 *
 * Run the simulator first, then launch Sc_vision.  If the shared memory
 * region does not exist, the constructor will throw.
 */
class SimulatorCamera : public CameraBase
{
public:
  SimulatorCamera();
  ~SimulatorCamera() override;

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  int shm_fd_;
  void * shm_ptr_;
  std::size_t map_size_;
  uint32_t width_;
  uint32_t height_;
  bool valid_;
};

/// Read gimbal yaw from shared memory (radians). Defined in simulator.cpp.
float read_gimbal_yaw(const void * shm_ptr);

/// Read gimbal pitch from shared memory (radians).
float read_gimbal_pitch(const void * shm_ptr);

/// Read gimbal roll from shared memory (radians).
float read_gimbal_roll(const void * shm_ptr);

/// Read bullet speed from shared memory (m/s).
float read_bullet_speed(const void * shm_ptr);

/// Read operating mode from shared memory (int32_t).
int32_t read_mode(const void * shm_ptr);

}  // namespace io

#endif  // IO__SIMULATOR_HPP
