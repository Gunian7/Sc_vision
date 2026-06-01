#ifndef IO__SIMULATOR_HPP
#define IO__SIMULATOR_HPP

#include <chrono>
#include <cstdint>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <string>

#include "io/camera.hpp"

namespace io
{

/**
 * @brief A camera that reads frames from POSIX shared memory written by the
 *        RoboMaster Simulator (daedalus).
 *
 * Shared memory layout (/simulator_frame):
 *   offset  size   field
 *   0       4      width  (uint32_t)
 *   4       4      height (uint32_t)
 *   8       8      timestamp_ns (int64_t, Unix epoch ns)
 *   16      N      pixel data (RGB8, row-major, width*height*3 bytes)
 *
 * Synchronisation uses a named semaphore (/simulator_sem).
 *
 * Run the simulator first, then launch Sc_vision.  If the shared memory
 * region does not exist, the constructor will throw.
 */
class SimulatorCamera : public CameraBase
{
public:
  /**
   * @param open_name   ignored; the shm path is hard-coded
   * @param config_path ignored; resolution is read from the shm header
   */
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

}  // namespace io

#endif  // IO__SIMULATOR_HPP
