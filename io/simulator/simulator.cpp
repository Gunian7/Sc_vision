#include "simulator.hpp"

#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>

namespace io
{

static constexpr const char * SHM_NAME = "/simulator_frame";
static constexpr const char * SEM_NAME = "/simulator_sem";

/// Helper: read a scalar value from shared memory at a given offset.
template <typename T>
static T shm_read(const void * base, std::size_t offset) {
    T val;
    std::memcpy(&val, static_cast<const uint8_t *>(base) + offset, sizeof(T));
    return val;
}

SimulatorCamera::SimulatorCamera()
: shm_fd_(-1),
  shm_ptr_(nullptr),
  map_size_(0),
  width_(0),
  height_(0),
  valid_(false)
{
  // Open shared memory
  shm_fd_ = shm_open(SHM_NAME, O_RDWR, 0);
  if (shm_fd_ < 0) {
    throw std::runtime_error(
      "SimulatorCamera: failed to open shared memory " + std::string(SHM_NAME) +
      ". Is the simulator running?");
  }

  // Read header to get dimensions
  uint32_t hdr[2];
  if (::read(shm_fd_, &hdr, sizeof(hdr)) != static_cast<ssize_t>(sizeof(hdr))) {
    ::close(shm_fd_);
    throw std::runtime_error("SimulatorCamera: failed to read shm header");
  }
  width_ = hdr[0];
  height_ = hdr[1];

  // Protect against obviously invalid sizes
  if (width_ == 0 || height_ == 0 || width_ > 4096 || height_ > 4096) {
    ::close(shm_fd_);
    throw std::runtime_error(
      "SimulatorCamera: invalid dimensions in shm header (" +
      std::to_string(width_) + "x" + std::to_string(height_) + ")");
  }

  // Total size: offset 36 + pixel data
  map_size_ = ShmLayout::OFFSET_PIXEL_DATA + static_cast<std::size_t>(width_) * height_ * 3;

  // Mmap
  shm_ptr_ = ::mmap(nullptr, map_size_, PROT_READ | PROT_WRITE,
                    MAP_SHARED, shm_fd_, 0);
  if (shm_ptr_ == MAP_FAILED) {
    ::close(shm_fd_);
    throw std::runtime_error("SimulatorCamera: mmap failed");
  }

  valid_ = true;
}

SimulatorCamera::~SimulatorCamera()
{
  if (shm_ptr_ != nullptr && shm_ptr_ != MAP_FAILED) {
    ::munmap(shm_ptr_, map_size_);
  }
  if (shm_fd_ >= 0) {
    ::close(shm_fd_);
  }
}

void SimulatorCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  if (!valid_) {
    throw std::runtime_error("SimulatorCamera: not initialised");
  }

  // Open the semaphore
  sem_t * sem = ::sem_open(SEM_NAME, 0);
  if (sem == SEM_FAILED) {
    throw std::runtime_error("SimulatorCamera: sem_open failed");
  }

  // Wait for the writer to signal that a new frame is ready.
  ::sem_wait(sem);

  // Read timestamp from offset 8
  int64_t ts_ns = shm_read<int64_t>(shm_ptr_, ShmLayout::OFFSET_TIMESTAMP_NS);

  // Convert ns -> steady_clock::time_point.
  auto ts_sys = std::chrono::system_clock::time_point(
    std::chrono::nanoseconds(ts_ns));
  timestamp = std::chrono::steady_clock::now();  // best-effort

  // Build cv::Mat that wraps the shared memory (zero-copy until clone).
  const uint8_t * pixel_base = static_cast<const uint8_t *>(shm_ptr_) +
                               ShmLayout::OFFSET_PIXEL_DATA;
  cv::Mat raw(height_, width_, CV_8UC3, const_cast<uint8_t *>(pixel_base));

  // Clone so the caller owns the data independently of the shared memory.
  img = raw.clone();

  // Post semaphore to let the writer produce the next frame.
  ::sem_post(sem);
}

/// Read gimbal yaw from the shared memory (radians).
float read_gimbal_yaw(const void * shm_ptr) {
    return shm_read<float>(shm_ptr, ShmLayout::OFFSET_GIMBAL_YAW);
}

/// Read gimbal pitch from the shared memory (radians).
float read_gimbal_pitch(const void * shm_ptr) {
    return shm_read<float>(shm_ptr, ShmLayout::OFFSET_GIMBAL_PITCH);
}

/// Read gimbal roll from the shared memory (radians).
float read_gimbal_roll(const void * shm_ptr) {
    return shm_read<float>(shm_ptr, ShmLayout::OFFSET_GIMBAL_ROLL);
}

/// Read bullet speed from the shared memory (m/s).
float read_bullet_speed(const void * shm_ptr) {
    return shm_read<float>(shm_ptr, ShmLayout::OFFSET_BULLET_SPEED);
}

/// Read operating mode from the shared memory (int32_t).
int32_t read_mode(const void * shm_ptr) {
    return shm_read<int32_t>(shm_ptr, ShmLayout::OFFSET_MODE);
}

}  // namespace io
