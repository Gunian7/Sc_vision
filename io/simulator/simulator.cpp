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

  map_size_ = 16 + static_cast<std::size_t>(width_) * height_ * 3;

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
  // The writer posts the semaphore after writing; we wait here.
  ::sem_wait(sem);

  // Read timestamp from offset 8
  int64_t ts_ns;
  std::memcpy(&ts_ns, static_cast<const uint8_t *>(shm_ptr_) + 8, sizeof(ts_ns));

  // Convert ns -> steady_clock::time_point.
  // Use system_clock epoch as bridge (both writer and reader run on the same
  // machine so CLOCK_REALTIME is consistent).
  auto ts_sys = std::chrono::system_clock::time_point(
    std::chrono::nanoseconds(ts_ns));
  timestamp = std::chrono::steady_clock::now();  // best-effort: we don't have a
                                                  // monotonic ns from the writer.
  (void)ts_sys;  // Could be used for synchronisation if needed.

  // Build cv::Mat that wraps the shared memory (zero-copy until clone).
  // Layout in shm: offset 16 = pixel data, rows = height, cols = width, RGB8.
  const uint8_t * pixel_base = static_cast<const uint8_t *>(shm_ptr_) + 16;
  cv::Mat raw(height_, width_, CV_8UC3, const_cast<uint8_t *>(pixel_base));

  // Clone so the caller owns the data independently of the shared memory.
  img = raw.clone();

  // Post semaphore to let the writer produce the next frame.
  ::sem_post(sem);
}

}  // namespace io
