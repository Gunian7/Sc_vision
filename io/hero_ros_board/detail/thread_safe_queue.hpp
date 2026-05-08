#ifndef IO__HERO_ROS_BOARD__DETAIL__THREAD_SAFE_QUEUE_HPP
#define IO__HERO_ROS_BOARD__DETAIL__THREAD_SAFE_QUEUE_HPP

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>

namespace io::detail
{

template <typename T, bool PopWhenFull = false>
class ThreadSafeQueue
{
public:
  explicit ThreadSafeQueue(size_t max_size, std::function<void(void)> full_handler = [] {})
      : max_size_(max_size), full_handler_(std::move(full_handler))
  {
  }

  void push(const T &value)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (queue_.size() >= max_size_)
    {
      if constexpr (PopWhenFull)
      {
        queue_.pop();
      }
      else
      {
        full_handler_();
        return;
      }
    }
    queue_.push(value);
    not_empty_condition_.notify_all();
  }

  void pop(T &value)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });
    if (queue_.empty())
    {
      std::cerr << "Error: Attempt to pop from an empty queue." << std::endl;
      return;
    }
    value = queue_.front();
    queue_.pop();
  }

  T pop()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });
    T value = std::move(queue_.front());
    queue_.pop();
    return value;
  }

  bool empty()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
  }

private:
  std::queue<T> queue_;
  size_t max_size_;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_condition_;
  std::function<void(void)> full_handler_;
};

} // namespace io::detail

#endif
