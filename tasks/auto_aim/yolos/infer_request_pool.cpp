#include "infer_request_pool.hpp"

namespace auto_aim {

InferRequestPool::InferRequestPool(ov::CompiledModel & compiled_model, int num_requests)
{
  if (num_requests <= 0) num_requests = 1;
  requests_.reserve(num_requests);
  for (int i = 0; i < num_requests; ++i) {
    requests_.emplace_back(compiled_model.create_infer_request());
    free_indices_.push(i);
  }
}

int InferRequestPool::acquire()
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (free_indices_.empty()) return -1;
  int idx = free_indices_.front();
  free_indices_.pop();
  return idx;
}

ov::InferRequest & InferRequestPool::request(int index)
{
  return requests_.at(index);
}

void InferRequestPool::release(int index)
{
  std::lock_guard<std::mutex> lock(mtx_);
  free_indices_.push(index);
}

int InferRequestPool::free_count()
{
  std::lock_guard<std::mutex> lock(mtx_);
  return static_cast<int>(free_indices_.size());
}

// Lease implementation
InferRequestPool::Lease::Lease(Lease && other) noexcept
  : pool(other.pool), idx(other.idx)
{
  other.pool = nullptr;
  other.idx = -1;
}

InferRequestPool::Lease & InferRequestPool::Lease::operator=(Lease && other) noexcept
{
  if (this != &other) {
    // release current
    if (pool && idx >= 0) pool->release(idx);
    pool = other.pool;
    idx = other.idx;
    other.pool = nullptr;
    other.idx = -1;
  }
  return *this;
}

InferRequestPool::Lease::~Lease()
{
  if (pool && idx >= 0) {
    pool->release(idx);
  }
}

ov::InferRequest & InferRequestPool::Lease::request()
{
  return pool->request(idx);
}

InferRequestPool::Lease InferRequestPool::acquire_lease()
{
  int idx = acquire();
  if (idx < 0) return Lease();
  return Lease(this, idx);
}

} // namespace auto_aim
