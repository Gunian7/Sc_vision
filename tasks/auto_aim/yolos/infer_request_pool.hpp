#ifndef AUTO_AIM__INFER_REQUEST_POOL_HPP
#define AUTO_AIM__INFER_REQUEST_POOL_HPP

#include <openvino/openvino.hpp>
#include <vector>
#include <queue>
#include <mutex>
#include <memory>

namespace auto_aim {

class InferRequestPool {
public:
  InferRequestPool() = delete;
  InferRequestPool(ov::CompiledModel & compiled_model, int num_requests);

  // Non-blocking acquire: returns -1 if none available
  int acquire();

  // Get reference to infer request at index (valid after acquire)
  ov::InferRequest & request(int index);

  // Release previously acquired index
  void release(int index);

  int free_count();

  // RAII lease: holds an acquired request index and releases on destruction
  struct Lease {
    Lease() : pool(nullptr), idx(-1) {}
    // non-copyable
    Lease(const Lease &) = delete;
    Lease & operator=(const Lease &) = delete;
    // movable
    Lease(Lease && other) noexcept;
    Lease & operator=(Lease && other) noexcept;
    ~Lease();

    bool valid() const { return pool != nullptr && idx >= 0; }

    ov::InferRequest & request();

  private:
    friend class InferRequestPool;
    explicit Lease(InferRequestPool * pool, int idx) : pool(pool), idx(idx) {}
    InferRequestPool * pool;
    int idx;
  };

  // Acquire a Lease (non-blocking). If no free request, returned Lease is invalid.
  Lease acquire_lease();

private:
  std::vector<ov::InferRequest> requests_;
  std::queue<int> free_indices_;
  std::mutex mtx_;
};

} // namespace auto_aim

#endif // AUTO_AIM__INFER_REQUEST_POOL_HPP
