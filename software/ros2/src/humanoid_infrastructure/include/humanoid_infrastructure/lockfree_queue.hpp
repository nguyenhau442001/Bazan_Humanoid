#ifndef HUMANOID_INFRASTRUCTURE__LOCKFREE_QUEUE_HPP_
#define HUMANOID_INFRASTRUCTURE__LOCKFREE_QUEUE_HPP_

#include <atomic>
#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

namespace humanoid_infrastructure
{

/**
 * @brief Lock-free Single-Producer Single-Consumer (SPSC) circular queue.
 * @details Designed for low-latency zero-allocation streaming between 1kHz RT joint control
 *          and asynchronous tasks.
 */
template <typename T, size_t Capacity = 1024>
class SPSCQueue
{
  static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be a power of two");

public:
  SPSCQueue() : head_(0), tail_(0)
  {
    buffer_.resize(Capacity);
  }

  bool push(const T & item)
  {
    const size_t current_tail = tail_.load(std::memory_order_relaxed);
    if ((current_tail - head_.load(std::memory_order_acquire)) == Capacity) {
      return false; // Queue full
    }
    buffer_[current_tail & (Capacity - 1)] = item;
    tail_.store(current_tail + 1, std::memory_order_release);
    return true;
  }

  bool push(T && item)
  {
    const size_t current_tail = tail_.load(std::memory_order_relaxed);
    if ((current_tail - head_.load(std::memory_order_acquire)) == Capacity) {
      return false; // Queue full
    }
    buffer_[current_tail & (Capacity - 1)] = std::move(item);
    tail_.store(current_tail + 1, std::memory_order_release);
    return true;
  }

  std::optional<T> pop()
  {
    const size_t current_head = head_.load(std::memory_order_relaxed);
    if (current_head == tail_.load(std::memory_order_acquire)) {
      return std::nullopt; // Queue empty
    }
    T item = std::move(buffer_[current_head & (Capacity - 1)]);
    head_.store(current_head + 1, std::memory_order_release);
    return item;
  }

  bool empty() const
  {
    return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
  }

  size_t size() const
  {
    const size_t head = head_.load(std::memory_order_acquire);
    const size_t tail = tail_.load(std::memory_order_acquire);
    return (tail >= head) ? (tail - head) : (Capacity - (head - tail));
  }

private:
  alignas(64) std::atomic<size_t> head_;
  alignas(64) std::atomic<size_t> tail_;
  std::vector<T> buffer_;
};

}  // namespace humanoid_infrastructure

#endif  // HUMANOID_INFRASTRUCTURE__LOCKFREE_QUEUE_HPP_
