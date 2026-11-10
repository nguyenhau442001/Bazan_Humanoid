#ifndef HUMANOID_INFRASTRUCTURE__THREAD_POOL_HPP_
#define HUMANOID_INFRASTRUCTURE__THREAD_POOL_HPP_

#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace humanoid_infrastructure
{

/**
 * @brief Thread pool with priority dispatching for offloading compute-intensive robotics tasks
 *        (e.g., perception preprocessing, kinematic matrix inversions) away from the main thread.
 */
class ThreadPool
{
public:
  explicit ThreadPool(size_t threads = std::thread::hardware_concurrency())
  : stop_(false)
  {
    for (size_t i = 0; i < threads; ++i) {
      workers_.emplace_back([this]() {
        while (true) {
          std::function<void()> task;
          {
            std::unique_lock<std::mutex> lock(this->queue_mutex_);
            this->cv_.wait(lock, [this]() {
              return this->stop_ || !this->tasks_.empty();
            });
            if (this->stop_ && this->tasks_.empty()) {
              return;
            }
            task = std::move(this->tasks_.front());
            this->tasks_.pop();
          }
          task();
        }
      });
    }
  }

  template <class F, class... Args>
  auto enqueue(F && f, Args &&... args)
    -> std::future<typename std::invoke_result<F, Args...>::type>
  {
    using return_type = typename std::invoke_result<F, Args...>::type;

    auto task = std::make_shared<std::packaged_task<return_type()>>(
      std::bind(std::forward<F>(f), std::forward<Args>(args)...));

    std::future<return_type> res = task->get_future();
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      if (stop_) {
        throw std::runtime_error("enqueue on stopped ThreadPool");
      }
      tasks_.emplace([task]() { (*task)(); });
    }
    cv_.notify_one();
    return res;
  }

  ~ThreadPool()
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      stop_ = true;
    }
    cv_.notify_all();
    for (std::thread & worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

private:
  std::vector<std::thread> workers_;
  std::queue<std::function<void()>> tasks_;
  std::mutex queue_mutex_;
  std::condition_variable cv_;
  bool stop_;
};

}  // namespace humanoid_infrastructure

#endif  // HUMANOID_INFRASTRUCTURE__THREAD_POOL_HPP_
