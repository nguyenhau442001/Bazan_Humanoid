#ifndef HUMANOID_INFRASTRUCTURE__SHM_IPC_HPP_
#define HUMANOID_INFRASTRUCTURE__SHM_IPC_HPP_

#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdexcept>

namespace humanoid_infrastructure
{

/**
 * @brief High-throughput POSIX shared-memory wrapper for zero-copy IPC
 *        between perception accelerators and control coprocessors.
 */
template <typename T>
class SharedMemoryChannel
{
public:
  SharedMemoryChannel(const std::string & shm_name, bool is_creator)
  : name_(shm_name), is_creator_(is_creator), fd_(-1), ptr_(nullptr)
  {
    const size_t size = sizeof(T);
    if (is_creator_) {
      shm_unlink(name_.c_str());
      fd_ = shm_open(name_.c_str(), O_CREAT | O_RDWR, 0666);
      if (fd_ == -1) {
        throw std::runtime_error("Failed to create POSIX shared memory: " + name_);
      }
      if (ftruncate(fd_, size) == -1) {
        close(fd_);
        throw std::runtime_error("Failed to ftruncate POSIX shared memory");
      }
    } else {
      fd_ = shm_open(name_.c_str(), O_RDWR, 0666);
      if (fd_ == -1) {
        throw std::runtime_error("Failed to open POSIX shared memory: " + name_);
      }
    }

    void * addr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (addr == MAP_FAILED) {
      close(fd_);
      throw std::runtime_error("Failed to mmap shared memory");
    }
    ptr_ = static_cast<T *>(addr);
  }

  void write(const T & data)
  {
    if (ptr_) {
      std::memcpy(ptr_, &data, sizeof(T));
    }
  }

  void read(T & out_data) const
  {
    if (ptr_) {
      std::memcpy(&out_data, ptr_, sizeof(T));
    }
  }

  T * raw() { return ptr_; }
  const T * raw() const { return ptr_; }

  ~SharedMemoryChannel()
  {
    if (ptr_) {
      munmap(ptr_, sizeof(T));
    }
    if (fd_ != -1) {
      close(fd_);
    }
    if (is_creator_) {
      shm_unlink(name_.c_str());
    }
  }

private:
  std::string name_;
  bool is_creator_;
  int fd_;
  T * ptr_;
};

}  // namespace humanoid_infrastructure

#endif  // HUMANOID_INFRASTRUCTURE__SHM_IPC_HPP_
