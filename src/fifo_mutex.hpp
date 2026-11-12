#pragma once

#include <condition_variable>
#include <cstdint>
#include <mutex>

namespace lpf {
// SDK calls can exceed the controller period. Preserve arrival order so a
// sender cannot reacquire the session ahead of an already waiting reader.
class FifoMutex {
 public:
  void lock() {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto ticket = next_++;
    ready_.wait(lock, [&] { return ticket == serving_; });
  }
  void unlock() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++serving_;
    }
    ready_.notify_all();
  }

 private:
  std::mutex mutex_;
  std::condition_variable ready_;
  std::uint64_t next_ = 0, serving_ = 0;
};
}  // namespace lpf
