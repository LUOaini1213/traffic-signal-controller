#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>

namespace tsc {

// Multi-producer / multi-consumer FIFO guarded by one mutex. `capacity` = 0 means unbounded;
// otherwise push() blocks while the queue is full (back-pressure on detector producers).
// close() wakes everyone: further pushes are rejected, pops drain what is left then return
// nullopt.
template <class T>
class BlockingQueue {
 public:
  explicit BlockingQueue(std::size_t capacity = 0) : capacity_(capacity) {}
  BlockingQueue(const BlockingQueue&) = delete;
  BlockingQueue& operator=(const BlockingQueue&) = delete;
  BlockingQueue(BlockingQueue&&) = delete;
  BlockingQueue& operator=(BlockingQueue&&) = delete;
  ~BlockingQueue() = default;

  // Returns false if the queue was closed (the value is dropped).
  bool push(T value) {
    std::unique_lock lock(mutex_);
    not_full_.wait(lock, [&] { return closed_ || capacity_ == 0 || items_.size() < capacity_; });
    if (closed_) return false;
    items_.push_back(std::move(value));
    lock.unlock();
    not_empty_.notify_one();
    return true;
  }

  std::optional<T> pop() {
    std::unique_lock lock(mutex_);
    not_empty_.wait(lock, [&] { return closed_ || !items_.empty(); });
    if (items_.empty()) return std::nullopt;
    T value = std::move(items_.front());
    items_.pop_front();
    lock.unlock();
    not_full_.notify_one();
    return value;
  }

  void close() {
    {
      std::lock_guard lock(mutex_);
      closed_ = true;
    }
    not_empty_.notify_all();
    not_full_.notify_all();
  }

  [[nodiscard]] std::size_t size() const {
    std::lock_guard lock(mutex_);
    return items_.size();
  }

  [[nodiscard]] bool closed() const {
    std::lock_guard lock(mutex_);
    return closed_;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable not_empty_;
  std::condition_variable not_full_;
  std::deque<T> items_;
  std::size_t capacity_;
  bool closed_ = false;
};

}  // namespace tsc
