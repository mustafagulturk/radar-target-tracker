#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>

namespace radar {

/// Bounded multi-producer / multi-consumer queue.
///
/// When full, push() discards the *oldest* element: for a radar picture,
/// fresh data is worth more than stale data, and the producer (the network
/// receiver) must never block. close() wakes all waiting consumers so worker
/// threads can shut down cleanly.
template <typename T>
class ThreadSafeQueue {
public:
    explicit ThreadSafeQueue(std::size_t capacity) : capacity_(capacity == 0 ? 1 : capacity) {}

    ThreadSafeQueue(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

    /// Returns false if the queue is closed (the value is discarded).
    bool push(T value) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (closed_) {
                return false;
            }
            if (items_.size() >= capacity_) {
                items_.pop_front();
                ++dropped_;
            }
            items_.push_back(std::move(value));
        }
        not_empty_.notify_one();
        return true;
    }

    /// Blocks until an element is available or the queue is closed and drained.
    std::optional<T> pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        not_empty_.wait(lock, [this] { return closed_ || !items_.empty(); });
        return takeLocked();
    }

    /// Like pop(), but gives up after `timeout`.
    template <typename Rep, typename Period>
    std::optional<T> popFor(std::chrono::duration<Rep, Period> timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        not_empty_.wait_for(lock, timeout, [this] { return closed_ || !items_.empty(); });
        return takeLocked();
    }

    std::optional<T> tryPop() {
        std::lock_guard<std::mutex> lock(mutex_);
        return takeLocked();
    }

    void close() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
        }
        not_empty_.notify_all();
    }

    bool closed() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return closed_;
    }

    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return items_.size();
    }

    std::size_t capacity() const { return capacity_; }

    /// Number of elements discarded because the queue was full.
    std::size_t dropped() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_;
    }

private:
    std::optional<T> takeLocked() {
        if (items_.empty()) {
            return std::nullopt;
        }
        T value = std::move(items_.front());
        items_.pop_front();
        return value;
    }

    const std::size_t capacity_;
    mutable std::mutex mutex_;
    std::condition_variable not_empty_;
    std::deque<T> items_;
    std::size_t dropped_ = 0;
    bool closed_ = false;
};

}  // namespace radar
