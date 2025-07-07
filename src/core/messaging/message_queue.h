#pragma once

#ifndef _ESP_AI_MESSAGE_QUEUE_H_
#define _ESP_AI_MESSAGE_QUEUE_H_

#include <condition_variable>
#include <deque>
#include <mutex>

#include "message.h"

template <typename T>
class MessageQueue {
 public:
  void Send(Message<T>&& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    message_queue_.emplace_back(std::move(message));
    cv_.notify_all();
  }

  // Message<T> Recevie() {
  //   std::unique_lock<std::mutex> lock(mutex_);
  //   cv_.wait(lock, [this]() { return !message_queue_.empty(); });
  //   auto message = std::move(message_queue_.front());
  //   message_queue_.pop_front();
  //   return message;
  // }

  std::optional<Message<T>> Recevie(const bool block = true, const uint64_t timeout_ms = UINT64_MAX) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (block) {
      if (UINT64_MAX == timeout_ms) {
        cv_.wait(lock, [this]() { return !message_queue_.empty(); });
      } else {
        cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this]() { return !message_queue_.empty(); });
      }
    }

    if (message_queue_.empty()) {
      return std::nullopt;
    }

    auto message = std::move(message_queue_.front());
    message_queue_.pop_front();
    return message;
  }

  void Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    message_queue_.clear();
    cv_.notify_all();
  }

  size_t Size() {
    std::lock_guard<std::mutex> lock(mutex_);
    return message_queue_.size();
  }

 private:
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Message<T>> message_queue_;
};

#endif