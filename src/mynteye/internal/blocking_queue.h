// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef MYNTEYE_INTERNAL_BLOCKING_QUEUE_H_
#define MYNTEYE_INTERNAL_BLOCKING_QUEUE_H_
#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <utility>

#include "mynteye/stubs/global.h"

MYNTEYE_BEGIN_NAMESPACE

template <typename T, typename Container = std::queue<T>>
class BlockingQueue {
 public:
  explicit BlockingQueue(std::size_t max_size = 0);

  void Put(const T& t);
  void Put(T&& t);

  T Take();
  bool TryTake(T* t);

  Container TakeAll();

  T Peek();
  bool TryPeek(T* t);

  bool Empty() const;
  std::size_t Size() const;

 protected:
  void FixMaxSize();

  std::size_t max_size_;

  mutable std::mutex mutex_;
  std::condition_variable condition_;

  Container queue_;

  MYNTEYE_DISABLE_COPY(BlockingQueue)
  MYNTEYE_DISABLE_MOVE(BlockingQueue)
};

template <typename T, typename C>
BlockingQueue<T, C>::BlockingQueue(std::size_t max_size)
  : max_size_(std::move(max_size)) {
}

template <typename T, typename C>
void BlockingQueue<T, C>::Put(const T& t) {
  {
    std::lock_guard<std::mutex> _(mutex_);
    queue_.push(t);
    FixMaxSize();
  }
  condition_.notify_one();
}

template <typename T, typename C>
void BlockingQueue<T, C>::Put(T&& t) {
  {
    std::lock_guard<std::mutex> _(mutex_);
    queue_.push(std::move(t));
    FixMaxSize();
  }
  condition_.notify_one();
}

template <typename T, typename C>
T BlockingQueue<T, C>::Take() {
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [this] { return !queue_.empty(); });

  T t(std::move(queue_.front()));
  queue_.pop();
  return t;
}

template <typename T, typename C>
bool BlockingQueue<T, C>::TryTake(T* t) {
  std::lock_guard<std::mutex> _(mutex_);

  if (queue_.empty()) {
    return false;
  }

  *t = std::move(queue_.front());
  queue_.pop();
  return true;
}

template <typename T, typename C>
C BlockingQueue<T, C>::TakeAll() {
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [this] { return !queue_.empty(); });

  return std::move(queue_);
}

template <typename T, typename C>
T BlockingQueue<T, C>::Peek() {
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [this] { return !queue_.empty(); });

  return queue_.front();
}

template <typename T, typename C>
bool BlockingQueue<T, C>::TryPeek(T* t) {
  std::lock_guard<std::mutex> _(mutex_);

  if (queue_.empty()) {
    return false;
  }

  *t = queue_.front();
  return true;
}

template <typename T, typename C>
bool BlockingQueue<T, C>::Empty() const {
  std::lock_guard<std::mutex> _(mutex_);
  return queue_.empty();
}

template <typename T, typename C>
std::size_t BlockingQueue<T, C>::Size() const {
  std::lock_guard<std::mutex> _(mutex_);
  return queue_.size();
}

template <typename T, typename C>
void BlockingQueue<T, C>::FixMaxSize() {
  if (max_size_ > 0 && queue_.size() > max_size_) {
    queue_.pop();  // drop first
  }
}

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_BLOCKING_QUEUE_H_
