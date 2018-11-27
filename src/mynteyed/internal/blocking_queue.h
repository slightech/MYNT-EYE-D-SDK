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
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "mynteyed/stubs/global.h"

MYNTEYE_BEGIN_NAMESPACE

template <typename T, typename Container = std::deque<T>>
class BlockingQueue {
 public:
  using size_type = typename Container::size_type;
  using iterator = typename Container::iterator;
  using const_iterator = typename Container::const_iterator;

  explicit BlockingQueue(size_type max_size = 0);

  // With lock

  void Put(const T& t);
  void Put(T&& t);

  T Take();  // block
  bool TryTake(T* t);

  T Peek();  // block
  bool TryPeek(T* t);

  Container TakeAll();  // block
  Container MoveAll();

  void Clear();

  bool Empty() const;
  size_type Size() const;

  // Without lock

  std::mutex& mutex() const {
    return mutex_;
  }

  bool empty() const { return queue_.empty(); }
  size_type size() const { return queue_.size(); }

  typename Container::reference back() { return queue_.back(); }
  typename Container::const_reference back() const { return queue_.back(); }

  typename Container::reference front() { return queue_.front(); }
  typename Container::const_reference front() const { return queue_.front(); }

  /*
  void push_back(const T& t) { queue_.push_back(t); }
  void push_back(T&& t) { queue_.push_back(std::move(t)); }
  void pop_front() { queue_.pop_front(); }

  void push_front(const T& t) { queue_.push_front(t); }
  void push_front(T&& t) { queue_.push_front(std::move(t)); }
  void pop_back() { queue_.pop_back(); }
  */

  iterator begin() { return queue_.begin(); }
  const_iterator begin() const { return queue_.begin(); }
  const_iterator cbegin() const { return queue_.cbegin(); }

  iterator end() { return queue_.end(); }
  const_iterator end() const { return queue_.end(); }
  const_iterator cend() const { return queue_.cend(); }

  iterator rbegin() { return queue_.rbegin(); }
  const_iterator rbegin() const { return queue_.rbegin(); }
  const_iterator crbegin() const { return queue_.crbegin(); }

  iterator rend() { return queue_.rend(); }
  const_iterator rend() const { return queue_.rend(); }
  const_iterator crend() const { return queue_.crend(); }

  iterator erase(const_iterator pos) { return queue_.erase(pos); }
  iterator erase(const_iterator first, const_iterator last) {
    return queue_.erase(first, last);
  }

 protected:
  void FixMaxSize();

  size_type max_size_;

  Container queue_;

  mutable std::mutex mutex_;
  std::condition_variable condition_;

  MYNTEYE_DISABLE_COPY(BlockingQueue)
  MYNTEYE_DISABLE_MOVE(BlockingQueue)
};

template <typename T, typename C>
BlockingQueue<T, C>::BlockingQueue(size_type max_size)
  : max_size_(std::move(max_size)) {
}

template <typename T, typename C>
void BlockingQueue<T, C>::Put(const T& t) {
  {
    std::lock_guard<std::mutex> _(mutex_);
    queue_.push_back(t);
    FixMaxSize();
  }
  condition_.notify_one();
}

template <typename T, typename C>
void BlockingQueue<T, C>::Put(T&& t) {
  {
    std::lock_guard<std::mutex> _(mutex_);
    queue_.push_back(std::move(t));
    FixMaxSize();
  }
  condition_.notify_one();
}

template <typename T, typename C>
T BlockingQueue<T, C>::Take() {
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [this] { return !queue_.empty(); });

  T t(std::move(queue_.front()));
  queue_.pop_front();
  return t;
}

template <typename T, typename C>
bool BlockingQueue<T, C>::TryTake(T* t) {
  std::lock_guard<std::mutex> _(mutex_);

  if (queue_.empty()) {
    return false;
  }

  *t = std::move(queue_.front());
  queue_.pop_front();
  return true;
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
C BlockingQueue<T, C>::TakeAll() {
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [this] { return !queue_.empty(); });

  return std::move(queue_);
}

template <typename T, typename C>
C BlockingQueue<T, C>::MoveAll() {
  std::unique_lock<std::mutex> lock(mutex_);
  return std::move(queue_);
}

template <typename T, typename C>
void BlockingQueue<T, C>::Clear() {
  std::lock_guard<std::mutex> _(mutex_);
  return queue_.clear();
}

template <typename T, typename C>
bool BlockingQueue<T, C>::Empty() const {
  std::lock_guard<std::mutex> _(mutex_);
  return queue_.empty();
}

template <typename T, typename C>
typename BlockingQueue<T, C>::size_type BlockingQueue<T, C>::Size() const {
  std::lock_guard<std::mutex> _(mutex_);
  return queue_.size();
}

template <typename T, typename C>
void BlockingQueue<T, C>::FixMaxSize() {
  if (max_size_ > 0 && queue_.size() > max_size_) {
    queue_.pop_front();  // drop first
  }
}

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_BLOCKING_QUEUE_H_
