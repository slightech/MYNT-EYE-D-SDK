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
#ifndef MYNTEYE_INTERNAL_ASYNC_CALLBACK_H_
#define MYNTEYE_INTERNAL_ASYNC_CALLBACK_H_
#pragma once

#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include "mynteyed/stubs/global.h"

MYNTEYE_BEGIN_NAMESPACE

template <typename T>
class AsyncCallback : public std::enable_shared_from_this<AsyncCallback<T>> {
 public:
  using callback_t = std::function<void(const T& data)>;
  using pointer = std::shared_ptr<AsyncCallback<T>>;

 private:
  /**
   * max_size > 0, cache size limit; otherwise, without limit.
   */
  AsyncCallback(callback_t callback, std::size_t max_size);

 public:
  ~AsyncCallback();

  template <typename... P>
  static pointer Create(P&&... all) {
    return pointer(new AsyncCallback<T>(std::forward<P>(all)...));
  }

  callback_t operator()() {
    return std::bind(&AsyncCallback<T>::OnCallback, this->shared_from_this(),
        std::placeholders::_1);
  }

  void OnCallback(const T& data);

 private:
  void Run();

  callback_t callback_;
  std::size_t max_size_;

  bool running_;
  std::thread thread_;

  std::deque<T> datas_;
  std::size_t count_;

  std::mutex mutex_;
  std::condition_variable condition_;
};

template <typename T>
AsyncCallback<T>::AsyncCallback(callback_t callback, std::size_t max_size)
  : callback_(callback), max_size_(max_size), running_(false), count_(0) {
  running_ = true;
  thread_ = std::thread(&AsyncCallback<T>::Run, this);
}

template <typename T>
AsyncCallback<T>::~AsyncCallback() {
  {
    std::lock_guard<std::mutex> _(mutex_);
    running_ = false;
    ++count_;
  }
  condition_.notify_one();
  if (thread_.joinable()) {
    thread_.join();
  }
}

template <typename T>
void AsyncCallback<T>::OnCallback(const T& data) {
  if (callback_ == nullptr) return;
  {
    std::lock_guard<std::mutex> _(mutex_);
    if (max_size_ > 0 && datas_.size() == max_size_) {
      datas_.pop_front();
    } else {
      ++count_;
    }
    datas_.push_back(data);
  }
  condition_.notify_one();
}

template <typename T>
void AsyncCallback<T>::Run() {
  std::deque<T> datas;
  while (running_) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this] { return count_ > 0; });
      if (!running_) break;
      datas = std::move(datas_);
      count_ = 0;
    }

    // callback_ != nullptr
    while (!datas.empty()) {
      // allow cost long time
      callback_(std::move(datas.front()));
      if (!running_) break;
      datas.pop_front();
    }
  }
  datas.clear();
}

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_ASYNC_CALLBACK_H_
