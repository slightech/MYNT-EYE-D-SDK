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
#ifndef MYNTEYE_DEVICE_DATA_CACHES_H_
#define MYNTEYE_DEVICE_DATA_CACHES_H_
#pragma once

#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <vector>

#include "mynteyed/stubs/global.h"

MYNTEYE_BEGIN_NAMESPACE

class DataCaches {
 public:
  using size_t = std::size_t;
  using data_t = std::vector<std::uint8_t>;
  using data_ptr_t = std::shared_ptr<data_t>;
  using caches_t = std::list<data_ptr_t>;

  DataCaches();
  ~DataCaches();

  void SetProperSizes(std::set<size_t> sizes);

  // Get data with fixed size
  data_ptr_t GetFixed(const size_t& size);
  // Get data with proper size
  data_ptr_t GetProper(const size_t& size);

  void DebugPrint() const;

 private:
  data_ptr_t Create(const size_t& size);
  data_ptr_t Get(const size_t& size);

  std::map<size_t, caches_t> caches_map_;

  std::vector<size_t> proper_sizes_;

  std::mutex mutex_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_DATA_CACHES_H_
