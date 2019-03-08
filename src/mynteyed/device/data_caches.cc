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
#include "mynteyed/device/data_caches.h"

#include <algorithm>
#include <iostream>
#include <iterator>

#include "mynteyed/util/log.h"

// #define CACHES_INFO_PRINT
#define CACHES_EACH_MAX_SIZE 12

MYNTEYE_USE_NAMESPACE

DataCaches::DataCaches() {
}

DataCaches::~DataCaches() {
}

void DataCaches::SetProperSizes(std::set<size_t> sizes) {
  proper_sizes_ = std::vector<size_t>{sizes.begin(), sizes.end()};
  std::sort(proper_sizes_.begin(), proper_sizes_.end());

#ifdef CACHES_INFO_PRINT
  std::cout << "proper_sizes: ";
  std::copy(proper_sizes_.begin(), proper_sizes_.end(),
      std::ostream_iterator<size_t>(std::cout, ","));
  std::cout << std::endl;
#endif
}

DataCaches::data_ptr_t DataCaches::GetFixed(const size_t& size) {
  std::lock_guard<std::mutex> _(mutex_);
  return Get(size);
}

DataCaches::data_ptr_t DataCaches::GetProper(const size_t& size) {
  std::lock_guard<std::mutex> _(mutex_);
  auto&& it = std::find_if(proper_sizes_.begin(), proper_sizes_.end(),
      [&size](const size_t& proper_size) { return proper_size >= size; });
  if (it == proper_sizes_.end()) {
    LOGW("GetProper size: %d", size);
    throw_error("The size is larger then all proper sizes, unaccepted");
  }
#ifdef CACHES_INFO_PRINT
  LOGI("GetProper size: %d, proper_size: %d", size, *it);
#endif
  return Get(*it);
}

DataCaches::data_ptr_t DataCaches::Create(const size_t& size) {
#ifdef CACHES_INFO_PRINT
  LOGI("Create data: %d", size);
#endif
  return std::make_shared<data_t>(size);
}

DataCaches::data_ptr_t DataCaches::Get(const size_t& size) {
  auto&& caches = caches_map_[size];
  if (!caches.empty()) {
    for (auto&& data : caches) {
      if (data.use_count() > 1) {
        continue;
      }
#ifdef CACHES_INFO_PRINT
      LOGI("Get cache: %d", size);
#endif
      return data;
    }
  }
  /*
  if (caches.size() > CACHES_EACH_MAX_SIZE) {
    LOGW("Caches(%d) is too large", size);
  }
  */
  auto&& data = Create(size);
  caches.push_back(data);
  return data;
}

void DataCaches::DebugPrint() const {
  std::cout << "DataCaches" << std::endl;
  for (auto&& entry : caches_map_) {
    std::cout << "  size: " << entry.first
        << ", count: " << entry.second.size()
        << std::endl;
  }
}
