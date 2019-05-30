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

#pragma once
#include <mutex>
#include <string>
#include <vector>
#include <memory>
#include "mynteyed/filter/base_filter.h"
#include "mynteyed/types.h"
#include "mynteyed/internal/camera_p.h"

MYNTEYE_BEGIN_NAMESPACE
class BaseFilter;

class FilterSpigot  : public std::enable_shared_from_this<FilterSpigot> {
 public:
  bool DeleteFilter(std::shared_ptr<BaseFilter> filter);
  bool DeleteFilter(const std::string &fltname);
  bool AddFilter(
      const std::string& filtname,
      std::shared_ptr<BaseFilter> filter,
      size_t location = 0);
  bool ProcessFrame(std::shared_ptr<Image> out, const std::shared_ptr<Image> in);  // NOLINT
  FilterSpigot();
 private:
  friend CameraPrivate;
  std::vector<std::shared_ptr<BaseFilter>> m_queue;
};

MYNTEYE_END_NAMESPACE

