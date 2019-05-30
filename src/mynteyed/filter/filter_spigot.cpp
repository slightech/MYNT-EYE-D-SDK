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

#include "mynteyed/filter/filter_spigot.h"
#include "mynteyed/device/image.h"

MYNTEYE_USE_NAMESPACE

FilterSpigot::FilterSpigot() {}

bool FilterSpigot::DeleteFilter(std::shared_ptr<BaseFilter> filter) {
  return false;
}

bool FilterSpigot::DeleteFilter(const std::string &fltname) {
  return false;
}

bool FilterSpigot::AddFilter(
    const std::string& filtname,
    std::shared_ptr<BaseFilter> filter,
    size_t location) {
  return false;
}

bool FilterSpigot::ProcessFrame(
    std::shared_ptr<Image> out, const std::shared_ptr<Image> in) {
  if (out == in) {}
  return false;
}

