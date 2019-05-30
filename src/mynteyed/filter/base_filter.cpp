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

#include "mynteyed/filter/base_filter.h"

MYNTEYE_USE_NAMESPACE

BaseFilter::BaseFilter() : _is_enable(false) {}

bool BaseFilter::LoadConfig(void* data) {
  std::cout << "config data: ";
  uint16_t *out_put_data = (uint16_t *)data;  // NOLINT
  for (size_t i = 0; i < MAX_CONFIG_LENGTH/2; i++) {
    std::cout << std::hex << *out_put_data++ << "|";
  }
  return false;
}

