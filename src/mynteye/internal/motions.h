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
#ifndef MYNTEYE_INTERNAL_MOTIONS_H_
#define MYNTEYE_INTERNAL_MOTIONS_H_
#pragma once

#include <mutex>
#include <vector>

#include "mynteye/data/types_internal.h"
#include "mynteye/types.h"

MYNTEYE_BEGIN_NAMESPACE

class Motions {
 public:
  using data_t = MotionData;
  using datas_t = std::vector<data_t>;

  Motions();
  ~Motions();

  void EnableMotionDatas(std::size_t max_size);
  bool IsMotionDatasEnabled();

  datas_t GetMotionDatas();

  void ImuDataCallback(const ImuDataPacket &packet);

 private:
  bool is_motion_datas_enabled_;
  std::size_t motion_datas_max_size_;

  datas_t motion_datas_;

  std::mutex mtx_datas_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_MOTIONS_H_
