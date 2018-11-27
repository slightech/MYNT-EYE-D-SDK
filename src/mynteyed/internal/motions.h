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

#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

#include "mynteyed/data/types_internal.h"
#include "mynteyed/types.h"

MYNTEYE_BEGIN_NAMESPACE

class Motions {
 public:
  using data_t = MotionData;
  using datas_t = std::vector<data_t>;

  using motion_callback_t = std::function<void(const MotionData& data)>;

  Motions();
  ~Motions();

  void SetMotionIntrinsics(const std::shared_ptr<MotionIntrinsics>& ex);

  void EnableProcessMode(const std::int32_t& mode);

  /**
   * Enable motion datas.
   *
   * If max_size <= 0, indicates only can get datas from callback.
   * If max_size > 0, indicates can get datas from callback or using GetMotionDatas().
   *
   * Note: if max_size > 0, the motion datas will be cached until you call GetMotionDatas().
   */
  void EnableMotionDatas(std::size_t max_size);
  void DisableMotionDatas();
  bool IsMotionDatasEnabled() const;

  datas_t GetMotionDatas();

  void SetMotionCallback(motion_callback_t callback);

  void OnImuDataCallback(const ImuDataPacket& packet);

 private:
  void ProcImuAssembly(std::shared_ptr<ImuData> data) const;
  void ProcImuTempDrift(std::shared_ptr<ImuData> data) const;

  std::shared_ptr<MotionIntrinsics> motion_intrinsics_;

  std::int32_t proc_mode_;

  bool is_motion_datas_enabled_;
  std::size_t motion_datas_max_size_;

  datas_t motion_datas_;

  std::mutex metux_;

  motion_callback_t motion_callback_;

  std::uint32_t motion_count_;
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_INTERNAL_MOTIONS_H_
