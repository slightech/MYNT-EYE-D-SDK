#ifndef MYNTEYE_INTERNAL_DISTANCE_H_
#define MYNTEYE_INTERNAL_DISTANCE_H_
#pragma once

#include <cstdint>
#include <functional>
#include <vector>
#include <mutex>

#include "mynteyed/data/types_internal.h"
#include "mynteyed/types.h"

MYNTEYE_BEGIN_NAMESPACE

class Distance {
 public:
  using data_t = DistanceData;
  using datas_t = std::vector<data_t>;

  using distance_callback_t = std::function<void(const DistanceData& data)>;

  Distance();
  ~Distance();

  /**
   * Enable distance datas.
   *
   * If max_size <= 0, indicates only can get datas from callback.
   * If max_size > 0, indicates can get datas from callback or using GetDistanceDatas().
   *
   * Note: if max_size > 0, the distance datas will be cached until you call GetDistanceDatas().
   */
  void EnableDistanceDatas(std::size_t max_size);
  void DisableDistanceDatas();
  bool IsDistanceDatasEnabled() const;

  datas_t GetDistanceDatas();

  void SetDistanceCallback(distance_callback_t callback);

  void OnDisDataCallback(const ObstacleDisPacket& packet);

 private:
  bool is_distance_datas_enabled_;
  std::size_t distance_datas_max_size_;

  datas_t distance_datas_;

  std::mutex mutex_;

  distance_callback_t distance_callback_;

  std::uint32_t distance_count_;
};

MYNTEYE_END_NAMESPACE

#endif // MYNTEYE_INTERNAL_LOCATION_H_
