#ifndef MYNTEYE_INTERNAL_LOCATION_H_
#define MYNTEYE_INTERNAL_LOCATION_H_
#pragma once

#include <cstdint>
#include <functional>
#include <vector>
#include <mutex>

#include "mynteyed/data/types_internal.h"
#include "mynteyed/types.h"

MYNTEYE_BEGIN_NAMESPACE

class Location {
 public:
  using data_t = LocationData;
  using datas_t = std::vector<data_t>;

  using location_callback_t = std::function<void(const LocationData& data)>;

  Location();
  ~Location();

  /**↩
   * Enable location datas.
   *
   * If max_size <= 0, indicates only can get datas from callback.
   * If max_size > 0, indicates can get datas from callback or using GetLocationDatas().
   *
   * Note: if max_size > 0, the distance datas will be cached until you call GetLocationDatas().
  */
  void EnableLocationDatas(std::size_t max_size);
  void DisableLocationDatas();
  bool IsLocationDatasEnabled() const;

  datas_t GetLocationDatas();

  void SetLocationCallback(location_callback_t callback);

  void OnGPSDataCallback(const GPSDataPacket& packet);

 private:
  bool is_location_datas_enabled_;
  std::size_t location_datas_max_size_;

  datas_t location_datas_;

  std::mutex mutex_;

  location_callback_t location_callback_;

  std::uint32_t location_count_;
};

MYNTEYE_END_NAMESPACE

#endif // MYNTEYE_INTERNAL_LOCATION_H_
