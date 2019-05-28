#ifndef MYNTEYE_INTERNAL_MATCH_H_
#define MYNTEYE_INTERNAL_MATCH_H_
#pragma once

#include <vector>
#include <map>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include "mynteyed/data/types_internal.h"
#include "mynteyed/types.h"

MYNTEYE_BEGIN_NAMESPACE

enum class Order : std::int32_t {
  LEFT_IMAGE = 1,
  RIGHT_IMAGE,
  DEPTH_IMAGE,
  NONE,
};

class Match {
 public:
  using img_data_t = StreamData;
  using img_datas_t = std::vector<img_data_t>;

  Match();
  ~Match();

  void OnStreamDataCallback(const ImageType &type, const img_data_t& data);

  img_datas_t GetStreamDatas(const ImageType& type);

  void SetIRDepthStatus(const bool &enable);

  bool WaitForStreamData();

  bool HasStreamDatas(const ImageType &type);

  bool IsStreamDatasReady();

  void InitStreamKey(const bool &enable);

 protected:
  void OnUpdateMatchedDatas(const ImageType& type, const StreamData& data);
  img_datas_t MatchStreamDatas(const ImageType& type);
  void InitOrder(const ImageType& type);

 private:
  std::map<ImageType, img_datas_t> stream_datas_;

  std::recursive_mutex match_mutex_;

  std::uint16_t base_frame_id_ = 0;

  Order order_;

  bool is_ir_depth_only_ = false;

  std::condition_variable_any cs_;

  std::vector<ImageType> key_streams_;
};

MYNTEYE_END_NAMESPACE

#endif // MYNTEYE_INTERNAL_MATCH_H_
