#ifndef MYNTEYE_INTERNAL_MATCH_H_
#define MYNTEYE_INTERNAL_MATCH_H_
#pragma once

#include <vector>
#include <map>
#include <mutex>
#include <thread>

#include "mynteyed/data/types_internal.h"
#include "mynteyed/types.h"

MYNTEYE_BEGIN_NAMESPACE

class Match {
 public:
  using img_data_t = StreamData;
  using img_datas_t = std::vector<img_data_t>;

  Match();
  ~Match();

  void OnStreamDataCallback(const ImageType &type, const img_data_t& data);

  img_datas_t GetStreamDatas(const ImageType& type);

  void Start();

 protected:
  void OnUpdateMatchedDatas(const ImageType& type, const StreamData& data);
  void MatchStreamDatas();

 private:
  std::map<ImageType, img_datas_t> stream_datas_;
  std::map<ImageType, img_datas_t> stream_matched_datas_;

  std::vector<int> v_offset_;

  std::mutex match_mutex_;
  std::mutex retrieve_mutex_;

  bool is_matching_;
  std::thread match_thread_;
};

MYNTEYE_END_NAMESPACE

#endif // MYNTEYE_INTERNAL_MATCH_H_
