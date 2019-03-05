#ifndef MYNTEYE_INTERNAL_MATCH_H_
#define MYNTEYE_INTERNAL_MATCH_H_
#pragma once

#include <vector>
#include <map>
#include <deque>
#include <mutex>
#include <condition_variable>

#include "mynteyed/data/types_internal.h"
#include "mynteyed/types.h"
#include "mynteyed/internal/streams.h"
#include "mynteyed/internal/blocking_queue.h"

MYNTEYE_BEGIN_NAMESPACE

class Streams;

class Match {
 public:
  template <typename T>
  using queue_t = BlockingQueue<T, std::deque<T>>;

  // img datas
  using img_data_queue_t = queue_t<img_data_t>;
  using img_data_queue_ptr_t = std::shared_ptr<img_data_queue_t>;

  using img_data_t = StreamData;
  using img_datas_t = std::vector<img_data_t>;

  explicit Match(const std::shared_ptr<Streams> &streams);
  ~Match();

  void OnStreamDataCallback(const img_data_t& data);

  void GetStreamData(const ImageType& type);

  img_datas_t GetStreamDatas(const ImageType& type);

 protected:
  void MatchSteamDatas(const ImageType& type);
  void OnUpdateMatchedDatas(const ImageType& type, const StreamData& data);

 pravite:
  std::shared_ptr<Streams> streams_;
  std::map<ImageType, img_datas_t> stream_datas_;
  std::map<ImageType, img_datas_t> stream_mathed_datas_;

  std::vector<int> v_offset_;

  /*
  std::recursive_mutex match_mutex_;
  std::recursive_mutex retrieve_mutex_;
  std::condition_variable_any cond_stream_datas_;
  */
  std::mutex match_mutex_;
  std::mutex retrieve_mutex_;
};

MYNTEYE_END_NAMESPACE

#endif // MYNTEYE_INTERNAL_MATCH_H_
