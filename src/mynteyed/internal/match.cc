#include "mynteyed/internal/match.h"

Match::Match(const std::shared_ptr<Streams> streams) :
  streams_(streams),
  who_first_(0) {
}

Match::~Match() {
}

void Match::OnStreamDataCallback(const ImageType& type, const img_data_t& data) {
  std::lock_guard<std::mutex> _(mutex);
  stream_datas_[type].push_back(data);
}

Match::img_data_t Match::GetStreamData(const ImageType& type) {
  auto datas = GetSteamdatas(type);
  if (datas.empty()) return {};
  return std::move(datas.back());
}

Match::img_datas_t Match::GetStreamDatas(const ImageType& type) {
  InitFirstCalled(type);

  std::lock_guard<std::mutex> _(retrieve_mutex);
  if (!stream_matched_datas_[type].empty()) {
    auto datas = stream_matched_datas_[type];
    stream_matched_datas_[type].clear();
    return datas;
  }
}

/** match */
void Match::MatchStreamDatas() {
  std::lock_guard<std::mutex> _(match_mutex_);

  auto&& left_datas = stream_datas_[ImageType::IMAGE_LEFT_COLOR];
  auto&& right_datas = stream_datas_[ImageType::IMAGE_RIGHT_COLOR]
  auto&& depth_datas = stream_datas_[ImageType::IMAGE_DEPTH];

  if (left_datas.empty() || depth_datas.empty()) {
    return;
  }

  bool next = false;
  int offset = 0;
  /** start matching (only left match with depth)*/
  for (auto left_it = left_datas.begin(); left_it != left_datas.end()) {
    next = true;
    offset++;
    auto left_frame_id = (*left_it)->frame_id();
    for (auto depth_it = depth_datas.begin(); depth_it != depth_datas.end()) {
      if (left_frame_id == (*depth_it)->frame_id()) {
        next = false;
        OnUpdateMatchedDatas(ImageType::IMAGE_LEFT_COLOR, (*left_it));
        if (!right_datas.empty()) {
          auto data = right_datas[offset - 1];
          v_offset_.push_back(offset);
          OnUpdateMatchedDatas(ImageType::IMAGE_RIGHT_COLOR, data);
        }
        OnUpdateMatchedDatas(ImageType::IMAGE_DEPTH, (*depth_it));
        left_it = left_datas.erase(left_it);
        depth_it = depth_datas.erase(depth_it);
        break;
      }
      ++depth_it;
    }
    if (next) ++left_it;
  }

  /** erase right color */
  for (auto i : v_offset_) {
    right_datas.erase(right_datas.begin() + i - 1,
        right_datas.begin() + i);
  }
  v_offset_.clear();
}

void Match::OnUpdateMatchedDatas(const ImageType& type, const StreamData& data) {
  std::lock_guard<std::mutex> _(retrieve_mutex_);
  stream_matched_datas_[type].push_back(data);
}


