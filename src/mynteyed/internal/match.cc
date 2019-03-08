#include "mynteyed/internal/match.h"
#include "mynteyed/util/log.h"
#include "mynteyed/util/rate.h"

MYNTEYE_USE_NAMESPACE

Match::Match() {
}

Match::~Match() {
}

void Match::OnStreamDataCallback(const ImageType &type, const img_data_t &data) {
  std::lock_guard<std::recursive_mutex> _(match_mutex_);
  stream_datas_[type].push_back(data);
}

Match::img_datas_t Match::GetStreamDatas(const ImageType& type) {
  if (is_called_ == 0)
    InitOrder(type);

  std::lock_guard<std::recursive_mutex> _(match_mutex_);
  if (!stream_datas_[type].empty()) {
    auto&& datas = stream_datas_[type];
    switch (is_called_) {
      case 1:
        if (type == ImageType::IMAGE_LEFT_COLOR) {
          base_frame_id_ = datas.back().img->frame_id();
          return {datas.begin(), datas.end()};
        }
        return MatchStreamDatas(type);
      case 2:
        if (type == ImageType::IMAGE_RIGHT_COLOR) {
          base_frame_id_ = stream_datas_[type].back().img->frame_id();
          return {datas.begin(), datas.end()};
        }
        return MatchStreamDatas(type);
      case 3:
        if (type == ImageType::IMAGE_DEPTH) {
          base_frame_id_ = stream_datas_[type].back().img->frame_id();
          return {datas.begin(), datas.end()};
        }
        return MatchStreamDatas(type);
    }
  }

  return {};
}

/** match */
Match::img_datas_t Match::MatchStreamDatas(const ImageType& type) {
  std::lock_guard<std::recursive_mutex> _(match_mutex_);

  auto&& datas = stream_datas_[type];
  if (base_frame_id_ == datas.back().img->frame_id()) {
    return {datas.begin(), datas.end()};
  } else {
    for (auto it = datas.begin(); it != datas.end();) {
      if (base_frame_id_ == (*it).img->frame_id()) {
        std::vector<StreamData> result(datas.begin(), ++it);
        datas.erase(datas.begin(), it);
        return result;
      }
      ++it;
    }
  }

  return {};
}

void Match::InitOrder(const ImageType& type) {
  switch (type) {
    case ImageType::IMAGE_LEFT_COLOR:
      is_called_ = 1;
      return;
    case ImageType::IMAGE_RIGHT_COLOR:
      is_called_ = 2;
      return;
    case ImageType::IMAGE_DEPTH:
      is_called_ = 3;
      return;
    default:
      return;
  }
}


























