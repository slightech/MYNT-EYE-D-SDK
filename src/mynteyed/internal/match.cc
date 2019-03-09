#include "mynteyed/internal/match.h"
#include "mynteyed/util/log.h"

MYNTEYE_USE_NAMESPACE

Match::Match() :
  order_(Order::NONE) {
}

Match::~Match() {
}

void Match::OnStreamDataCallback(const ImageType &type, const img_data_t &data) {
  std::lock_guard<std::recursive_mutex> _(match_mutex_);
  stream_datas_[type].push_back(data);
}

Match::img_datas_t Match::GetStreamDatas(const ImageType& type) {
  if (order_ == Order::NONE)
    InitOrder(type);

  std::lock_guard<std::recursive_mutex> _(match_mutex_);
  if (!stream_datas_[type].empty()) {
    auto&& datas = stream_datas_[type];
    switch (order_) {
      case Order::LEFT_IMAGE:
        if (type == ImageType::IMAGE_LEFT_COLOR) {
          base_frame_id_ = datas.back().img->frame_id();
          return {datas.begin(), datas.end()};
        }
        return MatchStreamDatas(type);
      case Order::RIGHT_IMAGE:
        if (type == ImageType::IMAGE_RIGHT_COLOR) {
          base_frame_id_ = stream_datas_[type].back().img->frame_id();
          return {datas.begin(), datas.end()};
        }
        return MatchStreamDatas(type);
      case Order::DEPTH_IMAGE:
        if (type == ImageType::IMAGE_DEPTH) {
          base_frame_id_ = stream_datas_[type].back().img->frame_id();
          return {datas.begin(), datas.end()};
        }
        return MatchStreamDatas(type);
      default:
        throw_error("Unknow order of get datas.");
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
      order_ = Order::LEFT_IMAGE;
      return;
    case ImageType::IMAGE_RIGHT_COLOR:
      order_ = Order::RIGHT_IMAGE;
      return;
    case ImageType::IMAGE_DEPTH:
      order_ = Order::DEPTH_IMAGE;
      return;
    default:
      return;
  }
}


























