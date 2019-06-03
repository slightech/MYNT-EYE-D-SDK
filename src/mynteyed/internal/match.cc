#include <functional>

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
  if (stream_datas_[type].size() > 10)
    stream_datas_[type].clear();
  stream_datas_[type].push_back(data);
  cs_.notify_one();
}

Match::img_datas_t Match::GetStreamDatas(const ImageType& type) {
  std::lock_guard<std::recursive_mutex> _(match_mutex_);
  if (is_ir_depth_only_) {
    auto datas = stream_datas_[type];
    stream_datas_[type].clear();
    return datas;
  }

  if (order_ == Order::NONE)
    InitOrder(type);

  if (!stream_datas_[type].empty()) {
    auto datas = stream_datas_[type];
    switch (order_) {
      case Order::LEFT_IMAGE:
        if (type == ImageType::IMAGE_LEFT_COLOR) {
          base_frame_id_ = datas.back().img->frame_id();
          stream_datas_[type].clear();
          return datas;
        }
        return MatchStreamDatas(type);
      case Order::RIGHT_IMAGE:
        if (type == ImageType::IMAGE_RIGHT_COLOR) {
          base_frame_id_ = datas.back().img->frame_id();
          stream_datas_[type].clear();
          return datas;
        }
        return MatchStreamDatas(type);
      case Order::DEPTH_IMAGE:
        if (type == ImageType::IMAGE_DEPTH) {
          base_frame_id_ = datas.back().img->frame_id();
          stream_datas_[type].clear();
          return datas;
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

  std::vector<StreamData> result;

  auto&& datas = stream_datas_[type];
  if (base_frame_id_ == datas.back().img->frame_id()) {
    result = {datas.begin(), datas.end()};
    datas.clear();
    return result;
  } else {
    for (auto it = datas.begin(); it != datas.end();) {
      if (base_frame_id_ == (*it).img->frame_id()) {
        result = {datas.begin(), ++it};
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

void Match::SetIRDepthStatus(const bool &enable) {
  is_ir_depth_only_ = enable;
}

bool Match::HasStreamDatas(const ImageType &type) {
  std::lock_guard<std::recursive_mutex> _(match_mutex_);
  return stream_datas_.find(type) != stream_datas_.end() &&
    !stream_datas_[type].empty();
}

bool Match::WaitForStreamData() {
  std::unique_lock<std::recursive_mutex> _(match_mutex_);
  auto ready = std::bind(&Match::IsStreamDatasReady, this);
  bool ok = cs_.wait_for(_, std::chrono::seconds(1), ready);
  if (!ok) {
    return false;
  }
  return true;
}

bool Match::IsStreamDatasReady() {
  for (auto &type : key_streams_) {
    if (!HasStreamDatas(type))
      return false;
  }

  return true;
}

void Match::InitStreamKey(const bool &enable) {
  if (enable) {
    key_streams_ = {ImageType::IMAGE_DEPTH};
  } else {
    key_streams_ = {ImageType::IMAGE_LEFT_COLOR};
  }
}



























