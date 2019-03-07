#include "mynteyed/internal/match.h"
#include "mynteyed/util/log.h"
#include "mynteyed/util/rate.h"

static const std::size_t MAX_CACHE_TOTAL = 40;

MYNTEYE_USE_NAMESPACE

Match::Match() : is_matching_(false) {
}

Match::~Match() {
  is_matching_ = false;
  if (match_thread_.joinable()) {
    match_thread_.join();
  }
}

void Match::OnStreamDataCallback(const ImageType &type, const img_data_t &data) {
  std::lock_guard<std::mutex> _(match_mutex_);
  stream_datas_[type].push_back(data);
}

Match::img_datas_t Match::GetStreamDatas(const ImageType& type) {
  std::lock_guard<std::mutex> _(retrieve_mutex_);
  if (!stream_matched_datas_[type].empty()) {
    auto datas = stream_matched_datas_[type];
    stream_matched_datas_[type].clear();
    return datas;
  }

  return {};
}

/** match */
void Match::MatchStreamDatas() {
  std::lock_guard<std::mutex> _(match_mutex_);

  auto&& left_datas = stream_datas_[ImageType::IMAGE_LEFT_COLOR];
  auto&& right_datas = stream_datas_[ImageType::IMAGE_RIGHT_COLOR];
  auto&& depth_datas = stream_datas_[ImageType::IMAGE_DEPTH];

  if (left_datas.empty() || depth_datas.empty()) {
    return;
  }

  bool next = false;
  int offset = 0;
  /** start matching (only left match with depth)*/
  for (auto left_it = left_datas.begin(); left_it != left_datas.end();) {
    next = true;
    offset++;
    auto left_frame_id = (*left_it).img->frame_id();
    for (auto depth_it = depth_datas.begin(); depth_it != depth_datas.end();) {
      if (left_frame_id == (*depth_it).img->frame_id()) {
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
  if (stream_matched_datas_[type].size() > MAX_CACHE_TOTAL)
    stream_matched_datas_[type].clear();

  stream_matched_datas_[type].push_back(data);
}

void Match::Start() {
  if (is_matching_) {
    LOGW("%s, %d: Matching is started.\n", __FILE__, __LINE__);
    return;
  }

  is_matching_ = true;
  match_thread_ = std::thread([this](){
    Rate rate(10);
    while (is_matching_) {
      MatchStreamDatas();
      rate.Sleep();
    }
  });
}

/*
void Match::UpdateCalledNumbers(const ImageType& type) {
  if (type == ImageType::IMAGE_LEFT_COLOR ||
      type == ImageType::IMAGE_RIGHT_COLOR) {
    color_called_num_++;
    depth_called_num_ = 0;
  } else if (type == ImageType::IMAGE_DEPTH) {
    depth_called_num_++;
    color_called_num_ = 0;
  }

  if (color_called_num_ > depth_called_num_ &&
      color_called_num_ - depth_called_num_ > 1 &&
      type == ImageType::IMAGE_DEPTH) {
  }
}
*/

























