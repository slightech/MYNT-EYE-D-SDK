#include "mynteyed/util/log.h"
#include "mynteyed/internal/distance.h"

MYNTEYE_USE_NAMESPACE

Distance::Distance() :
  is_distance_datas_enabled_(false),
  distance_datas_max_size_(1000),
  distance_callback_(nullptr),
  distance_count_(0) {
}

Distance::~Distance() {
}

void Distance::EnableDistanceDatas(std::size_t max_size) {
  if (is_distance_datas_enabled_ &&
      distance_datas_max_size_ == max_size) {
    return;
  }

  std::lock_guard<std::mutex> _(mutex_);
  is_distance_datas_enabled_ = true;
  distance_datas_max_size_ = max_size;
}

void Distance::DisableDistanceDatas() {
  if (!is_distance_datas_enabled_)
    return;

  std::lock_guard<std::mutex> _(mutex_);
  is_distance_datas_enabled_ = false;
  distance_datas_max_size_ = 0;
  distance_datas_.clear();
}

bool Distance::IsDistanceDatasEnabled() const {
  return is_distance_datas_enabled_;
}

Distance::datas_t Distance::GetDistanceDatas() {
  if (!is_distance_datas_enabled_) {
    throw_error("Must enable distance datas before getting them, or you set "
                "distance callback instead");
  }

  std::lock_guard<std::mutex> _(mutex_);
  return std::move(distance_datas_);
}

void Distance::SetDistanceCallback(distance_callback_t callback) {
  std::lock_guard<std::mutex> _(mutex_);
  distance_callback_ = callback;
}

void Distance::OnDisDataCallback(const ObstacleDisPacket& packet) {
  auto &&dis = std::make_shared<ObstacleDis>();

  dis->detection_time = packet.detection_time;
  dis->distance = packet.distance;

  /*
  if (distance_count_ < 20) {
    ++distance_count_;
    return;
  }
  */

  std::lock_guard<std::mutex> _(mutex_);

  data_t data = {dis};
  if (distance_datas_max_size_ > 0) {
    if (distance_datas_.size() > distance_datas_max_size_) {
      distance_datas_.erase(distance_datas_.begin());
    }
    distance_datas_.push_back(data);
  }

  if (distance_callback_) {
    distance_callback_(data);
  }
}
