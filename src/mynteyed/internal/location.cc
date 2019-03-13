#include "mynteyed/util/log.h"
#include "mynteyed/internal/location.h"

MYNTEYE_USE_NAMESPACE

Location::Location() :
  is_location_datas_enabled_(false),
  location_datas_max_size_(1000),
  location_callback_(nullptr),
  location_count_(0) {
}

Location::~Location() {
}

void Location::EnableLocationDatas(std::size_t max_size) {
  if (is_location_datas_enabled_ &&
      location_datas_max_size_ == max_size) {
    return;
  }

  std::lock_guard<std::mutex> _(mutex_);
  is_location_datas_enabled_ = true;
  location_datas_max_size_ = max_size;
}

void Location::DisableLocationDatas() {
  if (!is_location_datas_enabled_)
    return;

  std::lock_guard<std::mutex> _(mutex_);
  is_location_datas_enabled_ = false;
  location_datas_max_size_ = 0;
  location_datas_.clear();
}

bool Location::IsLocationDatasEnabled() const {
  return is_location_datas_enabled_;
}

Location::datas_t Location::GetLocationDatas() {
  if (!is_location_datas_enabled_) {
    throw_error("Must enable location datas before getting them, or you set "
                "location callback instead");
  }

  std::lock_guard<std::mutex> _(mutex_);
  return std::move(location_datas_);
}

void Location::SetLocationCallback(location_callback_t callback) {
  std::lock_guard<std::mutex> _(mutex_);
  location_callback_ = callback;
}

void Location::OnGPSDataCallback(const GPSDataPacket& packet) {
  auto &&gps = std::make_shared<GPSData>();

  gps->device_time = packet.device_time;
  gps->latitude = packet.latitude;
  gps->longitude = packet.longitude;
  gps->NS = packet.NS;
  gps->EW = packet.EW;

  gps->year = packet.year;
  gps->month = packet.month;
  gps->day = packet.day;
  gps->hour = packet.hour;
  gps->minute = packet.minute;
  gps->second = packet.second;

  float latitude_cent_tmp, latitude_second_tmp;
  float longitude_cent_tmp, longitude_second_tmp;

  gps->latitude_degree =
    fabs(gps->latitude / 100); // separate latitude
  latitude_cent_tmp = gps->latitude - gps->latitude_degree * 100;
  gps->latitude_cent = fabs(latitude_cent_tmp);
  latitude_second_tmp = (latitude_cent_tmp - gps->latitude_cent) * 60;
  gps->latitude_second = fabs(latitude_second_tmp);

  gps->longitude_degree =
    fabs(gps->longitude / 100); // separate latitude
  longitude_cent_tmp = gps->longitude - gps->longitude_degree * 100;
  gps->longitude_cent = fabs(longitude_cent_tmp);
  longitude_second_tmp = (longitude_cent_tmp - gps->longitude_cent) * 60;
  gps->longitude_second = fabs(longitude_second_tmp);

  /*
  if (location_count_ < 20) {
    ++location_count_;
    return;
  }
  */

  std::lock_guard<std::mutex> _(mutex_);

  data_t data = {gps};
  if (location_datas_max_size_ > 0) {
    if (location_datas_.size() > location_datas_max_size_) {
      location_datas_.erase(location_datas_.begin());
    }
    location_datas_.push_back(data);
  }

  if (location_callback_) {
    location_callback_(data);
  }
}
