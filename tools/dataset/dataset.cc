// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "dataset/dataset.h"
#include "mynteyed/util/files.h"

#ifdef WITH_OPENCV2
#include <opencv2/highgui/highgui.hpp>
#else
#include <opencv2/imgcodecs/imgcodecs.hpp>
#endif

#include <iomanip>
#include <limits>
#include <stdexcept>
#include <utility>

#define FULL_PRECISION \
  std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)

#define IMAGE_FILENAME_WIDTH 6

MYNTEYE_BEGIN_NAMESPACE

namespace tools {

Dataset::Dataset(std::string outdir) : outdir_(std::move(outdir)) {
  std::cout << __func__ << std::endl;
  if (!files::mkdir(outdir_)) {
    std::cout << "Create directory failed: " << outdir_ << std::endl;
  }
}

Dataset::~Dataset() {
  for (auto &&it = stream_writers_.begin(); it != stream_writers_.end(); it++) {
    if (it->second) {
      it->second->ofs.flush();
      it->second->ofs.close();
    }
  }
  if (motion_writer_) {
    motion_writer_->ofs.flush();
    motion_writer_->ofs.close();
  }
}

void Dataset::SaveMotionData(const MYNTEYE_NAMESPACE::MotionData &data) {
  auto &&writer = GetMotionWriter();
  auto seq = motion_count_;

  writer->ofs << seq << ", " << static_cast<int>(data.imu->flag) << ", "
    << data.imu->timestamp << ", "<< data.imu->accel[0] << ", "
    << data.imu->accel[1] << ", " << data.imu->accel[2] << ", "
    << data.imu->gyro[0] << ", " << data.imu->gyro[1] << ", "
    << data.imu->gyro[2] << ", " << data.imu->temperature << std::endl;
  ++motion_count_;
}

void Dataset::SaveStreamData(const ImageType &type,
    const MYNTEYE_NAMESPACE::StreamData &data) {
  auto &&writer = GetStreamWriter(type);
  auto seq = stream_count_[type];

  if (data.img_info) {
    writer->ofs << seq << ", " << data.img_info->frame_id << ", "
      << data.img_info->timestamp << ", "
      << data.img_info->exposure_time << std::endl;
    ++stream_count_[type];
  }

  if (data.img) {
    static std::uint64_t count = 0;
    ++count;
    std::stringstream ss;
    ss << writer->outdir << MYNTEYE_OS_SEP << std::dec
       << std::setw(IMAGE_FILENAME_WIDTH) << std::setfill('0') << count << ".png";
    cv::imwrite(ss.str(), data.img->To(ImageFormat::COLOR_BGR)->ToMat());
  }
}

Dataset::writer_t Dataset::GetMotionWriter() {
  if (motion_writer_ == nullptr) {
    writer_t writer = std::make_shared<Writer>();
    writer->outdir = outdir_;
    writer->outfile = writer->outdir + MYNTEYE_OS_SEP "motion.txt";

    writer->ofs.open(writer->outfile, std::ofstream::out);
    writer->ofs << "seq, flag, timestamp, "
                   "accel_x, accel_y, accel_z, "
                   "gyro_x, gyro_y, gyro_z, temperature" << std::endl;
    writer->ofs << FULL_PRECISION;

    motion_writer_ = writer;
    motion_count_ = 0;
  }
  return motion_writer_;
}

Dataset::writer_t Dataset::GetStreamWriter(const ImageType &type) {
  try {
    return stream_writers_.at(type);
  } catch (const std::out_of_range &e) {
    writer_t writer = std::make_shared<Writer>();
    switch (type) {
      case ImageType::IMAGE_LEFT_COLOR: {
        writer->outdir = outdir_ + MYNTEYE_OS_SEP "left";
      } break;
      case ImageType::IMAGE_RIGHT_COLOR: {
        writer->outdir = outdir_ + MYNTEYE_OS_SEP "right";
      } break;
      default:
        std::cout << "Unsupported ImageType." << std::endl;
    }
    writer->outfile = writer->outdir + MYNTEYE_OS_SEP "stream.txt";

    files::mkdir(writer->outdir);
    writer->ofs.open(writer->outfile, std::ofstream::out);
    writer->ofs << "seq, frame_id, timestamp, exposure_time" << std::endl;
    writer->ofs << FULL_PRECISION;

    stream_writers_[type] = writer;
    stream_count_[type] = 0;
    return writer;
  }
}

}  // namespace tools

MYNTEYE_END_NAMESPACE
