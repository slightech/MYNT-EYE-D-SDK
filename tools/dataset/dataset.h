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
#ifndef TOOLS_DATASET_DATASET_H_
#define TOOLS_DATASET_DATASET_H_

#include <fstream>
#include <memory>
#include <string>
#include <map>

#include "mynteyed/camera.h"

MYNTEYE_BEGIN_NAMESPACE

namespace tools {

class Dataset {
 public:
  struct Writer {
    std::ofstream ofs;
    std::string outdir;
    std::string outfile;
  };

  using writer_t = std::shared_ptr<Writer>;

  explicit Dataset(std::string outdir);
  virtual ~Dataset();

  void SaveMotionData(const MYNTEYE_NAMESPACE::MotionData &data);
  void SaveStreamData(const ImageType &type,
      const MYNTEYE_NAMESPACE::StreamData &data);

 private:
  writer_t GetMotionWriter();
  writer_t GetStreamWriter(const ImageType &type);

  std::string outdir_;

  writer_t motion_writer_;
  std::map<ImageType, writer_t> stream_writers_;
  std::size_t motion_count_;
  std::map<ImageType, std::size_t> stream_count_;
};

}  // namespace tools

MYNTEYE_END_NAMESPACE

#endif  // TOOLS_DATASET_DATASET_H_
