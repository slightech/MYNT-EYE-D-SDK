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
#include "mynteyed/stubs/types_calib.h"

#include <iomanip>
#include <limits>

#define FULL_PRECISION \
  std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)

MYNTEYE_BEGIN_NAMESPACE

std::ostream &operator<<(std::ostream &os, const CameraIntrinsics &in) {
  os << FULL_PRECISION << "width: [" << in.width << "]";
  os << ", height: [" << in.height << "]";
  os << ", fx: [" << in.fx << "]";
  os << ", fy: [" << in.fy << "]";
  os << ", cx: [" << in.cx << "]";
  os << ", cy: [" << in.cy << "]";

  os << ", coeffs: [";
  for (int i = 0; i < 4; i++)
    os << in.coeffs[i] << ", ";
  os << in.coeffs[4] << "]";

  os << ", p: [";
  for (int i = 0; i < 11; i++)
    os << in.p[i] << ", ";
  os << in.p[11] << "]";

  os << ", r: [";
  for (int i = 0; i < 8; i++)
    os << in.r[i] << ", ";
  os << in.r[8] << "]";

  return os;
}

std::ostream &operator<<(std::ostream &os, const ImuIntrinsics &in) {
  os << FULL_PRECISION << "scale: [";
  for (int i = 0; i <= 2; i++)
    os << in.scale[0][i] << ", ";
  for (int i = 0; i <= 2; i++)
    os << in.scale[1][i] << ", ";
  for (int i = 0; i <= 1; i++)
    os << in.scale[2][i] << ", ";
  os << in.scale[2][2] << "]";

  os << ", assembly: [";
  for (int i = 0; i <= 2; i++)
    os << in.assembly[0][i] << ", ";
  for (int i = 0; i <= 2; i++)
    os << in.assembly[1][i] << ", ";
  for (int i = 0; i <= 1; i++)
    os << in.assembly[2][i] << ", ";
  os << in.assembly[2][2] << "]";

  os << ", drift: [";
  for (int i = 0; i <= 1; i++)
    os << in.drift[i] << ", ";
  os << in.drift[2] << "]";

  os << ", noise: [";
  for (int i = 0; i <= 1; i++)
    os << in.noise[i] << ", ";
  os << in.noise[2] << "]";

  os << ", bias: [";
  for (int i = 0; i <= 1; i++)
    os << in.bias[i] << ", ";
  os << in.bias[2] << "]";

  os << ", x: [";
  for (int i = 0; i <= 0; i++)
    os << in.x[i] << ", ";
  os << in.x[1] << "]";

  os << ", y: [";
  for (int i = 0; i <= 0; i++)
    os << in.y[i] << ", ";
  os << in.y[1] << "]";

  os << ", z: [";
  for (int i = 0; i <= 0; i++)
    os << in.z[i] << ", ";
  os << in.z[1] << "]";

  return os;
}

std::ostream &operator<<(std::ostream &os, const MotionIntrinsics &in) {
  return os << FULL_PRECISION << "accel: {" << in.accel << "}, gyro: {"
            << in.gyro << "}";
}

std::ostream &operator<<(std::ostream &os, const Extrinsics &ex) {
  os << FULL_PRECISION << "rotation: [";
  for (int i = 0; i <= 2; i++)
    os << ex.rotation[0][i] << ", ";
  for (int i = 0; i <= 2; i++)
    os << ex.rotation[1][i] << ", ";
  for (int i = 0; i <= 1; i++)
    os << ex.rotation[2][i] << ", ";
  os << ex.rotation[2][2] << "]";

  os << ", translation: [";
  for (int i = 0; i <= 1; i++)
    os << ex.translation[i] << ", ";
  os << ex.translation[2] << "]";

  return os;
}

MYNTEYE_END_NAMESPACE
