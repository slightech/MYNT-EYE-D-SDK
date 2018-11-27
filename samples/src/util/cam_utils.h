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
#ifndef MYNTEYE_SAMPLES_UTIL_CAM_UTILS_H_
#define MYNTEYE_SAMPLES_UTIL_CAM_UTILS_H_
#pragma once

#include <stdio.h>
#ifdef MYNTEYE_OS_LINUX
#include <termio.h>
#endif
#ifdef MYNTEYE_OS_WIN
#include <conio.h>
#endif
#include <sstream>
#include <string>
#include <memory>
#include <utility>

#include "mynteyed/stubs/global.h"

MYNTEYE_BEGIN_NAMESPACE

namespace util {

inline
std::ostringstream& clear(std::ostringstream& os) {
  os.str("");
  os.clear();
  return os;
}

std::shared_ptr<std::ios> new_format(int width, int prec, char fillch = ' ');

template <typename T>
std::string to_string(T val) {
  std::ostringstream ss;
  ss << val;
  return ss.str();
}

template <typename T>
std::string to_string(T val, const std::shared_ptr<std::ios>& fmt) {
  std::ostringstream ss;
  if (fmt) ss.copyfmt(*fmt);
  ss << val;
  return ss.str();
}

template <typename T>
std::string to_string(T val, int width, int prec, char fillch = ' ') {
  auto fmt = new_format(std::move(width), std::move(prec), std::move(fillch));
  return to_string(val, fmt);
}

}  // namespace util

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_SAMPLES_UTIL_CAM_UTILS_H_
