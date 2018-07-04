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
#include "mynteye.h"

using namespace mynteye;

std::ostream &operator<<(std::ostream &os, const StreamFormat &code) {
    switch (code) {
        case StreamFormat::STREAM_MJPG:
            os << "STREAM_MJPG"; break;
        case StreamFormat::STREAM_YUYV:
            os << "STREAM_YUYV"; break;
        case StreamFormat::STREAM_LAST:
            os << "STREAM_LAST"; break;
        default:
            os << "UNKNOWN"; break;
    }
    return os;
}
