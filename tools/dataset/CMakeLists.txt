# Copyright 2018 Slightech Co., Ltd. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

get_filename_component(DIR_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)

set(OUTPUT_DIR "${CMAKE_CURRENT_LIST_DIR}/_output")

set_outdir(
  ARCHIVE ${OUTPUT_DIR}/lib
  LIBRARY ${OUTPUT_DIR}/lib
  RUNTIME ${OUTPUT_DIR}/bin
)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/src
  ${PRO_DIR}/include
)
set(MYNTEYE_DLL_SEARCH_PATHS
  ${PRO_DIR}/_install/bin
  ${PRO_DIR}/_install/bin/3rdparty
  ${OpenCV_LIB_SEARCH_PATH}
)
## record

make_executable(record
  SRCS record.cc dataset.cc
  LINK_LIBS  mynteye_core ${OpenCV_LIBS}
  DLL_SEARCH_PATHS ${PRO_DIR}/_install/bin ${MYNTEYE_DLL_SEARCH_PATHS}
)
