#!/usr/bin/env bash
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

BASE_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(realpath "$BASE_DIR/../..")
SCRIPTS_DIR=$(realpath "$BASE_DIR/..")

source "$SCRIPTS_DIR/common/echo.sh"
source "$SCRIPTS_DIR/common/detect.sh"

if [ ! -d "$ROOT_DIR/3rdparty/opencv" ]; then
  _echo_e "3rdparty/opencv not found, please manually download it to here."
  _echo_e
  _echo_e "  OpenCV Win pack 3.4.3: https://opencv.org/releases.html"
  exit 1
fi

if [ ! -d "$ROOT_DIR/3rdparty/libjpeg-turbo64" ]; then
  _echo_e "3rdparty/libjpeg-turbo64 not found, please manually download it to here."
  _echo_e
  _echo_e "  libjpeg-turbo-2.0.0-vc64.exe: https://sourceforge.net/projects/libjpeg-turbo/files/2.0.0/"
  exit 1
fi

if ! _detect_cmd makensis; then
  _echo_e "makensis not found, please manually download and install it."
  _echo_e
  _echo_e "  NSIS: http://nsis.sourceforge.net"
  exit 1
fi

export OpenCV_DIR="$ROOT_DIR/3rdparty/opencv/build"

_rm() {
  [ -e "$1" ] && (rm -r "$1" && _echo_i "RM: $1")
}

_md() {
  [ ! -d "$1" ] && (mkdir -p "$1" && _echo_i "MD: $1")
}

################################################################################
# build

make samples tools

################################################################################
# move to _install

################################################################################
# copy to _install

################################################################################
# archive exe

source "$ROOT_DIR/ocvinfo.sh"
_pkgname="$1-opencv-$OpenCV_VERSION"
_echo_i "pkgname: $_pkgname"

_rm "$ROOT_DIR/$_pkgname.exe"
mv "$ROOT_DIR/_install" "$ROOT_DIR/$_pkgname"

# makensis "$ROOT_DIR/winpack.nsi"

if _detect_cmd git; then
  _git_branch=`git symbolic-ref --short -q HEAD`
  if [ "$_git_branch" == "develop" ]; then
    _git_hash=`git rev-parse --short HEAD`
    mv "$ROOT_DIR/$_pkgname.exe" "$ROOT_DIR/$_pkgname-dev-$_git_hash.exe"
  fi
fi

mv "$ROOT_DIR/$_pkgname" "$ROOT_DIR/_install"

################################################################################
# remove from _install

################################################################################
# move back from _install

################################################################################
# clean build

_rm "$ROOT_DIR/_build"
_rm "$ROOT_DIR/_output"


_echo_d "Win pack success"
