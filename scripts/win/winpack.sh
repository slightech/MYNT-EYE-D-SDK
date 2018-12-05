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
export PATH="$ROOT_DIR/3rdparty/libjpeg-turbo64/bin:$PATH"

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

# 3rdparty/opencv
_md "$ROOT_DIR/_install/3rdparty"
mv "$ROOT_DIR/3rdparty/opencv" "$ROOT_DIR/_install/3rdparty/opencv"

# 3rdparty/libjpeg-turbo64
mv "$ROOT_DIR/3rdparty/libjpeg-turbo64" "$ROOT_DIR/_install/3rdparty/libjpeg-turbo64"

# samples
mv "$ROOT_DIR/samples/_output/bin" "$ROOT_DIR/_install/bin/samples"
mv "$ROOT_DIR/samples/_output/lib" "$ROOT_DIR/_install/lib/samples"
_rm "$ROOT_DIR/samples/_build"
_rm "$ROOT_DIR/samples/_output"
mv "$ROOT_DIR/samples" "$ROOT_DIR/_install/samples"

# tools
mv "$ROOT_DIR/tools/_output/bin" "$ROOT_DIR/_install/bin/tools"
mv "$ROOT_DIR/tools/_output/lib" "$ROOT_DIR/_install/lib/tools"
_rm "$ROOT_DIR/tools/_build"
_rm "$ROOT_DIR/tools/_output"
mv "$ROOT_DIR/tools/linter" "$ROOT_DIR/3rdparty/linter"
mv "$ROOT_DIR/tools" "$ROOT_DIR/_install/tools"

# platforms/win
mv "$ROOT_DIR/platforms/win/README.txt" "$ROOT_DIR/_install"

_rm "$ROOT_DIR/platforms/projects/vs2017/mynteyed_demo/.vs"
_rm "$ROOT_DIR/platforms/projects/vs2017/mynteyed_demo/x64"
_rm "$ROOT_DIR/platforms/projects/vs2017/mynteyed_demo/mynteyed_demo/x64"
_rm "$ROOT_DIR/platforms/projects/vs2017/mynteyed_demo/mynteyed_demo/mynteyed_demo.vcxproj.user"
mv "$ROOT_DIR/platforms/projects" "$ROOT_DIR/_install/projects"

################################################################################
# copy to _install

# cmake
_md "$ROOT_DIR/_install/cmake"
cp -f "$ROOT_DIR/cmake/Common.cmake" "$ROOT_DIR/_install/cmake"
cp -f "$ROOT_DIR/cmake/DetectCXX11.cmake" "$ROOT_DIR/_install/cmake"
cp -f "$ROOT_DIR/cmake/DetectOpenCV.cmake" "$ROOT_DIR/_install/cmake"
cp -f "$ROOT_DIR/cmake/IncludeGuard.cmake" "$ROOT_DIR/_install/cmake"
cp -f "$ROOT_DIR/cmake/TargetArch.cmake" "$ROOT_DIR/_install/cmake"
_md "$ROOT_DIR/_install/cmake/templates"
cp -f "$ROOT_DIR/cmake/templates/exe.bat.in" "$ROOT_DIR/_install/cmake/templates"

cp -f "$ROOT_DIR/scripts/win/cmake/mynteyed-targets.cmake" "$ROOT_DIR/_install/lib/cmake/mynteyed/"
cp -f "$ROOT_DIR/scripts/win/cmake/mynteyed-targets-release.cmake" "$ROOT_DIR/_install/lib/cmake/mynteyed/"

# generate.bat
cp -f "$ROOT_DIR/scripts/win/generate.bat" "$ROOT_DIR/_install/samples/"
cp -f "$ROOT_DIR/scripts/win/generate.bat" "$ROOT_DIR/_install/tools/"

################################################################################
# archive exe

source "$ROOT_DIR/ocvinfo.sh"
_pkgname="$1-opencv-$OpenCV_VERSION"
# _echo_i "pkgname: $_pkgname"

_rm "$ROOT_DIR/$_pkgname.exe"
mv "$ROOT_DIR/_install" "$ROOT_DIR/$_pkgname"

makensis "$ROOT_DIR/winpack.nsi"

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

_rm "$ROOT_DIR/_install/samples/generate.bat"
_rm "$ROOT_DIR/_install/tools/generate.bat"

################################################################################
# move back from _install

# 3rdparty/opencv
mv "$ROOT_DIR/_install/3rdparty/opencv" "$ROOT_DIR/3rdparty/opencv"

# 3rdparty/libjpeg-turbo64
mv "$ROOT_DIR/_install/3rdparty/libjpeg-turbo64" "$ROOT_DIR/3rdparty/libjpeg-turbo64"

# samples
mv "$ROOT_DIR/_install/samples" "$ROOT_DIR/samples"

# tools
mv "$ROOT_DIR/_install/tools" "$ROOT_DIR/tools"
mv "$ROOT_DIR/3rdparty/linter" "$ROOT_DIR/tools/linter"

# platforms/win
mv "$ROOT_DIR/_install/README.txt" "$ROOT_DIR/platforms/win"

mv "$ROOT_DIR/_install/projects" "$ROOT_DIR/platforms/projects"

################################################################################
# clean build

_rm "$ROOT_DIR/_build"
_rm "$ROOT_DIR/_output"


_echo_d "Win pack success"
