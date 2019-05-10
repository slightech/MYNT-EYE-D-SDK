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
#ifndef MYNTEYE_DEVICE_TYPES_INTERNAL_H_
#define MYNTEYE_DEVICE_TYPES_INTERNAL_H_
#pragma once

#include "mynteyed/stubs/global.h"

MYNTEYE_BEGIN_NAMESPACE

/** Camera calibration. */
struct CameraCalibration {
  union {
    unsigned char uByteArray[1024];/**< union data defined as below struct { }*/
    struct {
      unsigned short  InImgWidth;/**< Input image width(SideBySide image) */
      unsigned short  InImgHeight;/**< Input image height */
      unsigned short  OutImgWidth;/**< Output image width(SideBySide image) */
      unsigned short  OutImgHeight;/**< Output image height */
      int             RECT_ScaleEnable;/**< Rectified image scale */
      int             RECT_CropEnable;/**< Rectified image crop */
      unsigned short  RECT_ScaleWidth;/**< Input image width(Single image) *RECT_Scale_Col_N /RECT_Scale_Col_M */
      unsigned short  RECT_ScaleHeight;/**< Input image height(Single image) *RECT_Scale_Row_N /RECT_Scale_Row_M */
      float      CamMat1[9];/**< Left Camera Matrix
                fx, 0, cx, 0, fy, cy, 0, 0, 1
                fx,fy : focus  ; cx,cy : principle point */
      float      CamDist1[8];/**< Left Camera Distortion Matrix
                k1, k2, p1, p2, k3, k4, k5, k6
                k1~k6 : radial distort ; p1,p2 : tangential distort */
      float      CamMat2[9];/**< Right Camera Matrix
                fx, 0, cx, 0, fy, cy, 0, 0, 1
                fx,fy : focus  ; cx,cy : principle point */
      float      CamDist2[8];/**< Right Camera Distortion Matrix
                k1, k2, p1, p2, k3, k4, k5, k6
                k1~k6 : radial distort ; p1,p2 : tangential distort */
      float      RotaMat[9];/**< Rotation matrix between the left and right camera coordinate systems.
                | [0] [1] [2] |       |Xcr|
                | [3] [4] [5] |   *   |Ycr|            => cr = right camera coordinate
                | [6] [7] [8] |       |Zcr| */
      float      TranMat[3];/**< Translation vector between the coordinate systems of the cameras.
                |[0]|      |Xcr|
                |[1]|   +  |Ycr|               => cr = right camera coordinate
                |[2]|      |Zcr| */
      float      LRotaMat[9];/**< 3x3 rectification transform (rotation matrix) for the left camera.
                | [0] [1] [2] |       |Xcl|
                | [3] [4] [5] |   *   |Ycl|            => cl = left camera coordinate
                | [6] [7] [8] |       |Zcl| */
      float      RRotaMat[9];/**< 3x3 rectification transform (rotation matrix) for the left camera.
                | [0] [1] [2] |       |Xcr|
                | [3] [4] [5] |   *   |Ycr|            => cr = right camera coordinate
                | [6] [7] [8] |       |Zcr| */
      float      NewCamMat1[12];/**< 3x4 projection matrix in the (rectified) coordinate systems for the left camera.
                fx' 0 cx' 0 0 fy' cy' 0 0 0 1 0
                fx',fy' : rectified focus ; cx', cy; : rectified principle point */
      float      NewCamMat2[12];/**< 3x4 projection matrix in the (rectified) coordinate systems for the rightt camera.
                fx' 0 cx' TranMat[0]* 0 fy' cy' 0 0 0 1 0
                fx',fy' : rectified focus ; cx', cy; : rectified principle point */
      unsigned short  RECT_Crop_Row_BG;/**< Rectidied image crop row begin */
      unsigned short  RECT_Crop_Row_ED;/**< Rectidied image crop row end */
      unsigned short  RECT_Crop_Col_BG_L;/**< Rectidied image crop column begin */
      unsigned short  RECT_Crop_Col_ED_L;/**< Rectidied image crop column end */
      unsigned char  RECT_Scale_Col_M;/**< Rectified image scale column factor M */
      unsigned char  RECT_Scale_Col_N;/**< Rectified image scale column factor N
                Rectified image scale column ratio =  Scale_Col_N/ Scale_Col_M */
      unsigned char  RECT_Scale_Row_M;/**< Rectified image scale row factor M */
      unsigned char  RECT_Scale_Row_N;/**< Rectified image scale row factor N */
      float      RECT_AvgErr;/**< Reprojection error */
      unsigned short  nLineBuffers;/**< Linebuffer for Hardware limitation < 60 */
            float ReProjectMat[16];
    };
  };
};

enum class ControlParams : std::int32_t {
  AUTO_EXPOSURE = 0,
  AUTO_WHITE_BALANCE,
  IR_DEPTH_ONLY,
  IR_INTENSITY,
  GLOBAL_GAIN,
  EXPOSURE_TIME,
  HW_REGISTER,
  FW_REGISTER,
};

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DEVICE_TYPES_INTERNAL_H_
