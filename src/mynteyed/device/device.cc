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
#include <cstring>
#include <fstream>
#include <string>

#include "mynteyed/device/device.h"
#include "mynteyed/util/log.h"

#include "colorizer_p.h"

MYNTEYE_USE_NAMESPACE

static const int MAX_STREAM_COUNT = 64;
static const int MAX_CHECK_TIMES = 100;

namespace {

void get_stream_size(const StreamMode& stream_mode, int* width, int* height) {
  switch (stream_mode) {
    case StreamMode::STREAM_640x480:
      *width = 640;
      *height = 480;
      break;
    case StreamMode::STREAM_1280x480:
      *width = 1280;
      *height = 480;
      break;
    case StreamMode::STREAM_1280x720:
      *width = 1280;
      *height = 720;
      break;
    case StreamMode::STREAM_2560x720:
      *width = 2560;
      *height = 720;
      break;
    default:
      throw new std::runtime_error("StreamMode is unknown");
  }
}

std::string get_stream_format_string(const StreamFormat& stream_format) {
  switch (stream_format) {
    case StreamFormat::STREAM_MJPG:
      return "MJPG";
    case StreamFormat::STREAM_YUYV:
      return "YUYV";
    default:
      throw new std::runtime_error("StreamFormat is unknown");
  }
}

int get_sensor_mode(const SensorMode &mode) {
  switch (mode) {
    case SensorMode::LEFT:
      return 0;
    case SensorMode::RIGHT:
      return 1;
    case SensorMode::ALL:
      return 2;
    default:
      throw_error("Unknown sensor mode.");
  }
}

int get_sensor_type(const SensorType &type) {
  switch (type) {
    case SensorType::SENSOR_TYPE_H22:
      return 0;   // ETRONDI_SENSOR_TYPE_H22
    case SensorType::SENSOR_TYPE_OV7740:
      return 1;   // ETRONDI_SENSOR_TYPE_OV7740
    case SensorType::SENSOR_TYPE_AR0134:
      return 2;   // ETRONDI_SENSOR_TYPE_AR0134
    case SensorType::SENSOR_TYPE_AR0135:
      return 3;   // ETRONDI_SENSOR_TYPE_AR0135
    case SensorType::SENSOR_TYPE_OV9714:
      return 4;   // ETRONDI_SENSOR_TYPE_OV9714
    default:
      throw_error("Unknown sensor type.");
  }
}

}  // namespace

Device::Device()
  : handle_(nullptr), dev_sel_info_({-1}),
    camera_calibrations_({nullptr, nullptr}) {
  DBG_LOGD(__func__);
  Init();
}

Device::~Device() {
  DBG_LOGD(__func__);
  free(stream_color_info_ptr_);
  free(stream_depth_info_ptr_);
  Close();
  colorizer_ = nullptr;
}

void Device::Init() {
  int ret = EtronDI_Init(&handle_, false);
  UNUSED(ret);

  stream_color_info_ptr_ =
      (PETRONDI_STREAM_INFO)malloc(sizeof(ETRONDI_STREAM_INFO) * MAX_STREAM_COUNT);
  stream_depth_info_ptr_ =
      (PETRONDI_STREAM_INFO)malloc(sizeof(ETRONDI_STREAM_INFO) * MAX_STREAM_COUNT);
  // default image type
  depth_data_type_ = 2;
  // default frame rate
  framerate_ = 10;

  color_device_opened_ = false;
  depth_device_opened_ = false;
  is_device_opened_ = false;

  ir_depth_only_enabled_ = false;
  color_ir_depth_only_enabled_ = false;
  depth_ir_depth_only_enabled_ = false;

  device_status_ = {{COLOR_DEVICE, false}, {DEPTH_DEVICE, false}};
  is_actual_ = {{COLOR_DEVICE, false}, {DEPTH_DEVICE, false}};
  check_times_ = MAX_CHECK_TIMES;
  is_disconnect_ = false;

  OnInit();
}

void Device::GetDeviceInfos(std::vector<DeviceInfo>* dev_infos) {
  if (!dev_infos) {
    LOGE("GetDevices: dev_infos is null.");
    return;
  }
  dev_infos->clear();

  int num = 0;
  int count = 0;
  while (true) {
    if (num > 0 && num < 3) {
      EtronDI_Release(&handle_);
      EtronDI_Init(&handle_, false);
      LOGI("\n");
    } else if (num >= 3) {
      break;
    }
    count = EtronDI_GetDeviceNumber(handle_);
    if (count <= 0) {
      num++;
    } else {
      break;
    }
    DBG_LOGD("GetDevices: %d", count);
  }

  DEVSELINFO dev_sel_info;
  DEVINFORMATION* p_dev_info =
      (DEVINFORMATION*)malloc(sizeof(DEVINFORMATION) * count);  // NOLINT

  for (int i = 0; i < count; i++) {
    dev_sel_info.index = i;

    EtronDI_GetDeviceInfo(handle_, &dev_sel_info, p_dev_info+i);

    char sz_buf[256];
    int actual_length = 0;
    std::string s_sn = "null";
    unsigned char serial_n[512];
    int len;
    if (ETronDI_OK == EtronDI_GetFwVersion(
        handle_, &dev_sel_info, sz_buf, 256, &actual_length) &&
        ETronDI_OK == EtronDI_GetSerialNumber(
        handle_, &dev_sel_info, serial_n, 512, &len)) {
      char tmp[25];
      memset(tmp, '\0', sizeof(tmp));
      for (int i = 0; i < len / 2; i++) {
        tmp[i] = serial_n[i * 2];
      }
      s_sn = tmp;
      DeviceInfo info;
      info.index = i;
      info.name = p_dev_info[i].strDevName;
      info.type = p_dev_info[i].nDevType;
      info.pid = p_dev_info[i].wPID;
      info.vid = p_dev_info[i].wVID;
      info.chip_id = p_dev_info[i].nChipID;
      info.fw_version = sz_buf;
      info.sn = s_sn;
      dev_infos->push_back(std::move(info));
    }
  }

  free(p_dev_info);
}

std::shared_ptr<DeviceInfo> Device::GetDeviceInfo() {
  if (dev_sel_info_.index == -1) {
    LOGE("GetDevices: dev_sel_info_ is -1.");
    return nullptr;
  }
  int num = 0;
  int count = 0;
  while (true) {
    if (num > 0 && num < 3) {
      EtronDI_Release(&handle_);
      EtronDI_Init(&handle_, false);
      LOGI("\n");
    } else if (num >= 3) {
      break;
    }
    count = EtronDI_GetDeviceNumber(handle_);
    if (count <= 0) {
      num++;
    } else {
      break;
    }
    DBG_LOGD("GetDevices: %d", count);
  }

  DEVSELINFO dev_sel_info;
  DEVINFORMATION* p_dev_info =
      (DEVINFORMATION*)malloc(sizeof(DEVINFORMATION) * count);  // NOLINT

  for (int i = 0; i < count; i++) {
    dev_sel_info.index = i;

    EtronDI_GetDeviceInfo(handle_, &dev_sel_info, p_dev_info+i);

    char sz_buf[256];
    int actual_length = 0;
    std::string s_sn = "null";
    unsigned char serial_n[512];
    int len;
    if (ETronDI_OK == EtronDI_GetFwVersion(
        handle_, &dev_sel_info, sz_buf, 256, &actual_length) &&
        ETronDI_OK == EtronDI_GetSerialNumber(
        handle_, &dev_sel_info, serial_n, 512, &len)) {
      char tmp[25];
      memset(tmp, '\0', sizeof(tmp));
      for (int i = 0; i < len / 2; i++) {
        tmp[i] = serial_n[i * 2];
      }
      s_sn = tmp;
      DeviceInfo info;
      info.index = i;
      info.name = p_dev_info[i].strDevName;
      info.type = p_dev_info[i].nDevType;
      info.pid = p_dev_info[i].wPID;
      info.vid = p_dev_info[i].wVID;
      info.chip_id = p_dev_info[i].nChipID;
      info.fw_version = sz_buf;
      info.sn = s_sn;
      if (i == dev_sel_info_.index) {
        free(p_dev_info);
        return std::make_shared<DeviceInfo>(info);
      }
    }
  }

  free(p_dev_info);
  return nullptr;
}

void Device::GetStreamInfos(const std::int32_t& dev_index,
    std::vector<StreamInfo>* color_infos,
    std::vector<StreamInfo>* depth_infos) {
  if (!color_infos) {
    LOGE("GetResolutions: color_infos is null.");
    return;
  }
  color_infos->clear();

  if (!depth_infos) {
    LOGE("GetResolutions: depth_infos is null.");
    return;
  }
  depth_infos->clear();

  memset(stream_color_info_ptr_, 0, sizeof(ETRONDI_STREAM_INFO) * MAX_STREAM_COUNT);
  memset(stream_depth_info_ptr_, 0, sizeof(ETRONDI_STREAM_INFO) * MAX_STREAM_COUNT);

  DEVSELINFO dev_sel_info{dev_index};
  EtronDI_GetDeviceResolutionList(handle_, &dev_sel_info, MAX_STREAM_COUNT,
      stream_color_info_ptr_, MAX_STREAM_COUNT, stream_depth_info_ptr_);

  PETRONDI_STREAM_INFO stream_temp_info_ptr = stream_color_info_ptr_;
  int i = 0;
  while (i < MAX_STREAM_COUNT) {
    if (stream_temp_info_ptr->nWidth > 0) {
      StreamInfo info;
      info.index = i;
      info.width = stream_temp_info_ptr->nWidth;
      info.height = stream_temp_info_ptr->nHeight;
      info.format = stream_temp_info_ptr->bFormatMJPG ?
          StreamFormat::STREAM_MJPG : StreamFormat::STREAM_YUYV;
      color_infos->push_back(info);
    }
    stream_temp_info_ptr++;
    i++;
  }

  stream_temp_info_ptr = stream_depth_info_ptr_;
  i = 0;
  while (i < MAX_STREAM_COUNT) {
    if (stream_temp_info_ptr->nWidth > 0) {
      StreamInfo info;
      info.index = i;
      info.width = stream_temp_info_ptr->nWidth;
      info.height = stream_temp_info_ptr->nHeight;
      info.format = stream_temp_info_ptr->bFormatMJPG ?
          StreamFormat::STREAM_MJPG : StreamFormat::STREAM_YUYV;
      depth_infos->push_back(info);
    }
    stream_temp_info_ptr++;
    i++;
  }
}

bool Device::SetAutoExposureEnabled(bool enabled) {
  if (!IsInitDevice()) {
    LOGE("%s, %d:: Device is not initial.", __FILE__, __LINE__);
    return false;
  }

  bool ok;
  if (enabled) {
    params_member_[ControlParams::AUTO_EXPOSURE].enabled = enabled;
    ok = ETronDI_OK == EtronDI_EnableAE(handle_, &dev_sel_info_);
  } else {
    ok = ETronDI_OK == EtronDI_DisableAE(handle_, &dev_sel_info_);
  }
  if (ok) {
    LOGI("-- Auto-exposure state: %s", enabled ? "enabled" : "disabled");
  } else {
    LOGW("-- %s auto-exposure failed", enabled ? "Enable" : "Disable");
  }
  return ok;
}

bool Device::SetAutoWhiteBalanceEnabled(bool enabled) {
  if (!IsInitDevice()) {
    LOGE("%s, %d:: Device is not initial.", __FILE__, __LINE__);
    return false;
  }

  bool ok;
  if (enabled) {
    params_member_[ControlParams::AUTO_WHITE_BALANCE].enabled = enabled;
    ok = ETronDI_OK == EtronDI_EnableAWB(handle_, &dev_sel_info_);
  } else {
    ok = ETronDI_OK == EtronDI_DisableAWB(handle_, &dev_sel_info_);
  }
  if (ok) {
    LOGI("-- Auto-white balance state: %s", enabled ? "enabled" : "disabled");
  } else {
    LOGW("-- %s auto-white balance failed", enabled ? "Enable" : "Disable");
  }
  return ok;
}

void Device::SetInfraredDepthOnly(const OpenParams& params) {
  if (!params.ir_depth_only) {
    EtronDI_EnableInterleave(handle_, &dev_sel_info_, false);
    return;
  }

  int error_n = 0;
  if (params.dev_mode != DeviceMode::DEVICE_ALL) {
    error_n = 1;
    EtronDI_EnableInterleave(handle_, &dev_sel_info_, false);
  } else if (params.framerate < 30 || params.framerate > 60) {
    error_n = 2;
    EtronDI_EnableInterleave(handle_, &dev_sel_info_, false);
  } else if (params.stream_mode == StreamMode::STREAM_2560x720 &&
      params.framerate != 30) {
    error_n = 3;
    EtronDI_EnableInterleave(handle_, &dev_sel_info_, false);
  } else if ((params.stream_mode == StreamMode::STREAM_1280x720 ||
        params.stream_mode == StreamMode::STREAM_1280x480 ||
        params.stream_mode == StreamMode::STREAM_640x480) &&
      params.framerate != 60) {

    error_n = 4;
    EtronDI_EnableInterleave(handle_, &dev_sel_info_, false);
  }

  if (error_n > 0) {
    throw_error("\n\nThis feature is only available for "
        "[2560x720 30fps] and [1280x720, 1280x480, 640x480 60fps].\n");

    return;
  }

  if (params.color_mode == ColorMode::COLOR_RECTIFIED) {
    color_ir_depth_only_enabled_ = false;
    depth_ir_depth_only_enabled_ = true;
  } else {
    color_ir_depth_only_enabled_ = true;
    depth_ir_depth_only_enabled_ = false;
  }
  ir_depth_only_enabled_ = true;
  EtronDI_EnableInterleave(handle_, &dev_sel_info_, true);
  params_member_[ControlParams::IR_DEPTH_ONLY].enabled = ir_depth_only_enabled_;
  // framerate_ *= 2;
}

void Device::SetInfraredIntensity(const std::uint16_t &value) {
  if (!IsInitDevice()) {
    LOGE("%s, %d:: Device is not initial.", __FILE__, __LINE__);
    return;
  }

  if (value != 0) {
    EtronDI_SetIRMode(handle_, &dev_sel_info_, 0x03);
    EtronDI_SetCurrentIRValue(handle_, &dev_sel_info_, value);
  } else {
    EtronDI_SetCurrentIRValue(handle_, &dev_sel_info_, value);
    EtronDI_SetIRMode(handle_, &dev_sel_info_, 0x00);
  }
  params_member_[ControlParams::IR_INTENSITY].value = value;
}

bool Device::Open(const OpenParams& params) {
  open_params_ = params;
  stream_info_dev_index_ = params.dev_index;

  if (params.stream_mode == StreamMode::STREAM_2560x720 &&
      params.framerate > 30) {
    LOGW("The framerate is too large, please use a smaller value (<=30).");
    return false;
  } else if (params.framerate > 60) {
    LOGW("The framerate is too large, please use a smaller value (<=60).");
    return false;
  }

  dev_sel_info_.index = params.dev_index;

  // using 14 bits
  switch (params.color_mode) {
    case ColorMode::COLOR_RECTIFIED:
      depth_data_type_ = ETronDI_DEPTH_DATA_14_BITS;
      break;
    case ColorMode::COLOR_RAW:
    default:
      depth_data_type_ = ETronDI_DEPTH_DATA_14_BITS_RAW;
      break;
  }

  SetAutoExposureEnabled(params.state_ae);
  SetAutoWhiteBalanceEnabled(params.state_awb);

  if (params.framerate > 0) framerate_ = params.framerate;
  // if (IsUSB2() && params.depth_mode == DepthMode::DEPTH_RAW) {
  //   LOGE("USB 2.0 only supports depth_mode DEPTH_GRAY | DEPTH_COLORFUL, please adjusts params.");  // NOLINT
  //   return false;
  // }
  depth_mode_ = params.depth_mode;

  stream_info_dev_index_ = params.dev_index;
  if (!UpdateStreamInfos()) {
    LOGE("%s, %d:: Get Stream information failed.", __FILE__, __LINE__);
    return false;
  }

  GetStreamIndex(params, &color_res_index_, &depth_res_index_);

  CompatibleUSB2(params);
  CompatibleMJPG(params);

  LOGI("-- Framerate: %d", framerate_);

  EtronDI_SetDepthDataType(handle_, &dev_sel_info_, depth_data_type_);
  DBG_LOGI("SetDepthDataType: %d", depth_data_type_);

  LOGI("-- Color Stream: %dx%d %s",
      stream_color_info_ptr_[color_res_index_].nWidth,
      stream_color_info_ptr_[color_res_index_].nHeight,
      stream_color_info_ptr_[color_res_index_].bFormatMJPG ? "MJPG" : "YUYV");
  int depth_width = stream_depth_info_ptr_[depth_res_index_].nWidth;
  if (IsUSB2()) {
    depth_width *= 2;
  }
  LOGI("-- Depth Stream: %dx%d %s",
      depth_width,
      stream_depth_info_ptr_[depth_res_index_].nHeight,
      stream_depth_info_ptr_[depth_res_index_].bFormatMJPG ? "MJPG" : "YUYV");

  SetInfraredDepthOnly(params);

  if (params.ir_intensity >= 0) {
    SetInfraredIntensity(params.ir_intensity);
    LOGI("\n-- IR intensity: %d", params.ir_intensity);
  }

  ReleaseBuf();

  int ret = OpenDevice(params.dev_mode);

  if (ETronDI_OK == ret) {
    is_device_opened_ = true;
    if (depth_device_opened_) {
      // depth device must be opened.
      SyncCameraCalibrations();
    }
    bool is_8bits = depth_data_type_ == ETronDI_DEPTH_DATA_8_BITS
        || depth_data_type_ == ETronDI_DEPTH_DATA_8_BITS_RAW;  // usb2, 8bits
    if (!stream_color_info_ptr_[color_res_index_].bFormatMJPG &&
        params.dev_mode != DeviceMode::DEVICE_COLOR)
      colorizer_->Init(params.colour_depth_value, is_8bits,
          GetCameraCalibration(params.stream_mode));
    return true;
  } else {
    is_device_opened_ = false;
    dev_sel_info_.index = -1;  // reset flag
    return false;
  }
}

bool Device::IsOpened() const {
  return is_device_opened_;
}

void Device::CheckOpened(const std::string& event) const {
  if (!IsOpened()) {
    std::stringstream s;
    s << "Error: Camera must be opened";
    if (!event.empty()) {
      s << ", before " << event;
    }
    throw_error(s.str());
  }
}

bool Device::ExpectOpened(const std::string& event) const {
  if (IsOpened()) {
    return true;
  } else {
    LOGW("Warning: Camera should be opened, before %s", event);
    return false;
  }
}

OpenParams Device::GetOpenParams() const {
  return open_params_;
}

bool Device::IsRightColorSupported() const {
  CheckOpened(__func__);
  return IsRightColorSupported(open_params_.stream_mode);
}

bool Device::IsRightColorSupported(const StreamMode& stream_mode) const {
  return stream_mode == StreamMode::STREAM_1280x480
      || stream_mode == StreamMode::STREAM_2560x720;
}

std::shared_ptr<CameraCalibration> Device::GetCameraCalibration(
    const StreamMode& stream_mode) {
  switch (stream_mode) {
    case StreamMode::STREAM_640x480:
    case StreamMode::STREAM_1280x480:  // 480p, vga
      return GetCameraCalibration(1);
    case StreamMode::STREAM_1280x720:
    case StreamMode::STREAM_2560x720:  // 720p, hd
      return GetCameraCalibration(0);
    default:
      throw new std::runtime_error("StreamMode is unknown");
  }
}

bool Device::GetCameraCalibrationFile(const StreamMode& stream_mode,
                                      const std::string& filename) {
  switch (stream_mode) {
    case StreamMode::STREAM_640x480:
    case StreamMode::STREAM_1280x480:  // 480p, vga
      return GetCameraCalibrationFile(1, filename);
    case StreamMode::STREAM_1280x720:
    case StreamMode::STREAM_2560x720:  // 720p, hd
      return GetCameraCalibrationFile(0, filename);
    default:
      throw new std::runtime_error("StreamMode is unknown");
  }
}

#ifdef __i386
#define MY_INT long
#else
#define MY_INT int
#endif

typedef struct
{
	char	SN[16];			//16
	long	Version;		// 4
	short	NumOfTable;		// 2
	short	Reserve[13];	//26
} TableHeader;

#define TableHeaderSize  48  // bytes

typedef struct
{
	short	DataOffset;			//2
	short	ElementCount;		//2
	char	DataSize;			//1; 2-bytes, 4-byte
	char	TabID;				//1
	char    Attri; 				//1; T.B.D.
	char	FractionalBit;		//1; 1:16.0; 2:S13.18; 3:S08.23; 4:S18.13; 5:S11.20
	char	Reserve[8];			//8
} TableInfo;

#define TableInfoSize		16 //bytes

#define TAB_ID_UNDEFINE 		0
#define TAB_ID_InOutImgDim 		1
#define TAB_ID_RectFlags 		2
#define TAB_ID_RectScaleDim		3
#define TAB_ID_CamMat1 			4
#define TAB_ID_CamDist1 		5
#define TAB_ID_CamMat2 			6
#define TAB_ID_CamDist2 		7
#define TAB_ID_RotaMat			8
#define TAB_ID_TranMat			9
#define TAB_ID_LRotaMat			10
#define TAB_ID_RRotaMat			11
#define TAB_ID_NewCamMat1		12
#define TAB_ID_NewCamMat2		13
#define TAB_ID_ReprojMat		14
#define TAB_ID_ROI1				15
#define TAB_ID_ROI2				16
#define TAB_ID_ComROI			17
#define TAB_ID_ScaleROI			18
#define TAB_ID_RectCrop			19
#define TAB_ID_RectScaleMN		20
#define TAB_ID_RectScaleC		21
#define TAB_ID_RectErrMeasure	22
#define TAB_ID_LineBuffers		23
#define	TAB_ID_EssentialMat		24
#define	TAB_ID_FundamentalMat	25
#define TAB_ID_ConfidenceVal	26
#define TAB_ID_ConfidenceTH		27
#define TAB_ID_ConfidenceLevel	28


//#define Data_Type_Long			4
//#define Data_Type_Short			2

#define TAB_MAX_NUM 29

// data definition
//---------------------------------------------0--------------5-------------10-------------15-------------20-------------25
static short TAB_ElementCount[TAB_MAX_NUM] 	= {0, 5, 3, 2, 9, 8, 9, 8, 9, 3, 9, 9,12,12,16, 4, 4, 4, 4, 6, 4, 2, 4, 1, 9, 9,16,16, 1};
static char  TAB_DataSize[TAB_MAX_NUM]     	= {2, 2, 2, 2, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2, 2, 2, 2, 2, 2, 4, 4, 2, 4, 4, 4, 4, 2};
//static char  TAB_FractionalBit[TAB_MAX_NUM]	= {0, 0, 0, 0,18,18,18,18,23,23,23,23,13,13,20, 0, 0, 0, 0, 0, 0,23,20, 0,23,23,12, 3, 0}; //bug on item#22
static char  TAB_FractionalBit[TAB_MAX_NUM]	= {0, 0, 0, 0,18,18,18,18,23,23,23,23,13,13,18, 0, 0, 0, 0, 0, 0,23,23, 0,23,23,12, 3, 0};
static char  TAB_Attri[TAB_MAX_NUM]		  	= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

eSPCtrl_RectLogData Device::GetCameraCalibrationWithStruct(unsigned char* DumpBuffer, int nLen) {
  std::cout << "file length: " << nLen << std::endl;
  eSPCtrl_RectLogData data;
  auto pData = &data;
  double M1[9];
  double D1[8];
  double M2[9];
  double D2[8];
  double R[9];
  double T[3];
  double R1[9];
  double R2[9];
  double P1[12];
  double P2[12];
  double Q[16];
  double E[9];
  double F[9];

  float mConfidenceValue[32];
  float mConfidenceTH[32];
  short mConfidenceLevel;

  /*double InvR1[3][3];
  double InvR2[3][3];*/

  //bool EnableFixedROI;      //下一版更新

  short  tempBufferShort[8];
  MY_INT tempBufferLong[16];
  //TCHAR strtest[256];

  short idx = 0;

  LOGI("GetRectifyLogFromCalibrationLog prepare to cpy data...");

  //Table ID 0: Write Log Szie
  memcpy(tempBufferShort,DumpBuffer,sizeof(short)*1);
  int Log_Size  = (int)tempBufferShort[0];        // 2 bytes
  DumpBuffer += 2;
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 0 Log_Size:%d", Log_Size);
  auto thr = (TableHeader *)DumpBuffer;
  std::cout << "thr->NumOfTable" << thr->NumOfTable << std::endl
            << "thr->Reserve[]";
            for (size_t i = 0; i < 13; i++) {
              std::cout << thr->Reserve[i];
            }
            std::cout << std::endl
            << "thr->SN";
            for (size_t i = 0; i < 8; i++) {
              std::cout << thr->SN[i];
            }
            std::cout << std::endl
            << "thr->Version" << thr->Version << std::endl;
  DumpBuffer += TableHeaderSize;

  for (size_t i = 0; i < TAB_MAX_NUM; i++) {
    auto kk = (TableInfo *)DumpBuffer;
    std::cout <<"kk->Attri" <<  kk->Attri << std::endl
              <<"kk->DataOffset" <<  kk->DataOffset << std::endl
              <<"kk->DataSize" <<  kk->DataSize << std::endl
              <<"kk->ElementCount" <<  kk->ElementCount << std::endl
              <<"kk->FractionalBit" <<  kk->FractionalBit << std::endl
              <<"kk->TabID" <<  kk->TabID << std::endl;

    std::cout << "std::cout << kk->Reserve[]";
    for (size_t i = 0; i < 8; i++) {
      std::cout << kk->Reserve[i];
    }
    std::cout << std::endl;

    DumpBuffer += TableInfoSize;
  }

  // DumpBuffer += (TableInfoSize * TAB_MAX_NUM);

  //wsprintf(strtest,L" DumpBuffer = 0x%x   *DumpBuffer = %d\n",DumpBuffer,*DumpBuffer);
  //OutputDebugString(strtest);

  LOGI("GetRectifyLogFromCalibrationLog Table ID 0...");

  //Table ID 1: Write "InOutDim" Entries
  memcpy(tempBufferShort,DumpBuffer,sizeof(short)*TAB_ElementCount[idx]);

  int in_width   = (int)tempBufferShort[0];       // 2 bytes
  int in_height  = (int)tempBufferShort[1];       // 2 bytes
  int out_width  = (int)tempBufferShort[2];       // 2 bytes
  int out_height = (int)tempBufferShort[3];       // 2 bytes
  int color_type = (int)tempBufferShort[4];       // 2 bytes
  std::cout << "idx" << idx << "in_width" << in_width
            << "in_height" << in_height
            << "out_width" << out_width
            << "out_height" << out_height
            << "color_type" << color_type
            << std::endl;
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;
  pData->InImgWidth   = (unsigned short)in_width;
  pData->InImgHeight  = (unsigned short)in_height;
  pData->OutImgWidth  = (unsigned short)out_width;
  pData->OutImgHeight = (unsigned short)out_height;
  //wsprintf(strtest,_T("  width = %d height = %d  width = %d height = %d\n"),in_width,in_height,out_width,  out_height);
  //OutputDebugString(strtest);

  LOGI("GetRectifyLogFromCalibrationLog Table ID 1...");

  //Table ID 2: Write "RECT_Flags" Entries
  memcpy(tempBufferShort,DumpBuffer,sizeof(short)*TAB_ElementCount[idx]);
  bool EnableModule = (bool)tempBufferShort[0];   // 2 bytes
  bool EnableScale  = (bool)tempBufferShort[1];   // 2 bytes
  bool EnableCrop   = (bool)tempBufferShort[2];   // 2 bytes
  std::cout << "EnableModule " << (EnableModule ? "yes " : "no ")
            << "EnableScale " << (EnableScale ? "yes" : "no  ")
            << "EnableCrop " << (EnableCrop ? "yes " : "no ")
            << std::endl;
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  pData->RECT_ScaleEnable = (int)EnableScale;
  pData->RECT_CropEnable = (int)EnableCrop;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 2...");

  //Table ID 3: Write "RECT_ScaleDimension" Entries
  memcpy(tempBufferShort,DumpBuffer,sizeof(short)*TAB_ElementCount[idx]);
  int ScaleWidth  = (int)tempBufferShort[0];      // 2 bytes
  int ScaleHeight = (int)tempBufferShort[1];      // 2 bytes
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  pData->RECT_ScaleWidth  = (unsigned short)ScaleWidth;
  pData->RECT_ScaleHeight = (unsigned short)ScaleHeight;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 3...");

  //Table ID 4: Write "CamMat1" Entries               // 4 bytes
  memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  for(int i=0;i<TAB_ElementCount[idx];i++)
      M1[i] = ((double)(tempBufferLong[i]))/(1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 4...");

  //Table ID 5: Write "CamDist1" Entries              // 4 bytes
  memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  for(int i=0;i<TAB_ElementCount[idx];i++)
      D1[i] = ((double)(tempBufferLong[i]))/(1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 5...");

  //Table ID 6: Write "CamMat2" Entries               // 4 bytes
  memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  for(int i=0;i<TAB_ElementCount[idx];i++)
      M2[i] = ((double)(tempBufferLong[i]))/(1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 6...");

  //Table ID 7: Write "CamDist2" Entries              // 4 bytes
  memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  for(int i=0;i<TAB_ElementCount[idx];i++)
      D2[i] = ((double)(tempBufferLong[i]))/(1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 7...");

  //Table ID 8: Write "RotaMat" Entries               // 4 bytes
  memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  for(int i=0;i<TAB_ElementCount[idx];i++)
      R[i] = ((double)(tempBufferLong[i]))/(1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 8...");

  //Table ID 9: Write "TranMat" Entries               // 4 bytes
  memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  for(int i=0;i<TAB_ElementCount[idx];i++)
      T[i] = ((double)(tempBufferLong[i]))/(1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 9...");

  //Table ID 10: Write "LRotaMat" Entries             // 4 bytes
  memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  for(int i=0;i<TAB_ElementCount[idx];i++)
      R1[i] = ((double)(tempBufferLong[i]))/(1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 10...");

  //Table ID 11: Write "RRotaMat" Entries             // 4 bytes
  memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  for(int i=0;i<TAB_ElementCount[idx];i++)
      R2[i] = ((double)(tempBufferLong[i]))/(1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 11...");

  //Table ID 12: Write "NewCamMat1" Entries           // 4 bytes
  memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  for(int i=0;i<TAB_ElementCount[idx];i++)
      P1[i] = ((double)(tempBufferLong[i]))/(1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 12...");

  //Table ID 13: Write "NewCamMat2" Entries           // 4 bytes
  memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  for(int i=0;i<TAB_ElementCount[idx];i++)
      P2[i] = ((double)(tempBufferLong[i]))/(1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 13...");

  //Table ID 14: Write "ReprojMat" Entries            // 4 bytes
  memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  for (int i=0; i < TAB_ElementCount[idx]; i++)
      Q[i] = ((double)(tempBufferLong[i]))/(1<<TAB_FractionalBit[idx]);

  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  for(int i=0; i<9 ;i++) {
      pData->CamMat1[i] = M1[i];
      pData->CamMat2[i] = M2[i];
      pData->RotaMat[i] = R[i];
      pData->LRotaMat[i] = R1[i];
      pData->RRotaMat[i] = R2[i];
  }

  for(int i=0; i<8 ;i++){
      pData->CamDist1[i] = D1[i];
      pData->CamDist2[i] = D2[i];
  }

  for(int i=0; i<3 ;i++) {
      pData->TranMat[i] = T[i];
  }

  for(int i=0; i<12 ;i++) {
      pData->NewCamMat1[i] = P1[i];
      pData->NewCamMat2[i] = P2[i];
  }

  LOGI("GetRectifyLogFromCalibrationLog Table ID 14...");

  //Table ID 15: Write "ROI1" Entries
  memcpy(tempBufferShort,DumpBuffer,sizeof(short)*TAB_ElementCount[idx]);
  /*RoiL.x =      (int)tempBufferShort[0];        // 2 bytes
  RoiL.y =        (int)tempBufferShort[1];        // 2 bytes
  RoiL.width =    (int)tempBufferShort[2];        // 2 bytes
  RoiL.height =   (int)tempBufferShort[3];        // 2 bytes*/
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 15...");

  //Table ID 16: Write "ROI2" Entries
  memcpy(tempBufferShort,DumpBuffer,sizeof(short)*TAB_ElementCount[idx]);
  /*RoiR.x =      (int)tempBufferShort[0];        // 2 bytes
  RoiR.y =        (int)tempBufferShort[1];        // 2 bytes
  RoiR.width =    (int)tempBufferShort[2];        // 2 bytes
  RoiR.height =   (int)tempBufferShort[3];        // 2 bytes*/
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 16...");

  //Table ID 17: Write "ComROI" Entries
  memcpy(tempBufferShort,DumpBuffer,sizeof(short)*TAB_ElementCount[idx]);
  /*CommonROI.x =     (int)tempBufferShort[0];        // 2 bytes
  CommonROI.y =       (int)tempBufferShort[1];        // 2 bytes
  CommonROI.width =   (int)tempBufferShort[2];        // 2 bytes
  CommonROI.height =  (int)tempBufferShort[3];        // 2 bytes*/
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 17...");

  //Table ID 18: Write "ScaleROI" Entries
  memcpy(tempBufferShort,DumpBuffer,sizeof(short)*TAB_ElementCount[idx]);
  /*SCommonROI.x =        (int)tempBufferShort[0];        // 2 bytes
  SCommonROI.y =      (int)tempBufferShort[1];        // 2 bytes
  SCommonROI.width =  (int)tempBufferShort[2];        // 2 bytes
  SCommonROI.height = (int)tempBufferShort[3];        // 2 bytes*/
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 18...");

  //Table ID 19: Write "RECT_Crop" Entries
  memcpy(tempBufferShort,DumpBuffer,sizeof(short)*TAB_ElementCount[idx]);
  int mCrop_Row_BG   = (int)tempBufferShort[0];       // 2 bytes
  int mCrop_Row_ED   = (int)tempBufferShort[1];       // 2 bytes
  int mCrop_Col_BG_L = (int)tempBufferShort[2];       // 2 bytes
  int mCrop_Col_ED_L = (int)tempBufferShort[3];       // 2 bytes
  int mCrop_Col_BG_R = (int)tempBufferShort[4];       // 2 bytes
  int mCrop_Col_ED_R = (int)tempBufferShort[5];       // 2 bytes
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  pData->RECT_Crop_Row_BG = (unsigned short)mCrop_Row_BG;
  pData->RECT_Crop_Row_ED = (unsigned short)mCrop_Row_ED;
  pData->RECT_Crop_Col_BG_L = (unsigned short)mCrop_Col_BG_L;
  pData->RECT_Crop_Col_ED_L = (unsigned short)mCrop_Col_ED_L;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 19...");

  //Table ID 20: Write "RECT_Scale_MN" Entries
  memcpy(tempBufferShort,DumpBuffer,sizeof(short)*TAB_ElementCount[idx]);
  int mScale_Col_M = (int)tempBufferShort[0];     // 2 bytes
  int mScale_Col_N = (int)tempBufferShort[1];     // 2 bytes
  int mScale_Row_M = (int)tempBufferShort[2];     // 2 bytes
  int mScale_Row_N = (int)tempBufferShort[3];     // 2 bytes
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  pData->RECT_Scale_Col_M = (unsigned char)mScale_Col_M;
  pData->RECT_Scale_Col_N = (unsigned char)mScale_Col_N;
  pData->RECT_Scale_Row_M = (unsigned char)mScale_Row_M;
  pData->RECT_Scale_Row_N = (unsigned char)mScale_Row_N;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 20...");

  //Table ID 21: Write "RECT_Scale_C" Entries
  memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  double mScale_Col_C = ((double)(tempBufferLong[0]))/(1<<TAB_FractionalBit[idx]);
  double mScale_Row_C = ((double)(tempBufferLong[1]))/(1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 21...");

  //Table ID 22: Write "RECT_ErrMeasure" Entries      4 bytes
  memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  double  avgReprojectError = ((double)(tempBufferLong[0]))/(1<<TAB_FractionalBit[idx]);
  double  minYShiftError    = ((double)(tempBufferLong[1]))/(1<<TAB_FractionalBit[idx]);
  double  maxYShiftError    = ((double)(tempBufferLong[2]))/(1<<TAB_FractionalBit[idx]);
  double  avgYShiftError    = ((double)(tempBufferLong[3]))/(1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  pData->RECT_AvgErr = (float)avgReprojectError;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 22...");

  //Table ID 23: Write "LineBufer" Entries
  memcpy(tempBufferShort,DumpBuffer,sizeof(short)*TAB_ElementCount[idx]);
  int lineBuffer = (int)tempBufferShort[0];       // 2 bytes
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  pData->nLineBuffers = (unsigned short)lineBuffer;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 23...");

  //Table ID 24: Write "EssentialMat" Entries             // 4 bytes;
  memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  for(int i=0;i<TAB_ElementCount[idx];i++)
      E[i] = ((double)(tempBufferLong[i]))/(1<<TAB_FractionalBit[idx]);

  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 24...");

  //Table ID 25: Write "fundamentalMat" Entries           // 4 bytes;
  memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  for(int i=0;i<TAB_ElementCount[idx];i++)
      F[i] = ((double)(tempBufferLong[i]))/(1<<TAB_FractionalBit[idx]);

  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 25...");

  //Table ID 26: Write "ConfidenceVal" Entries            // 4 bytes;
  memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  for(int i=0;i<TAB_ElementCount[idx];i++)
      mConfidenceValue[i] =  ((float)(tempBufferLong[i]))/(1<<TAB_FractionalBit[idx]);

  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 26...");

  //Table ID 27: Write "ConfidenceTH" Entries
  memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  for (int i = 0; i < TAB_ElementCount[idx]; i++)
      mConfidenceTH[i] = ((float)(tempBufferLong[i]))/(1<<TAB_FractionalBit[idx]);

  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  LOGI("GetRectifyLogFromCalibrationLog Table ID 27...");

  //Table ID 28: Write "ConfidenceLevel" Entries
  memcpy(tempBufferShort,DumpBuffer,sizeof(short)*TAB_ElementCount[idx]);
  mConfidenceLevel = (int)tempBufferShort[0];  // 2 bytes
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);

  LOGI("GetRectifyLogFromCalibrationLog Table ID 28...");
  LOGI("GetRectifyLogFromCalibrationLog cpy data ready...");
  return data;
}

bool Device::SetCameraCalibrationBinFile(const std::string& filename) {
  std::ifstream t;
  int length;
  t.open(filename.c_str());
  t.seekg(0, std::ios::end);
  length = t.tellg();
  t.seekg(0, std::ios::beg);
  char* buffer = new char[length];
  t.read(buffer, length);
  t.close();

  int nActualLength = 0;
  // GetCameraCalibrationWithStruct((unsigned char *)buffer, length);
  // bool ok = true;

  bool ok = (ETronDI_OK == EtronDI_SetLogData(handle_, &dev_sel_info_,
      (unsigned char*)buffer, length, &nActualLength, 0));
  if (!ok) printf("error when setLogData\n");
  delete[] buffer;

  SyncCameraCalibrations();
  return ok;
}

bool Device::SetCameraCalibrationWithStruct(
    const struct CameraCalibration& data, const StreamMode& stream_mode) {
  std::ifstream t;
  int length = 4096;
  char* buffer = new char[length];
  GetCameraCalibrationWithStruct(buffer, data);
  bool ok = false;

  int nActualLength = 0;
  switch (stream_mode) {
    case StreamMode::STREAM_640x480:
    case StreamMode::STREAM_1280x480:
    ok = (ETronDI_OK == EtronDI_SetLogData(handle_, &dev_sel_info_,
      (unsigned char*)buffer, length, &nActualLength, 1));
      break;
    case StreamMode::STREAM_1280x720:
    case StreamMode::STREAM_2560x720:
    ok = (ETronDI_OK == EtronDI_SetLogData(handle_, &dev_sel_info_,
      (unsigned char*)buffer, length, &nActualLength, 0));
      break;
    default:
      throw new std::runtime_error("StreamMode is unknown");
  }
  if (!ok) printf("error when setLogData\n");
  delete[] buffer;
  if (ok) std::cout << "write calib info success!" << std::endl;
  SyncCameraCalibrations();
  return ok;
}

void Device::GetCameraCalibrationWithStruct(char* DumpBuffer,
    const struct CameraCalibration& data) {  // NOLINT
  auto pData = &data;
  double M1[9];
  double D1[8];
  double M2[9];
  double D2[8];
  double R[9];
  double T[3];
  double R1[9];
  double R2[9];
  double P1[12];
  double P2[12];
  double Q[16];
  double E[9];
  double F[9];

  for(int i=0; i<9 ;i++) {
    M1[i] =  pData->CamMat1[i];
    M2[i] =  pData->CamMat2[i];
    R[i] = pData->RotaMat[i];
    R1[i] = pData->LRotaMat[i];
    R2[i] = pData->RRotaMat[i];
  }

  for(int i=0; i<8 ;i++){
    D1[i] = pData->CamDist1[i];
    D2[i] = pData->CamDist2[i];
  }

  for(int i=0; i<3 ;i++) {
    T[i] = pData->TranMat[i];
  }

  for(int i=0; i<12 ;i++) {
    P1[i] = pData->NewCamMat1[i];
    P2[i] = pData->NewCamMat2[i];
  }

  float mConfidenceValue[32];
  float mConfidenceTH[32];
  short mConfidenceLevel;

  /*double InvR1[3][3];
  double InvR2[3][3];*/

  //bool EnableFixedROI;      //下一版更新

  short  *tempBufferShort;
  MY_INT *tempBufferLong;
  //TCHAR strtest[256];

  short idx = 0;

  int Log_Size  = (int)tempBufferShort[0];        // 2 bytes
  short *size_l = (short *)DumpBuffer;
  *size_l = 1230;
  DumpBuffer += 2;
  idx++;

  DumpBuffer += (TableHeaderSize + TableInfoSize * TAB_MAX_NUM);

  // wsprintf(strtest,L" DumpBuffer = 0x%x   *DumpBuffer = %d\n",DumpBuffer,*DumpBuffer);
  // OutputDebugString(strtest);

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 0...");

  // Table ID 1: Write "InOutDim" Entries
  // memcpy(tempBufferShort,DumpBuffer,sizeof(short)*TAB_ElementCount[idx]);
  tempBufferShort = (short *)DumpBuffer;

  tempBufferShort[0] = pData->InImgWidth;
  tempBufferShort[1] = pData->InImgHeight;
  tempBufferShort[2] = pData->OutImgWidth;
  tempBufferShort[3] = pData->OutImgHeight;
  tempBufferShort[4] = 3;
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 1...");

  tempBufferShort = (short *)DumpBuffer;
  tempBufferShort[0] = 1;
  tempBufferShort[1] = pData->RECT_ScaleEnable;
  tempBufferShort[2] = pData->RECT_CropEnable;
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 2...");

  tempBufferShort = (short *)DumpBuffer;
  tempBufferShort[0] = pData->RECT_ScaleWidth;
  tempBufferShort[1] = pData->RECT_ScaleHeight;
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 3...");

  tempBufferLong = (MY_INT *)DumpBuffer;
  for(int i=0;i<TAB_ElementCount[idx];i++)
    tempBufferLong[i] =M1[i] * (1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 4...");

  tempBufferLong = (MY_INT *)DumpBuffer;
  for (int i = 0; i < TAB_ElementCount[idx]; i++)
      tempBufferLong[i] = D1[i] * (1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 5...");

  tempBufferLong = (MY_INT *)DumpBuffer;
  for(int i=0;i<TAB_ElementCount[idx];i++)
      tempBufferLong[i] = M2[i] * (1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 6...");

  tempBufferLong = (MY_INT *)DumpBuffer;
  for(int i=0;i<TAB_ElementCount[idx];i++)
      tempBufferLong[i] = D2[i] * (1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 7...");

  tempBufferLong = (MY_INT *)DumpBuffer;
  for(int i=0;i<TAB_ElementCount[idx];i++)
      tempBufferLong[i] = R[i] * (1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 8...");

  tempBufferLong = (MY_INT *)DumpBuffer;
  for(int i=0;i<TAB_ElementCount[idx];i++)
      tempBufferLong[i] = T[i] * (1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 9...");

  tempBufferLong = (MY_INT *)DumpBuffer;
  for(int i=0;i<TAB_ElementCount[idx];i++)
      tempBufferLong[i] = R1[i] * (1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 10...");

  tempBufferLong = (MY_INT *)DumpBuffer;
  for(int i=0;i<TAB_ElementCount[idx];i++)
      tempBufferLong[i] = R2[i] * (1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 11...");

  tempBufferLong = (MY_INT *)DumpBuffer;
  for(int i=0;i<TAB_ElementCount[idx];i++)
      tempBufferLong[i] = P1[i] * (1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 12...");

  tempBufferLong = (MY_INT *)DumpBuffer;
  for(int i=0;i<TAB_ElementCount[idx];i++)
      tempBufferLong[i] = P2[i] * (1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 13...");

  tempBufferLong = (MY_INT *)DumpBuffer;
  for (int i=0; i < TAB_ElementCount[idx]; i++)
      tempBufferLong[i] = Q[i] * (1<<TAB_FractionalBit[idx]);

  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 14...");

  // tempBufferShort = (short *)DumpBuffer;
  // /*RoiL.x =      (int)tempBufferShort[0];        // 2 bytes
  // RoiL.y =        (int)tempBufferShort[1];        // 2 bytes
  // RoiL.width =    (int)tempBufferShort[2];        // 2 bytes
  // RoiL.height =   (int)tempBufferShort[3];        // 2 bytes*/
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // //LOGI("GetRectifyLogFromCalibrationLog Table ID 15...");

  // //Table ID 16: Write "ROI2" Entries
  // memcpy(tempBufferShort,DumpBuffer,sizeof(short)*TAB_ElementCount[idx]);
  // /*RoiR.x =      (int)tempBufferShort[0];        // 2 bytes
  // RoiR.y =        (int)tempBufferShort[1];        // 2 bytes
  // RoiR.width =    (int)tempBufferShort[2];        // 2 bytes
  // RoiR.height =   (int)tempBufferShort[3];        // 2 bytes*/
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // //LOGI("GetRectifyLogFromCalibrationLog Table ID 16...");

  // //Table ID 17: Write "ComROI" Entries
  // memcpy(tempBufferShort,DumpBuffer,sizeof(short)*TAB_ElementCount[idx]);
  // /*CommonROI.x =     (int)tempBufferShort[0];        // 2 bytes
  // CommonROI.y =       (int)tempBufferShort[1];        // 2 bytes
  // CommonROI.width =   (int)tempBufferShort[2];        // 2 bytes
  // CommonROI.height =  (int)tempBufferShort[3];        // 2 bytes*/
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // //LOGI("GetRectifyLogFromCalibrationLog Table ID 17...");

  // //Table ID 18: Write "ScaleROI" Entries
  // memcpy(tempBufferShort,DumpBuffer,sizeof(short)*TAB_ElementCount[idx]);
  // /*SCommonROI.x =        (int)tempBufferShort[0];        // 2 bytes
  // SCommonROI.y =      (int)tempBufferShort[1];        // 2 bytes
  // SCommonROI.width =  (int)tempBufferShort[2];        // 2 bytes
  // SCommonROI.height = (int)tempBufferShort[3];        // 2 bytes*/
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // //LOGI("GetRectifyLogFromCalibrationLog Table ID 18...");

  // //Table ID 19: Write "RECT_Crop" Entries
  tempBufferShort = (short *)DumpBuffer;
  tempBufferShort[0] = pData->RECT_Crop_Row_BG;       // 2 bytes
  tempBufferShort[1] = pData->RECT_Crop_Row_ED;       // 2 bytes
  tempBufferShort[2] = pData->RECT_Crop_Col_BG_L;       // 2 bytes
  tempBufferShort[3] = pData->RECT_Crop_Col_ED_L;       // 2 bytes
  // int mCrop_Col_BG_R = (int)tempBufferShort[4];       // 2 bytes
  // int mCrop_Col_ED_R = (int)tempBufferShort[5];       // 2 bytes
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // pData->RECT_Crop_Row_BG = (unsigned short)mCrop_Row_BG;
  // pData->RECT_Crop_Row_ED = (unsigned short)mCrop_Row_ED;
  // pData->RECT_Crop_Col_BG_L = (unsigned short)mCrop_Col_BG_L;
  // pData->RECT_Crop_Col_ED_L = (unsigned short)mCrop_Col_ED_L;

  // //LOGI("GetRectifyLogFromCalibrationLog Table ID 19...");

  // Table ID 20: Write "RECT_Scale_MN" Entries
  tempBufferShort = (short *)DumpBuffer;
  tempBufferShort[0] = pData->RECT_Scale_Col_M;     // 2 bytes
  tempBufferShort[1] = pData->RECT_Scale_Col_N;     // 2 bytes
  tempBufferShort[2] = pData->RECT_Scale_Row_M;     // 2 bytes
  tempBufferShort[3] = pData->RECT_Scale_Row_N;     // 2 bytes
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // pData->RECT_Scale_Col_M = (unsigned char)mScale_Col_M;
  // pData->RECT_Scale_Col_N = (unsigned char)mScale_Col_N;
  // pData->RECT_Scale_Row_M = (unsigned char)mScale_Row_M;
  // pData->RECT_Scale_Row_N = (unsigned char)mScale_Row_N;

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 20...");

  // Table ID 21: Write "RECT_Scale_C" Entries
  // memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  tempBufferLong = (MY_INT *)DumpBuffer;
  // double mScale_Col_C = ((double)(tempBufferLong[0]))/(1<<TAB_FractionalBit[idx]);
  // double mScale_Row_C = ((double)(tempBufferLong[1]))/(1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // //LOGI("GetRectifyLogFromCalibrationLog Table ID 21...");

  // //Table ID 22: Write "RECT_ErrMeasure" Entries      4 bytes
  // memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  tempBufferLong = (MY_INT *)DumpBuffer;
  tempBufferLong[0]= pData->RECT_AvgErr * (1<<TAB_FractionalBit[idx]);
  // double  minYShiftError    = ((double)(tempBufferLong[1]))/(1<<TAB_FractionalBit[idx]);
  // double  maxYShiftError    = ((double)(tempBufferLong[2]))/(1<<TAB_FractionalBit[idx]);
  // double  avgYShiftError    = ((double)(tempBufferLong[3]))/(1<<TAB_FractionalBit[idx]);
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // pData->RECT_AvgErr = (float)avgReprojectError;

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 22...");

  tempBufferShort = (short *)DumpBuffer;
  tempBufferShort[0] = pData->nLineBuffers;       // 2 bytes
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // pData->nLineBuffers = (unsigned short)lineBuffer;

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 23...");

  // //Table ID 24: Write "EssentialMat" Entries             // 4 bytes;
  // memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  // for(int i=0;i<TAB_ElementCount[idx];i++)
  //     E[i] = ((double)(tempBufferLong[i]))/(1<<TAB_FractionalBit[idx]);

  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // //LOGI("GetRectifyLogFromCalibrationLog Table ID 24...");

  // //Table ID 25: Write "fundamentalMat" Entries           // 4 bytes;
  // memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  // for(int i=0;i<TAB_ElementCount[idx];i++)
  //     F[i] = ((double)(tempBufferLong[i]))/(1<<TAB_FractionalBit[idx]);

  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // //LOGI("GetRectifyLogFromCalibrationLog Table ID 25...");
  tempBufferLong = (MY_INT *)DumpBuffer;
  // //Table ID 26: Write "ConfidenceVal" Entries            // 4 bytes;
  // memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  for(int i=0;i<TAB_ElementCount[idx];i++)
      tempBufferLong[i] = mConfidenceValue[i] * (1<<TAB_FractionalBit[idx]);

  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 26...");
  tempBufferLong = (MY_INT *)DumpBuffer;
  // //Table ID 27: Write "ConfidenceTH" Entries
  // memcpy(tempBufferLong,DumpBuffer,sizeof(MY_INT)*TAB_ElementCount[idx]);
  for(int i=0;i<TAB_ElementCount[idx];i++)
      tempBufferLong[i] = mConfidenceTH[i] * (1<<TAB_FractionalBit[idx]);

  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);
  idx++;

  // LOGI("GetRectifyLogFromCalibrationLog Table ID 27...");

  // //Table ID 28: Write "ConfidenceLevel" Entries
  // memcpy(tempBufferShort,DumpBuffer,sizeof(short)*TAB_ElementCount[idx]);
  tempBufferShort = (short *)DumpBuffer;
  tempBufferShort[0] = mConfidenceLevel;  // 2 bytes
  DumpBuffer += (TAB_DataSize[idx]*TAB_ElementCount[idx]);

  // //LOGI("GetRectifyLogFromCalibrationLog Table ID 28...");
  // //LOGI("GetRectifyLogFromCalibrationLog cpy data ready...");
}

void Device::Close() {
  if (dev_sel_info_.index != -1) {
    EtronDI_CloseDevice(handle_, &dev_sel_info_);
    is_device_opened_ = false;
    dev_sel_info_.index = -1;
  }
  ReleaseBuf();
  EtronDI_Release(&handle_);
}

void Device::GetStreamIndex(const OpenParams& params,
    int* color_res_index,
    int* depth_res_index) {
  return GetStreamIndex(
      params.dev_index, params.stream_mode,
      params.color_stream_format, params.depth_stream_format,
      color_res_index, depth_res_index);
}

void Device::GetStreamIndex(const std::int32_t& dev_index,
    const StreamMode& stream_mode,
    const StreamFormat& color_stream_format,
    const StreamFormat& depth_stream_format,
    int* color_res_index,
    int* depth_res_index) {
  if (!color_res_index) {
    LOGE("GetStreamIndex: color_res_index is null.");
    return;
  }
  if (!depth_res_index) {
    LOGE("GetStreamIndex: depth_res_index is null.");
    return;
  }

  *color_res_index = -1;
  *depth_res_index = -1;

  int width = 0, height = 0;
  get_stream_size(stream_mode, &width, &height);

  memset(stream_color_info_ptr_, 0, sizeof(ETRONDI_STREAM_INFO) * MAX_STREAM_COUNT);
  memset(stream_depth_info_ptr_, 0, sizeof(ETRONDI_STREAM_INFO) * MAX_STREAM_COUNT);

  DEVSELINFO dev_sel_info{dev_index};
  EtronDI_GetDeviceResolutionList(handle_, &dev_sel_info, MAX_STREAM_COUNT,
      stream_color_info_ptr_, MAX_STREAM_COUNT, stream_depth_info_ptr_);

  PETRONDI_STREAM_INFO stream_temp_info_ptr = stream_color_info_ptr_;
  int i = 0;
  while (i < 64) {
    if (stream_temp_info_ptr->nWidth == width &&
        stream_temp_info_ptr->nHeight == height &&
        color_stream_format == (stream_temp_info_ptr->bFormatMJPG ?
            StreamFormat::STREAM_MJPG : StreamFormat::STREAM_YUYV)) {
      *color_res_index = i;
      break;
    }
    stream_temp_info_ptr++;
    i++;
  }

  if (*color_res_index == -1) {
    LOGE("Error: Color Mode width[%d] height[%d] format[%s] not support. "
        "Please check the resolution list.", width, height,
        get_stream_format_string(color_stream_format));
    *color_res_index = 0;
  }

  stream_temp_info_ptr = stream_depth_info_ptr_;
  i = 0;
  while (i < 64) {
    if (stream_temp_info_ptr->nHeight == height &&
        depth_stream_format == (stream_temp_info_ptr->bFormatMJPG ?
            StreamFormat::STREAM_MJPG : StreamFormat::STREAM_YUYV)) {
      *depth_res_index = i;
      break;
    }
    stream_temp_info_ptr++;
    i++;
  }

  if (*depth_res_index == -1) {
    LOGE("Error: Depth Mode width[%d] height[%d] format[%s] not support. "
        "Please check the resolution list.", width, height,
        get_stream_format_string(depth_stream_format));
    *depth_res_index = 0;
  }
}

bool Device::GetSensorRegister(int id, std::uint16_t address,
    std::uint16_t* value, int flag) {
  if (!ExpectOpened(__func__)) return false;
#ifdef MYNTEYE_OS_WIN
  return ETronDI_OK == EtronDI_GetSensorRegister(handle_, &dev_sel_info_, id,
      address, value, flag, 2);
#else
  return ETronDI_OK == EtronDI_GetSensorRegister(handle_, &dev_sel_info_, id,
      address, value, flag, SENSOR_BOTH);
#endif
}

bool Device::GetHWRegister(std::uint16_t address, std::uint16_t* value,
    int flag) {
  if (!ExpectOpened(__func__)) return false;
  return ETronDI_OK == EtronDI_GetHWRegister(handle_, &dev_sel_info_,
      address, value, flag);
}

bool Device::GetFWRegister(std::uint16_t address, std::uint16_t* value,
    int flag) {
  if (!ExpectOpened(__func__)) return false;
  return ETronDI_OK == EtronDI_GetFWRegister(handle_, &dev_sel_info_, address,
      value, flag);
}

bool Device::SetSensorRegister(int id, std::uint16_t address,
    std::uint16_t value, int flag) {
  if (!ExpectOpened(__func__)) return false;
#ifdef MYNTEYE_OS_WIN
  return ETronDI_OK == EtronDI_SetSensorRegister(handle_, &dev_sel_info_, id,
      address, value, flag, 2);
#else
  return ETronDI_OK == EtronDI_SetSensorRegister(handle_, &dev_sel_info_, id,
      address, value, flag, SENSOR_BOTH);
#endif
}

bool Device::SetHWRegister(std::uint16_t address, std::uint16_t value,
    int flag) {
  if (!ExpectOpened(__func__)) return false;

  params_member_[ControlParams::HW_REGISTER].address = address;
  params_member_[ControlParams::HW_REGISTER].value = value;
  params_member_[ControlParams::HW_REGISTER].flag = flag;
  return ETronDI_OK == EtronDI_SetHWRegister(handle_, &dev_sel_info_, address,
      value, flag);
}

bool Device::SetFWRegister(std::uint16_t address, std::uint16_t value,
    int flag) {
  if (!ExpectOpened(__func__)) return false;

  params_member_[ControlParams::FW_REGISTER].address = address;
  params_member_[ControlParams::FW_REGISTER].value = value;
  params_member_[ControlParams::FW_REGISTER].flag = flag;
  return ETronDI_OK == EtronDI_SetFWRegister(handle_, &dev_sel_info_, address,
      value, flag);
}

std::shared_ptr<CameraCalibration> Device::GetCameraCalibration(int index) {
  return camera_calibrations_[index];
}

bool Device::GetCameraCalibrationFile(int index, const std::string& filename) {
  if (!ExpectOpened(__func__)) return false;
  int nRet;

  // for parse log test
  eSPCtrl_RectLogData eSPRectLogData;
  // EtronDI_GetRectifyLogData in puma
  // EtronDI_GetRectifyMatLogData
  nRet = EtronDI_GetRectifyMatLogData(handle_,
    &dev_sel_info_, &eSPRectLogData, index);
  // printf("nRet = %d", nRet);
  if (nRet != ETronDI_OK) {
    return false;
  }

  FILE *pfile;
  // char buf[128];
  // sprintf(buf, "RectfyLog_PUMA_%d.txt", index);
  pfile = fopen(/*buf*/filename.c_str(), "wt");
  if (pfile != NULL) {
    int i;
    fprintf(pfile, "InImgWidth = %d\n",        eSPRectLogData.InImgWidth);
    fprintf(pfile, "InImgHeight = %d\n",       eSPRectLogData.InImgHeight);
    fprintf(pfile, "OutImgWidth = %d\n",       eSPRectLogData.OutImgWidth);
    fprintf(pfile, "OutImgHeight = %d\n",      eSPRectLogData.OutImgHeight);
    //
    fprintf(pfile, "RECT_ScaleWidth = %d\n",   eSPRectLogData.RECT_ScaleWidth);
    fprintf(pfile, "RECT_ScaleHeight = %d\n",  eSPRectLogData.RECT_ScaleHeight);
    //
    fprintf(pfile, "CamMat1 = ");
    for (i=0; i < 9; i++) {
        fprintf(pfile, "%.8f, ",  eSPRectLogData.CamMat1[i]);
    }
    fprintf(pfile, "\n");
    //
    fprintf(pfile, "CamDist1 = ");
    for (i=0; i < 8; i++) {
        fprintf(pfile, "%.8f, ",  eSPRectLogData.CamDist1[i]);
    }
    fprintf(pfile, "\n");
    //
    fprintf(pfile, "CamMat2 = ");
    for (i=0; i < 9; i++) {
        fprintf(pfile, "%.8f, ",  eSPRectLogData.CamMat2[i]);
    }
    fprintf(pfile, "\n");
    //
    fprintf(pfile, "CamDist2 = ");
    for (i=0; i < 8; i++) {
        fprintf(pfile, "%.8f, ",  eSPRectLogData.CamDist2[i]);
    }
    fprintf(pfile, "\n");
    //
    fprintf(pfile, "RotaMat = ");
    for (i=0; i < 9; i++) {
        fprintf(pfile, "%.8f, ",  eSPRectLogData.RotaMat[i]);
    }
    fprintf(pfile, "\n");
    //
    fprintf(pfile, "TranMat = ");
    for (i=0; i < 3; i++) {
        fprintf(pfile, "%.8f, ",  eSPRectLogData.TranMat[i]);
    }
    fprintf(pfile, "\n");
    //
    fprintf(pfile, "LRotaMat = ");
    for (i=0; i < 9; i++) {
        fprintf(pfile, "%.8f, ",  eSPRectLogData.LRotaMat[i]);
    }
    fprintf(pfile, "\n");
    //
    fprintf(pfile, "RRotaMat = ");
    for (i=0; i < 9; i++) {
        fprintf(pfile, "%.8f, ",  eSPRectLogData.RRotaMat[i]);
    }
    fprintf(pfile, "\n");
    //
    fprintf(pfile, "NewCamMat1 = ");
    for (i=0; i < 12; i++) {
        fprintf(pfile, "%.8f, ",  eSPRectLogData.NewCamMat1[i]);
    }
    fprintf(pfile, "\n");
    //
    fprintf(pfile, "NewCamMat2 = ");
    for (i=0; i < 12; i++) {
        fprintf(pfile, "%.8f, ",  eSPRectLogData.NewCamMat2[i]);
    }
    fprintf(pfile, "\n");
    //
    fprintf(pfile, "RECT_Crop_Row_BG = %d\n",
      eSPRectLogData.RECT_Crop_Row_BG);
    fprintf(pfile, "RECT_Crop_Row_ED = %d\n",
      eSPRectLogData.RECT_Crop_Row_ED);
    fprintf(pfile, "RECT_Crop_Col_BG_L = %d\n",
      eSPRectLogData.RECT_Crop_Col_BG_L);
    fprintf(pfile, "RECT_Crop_Col_ED_L = %d\n",
      eSPRectLogData.RECT_Crop_Col_ED_L);
    fprintf(pfile, "RECT_Scale_Col_M = %d\n",
      eSPRectLogData.RECT_Scale_Col_M);
    fprintf(pfile, "RECT_Scale_Col_N = %d\n",
      eSPRectLogData.RECT_Scale_Col_N);
    fprintf(pfile, "RECT_Scale_Row_M = %d\n",
      eSPRectLogData.RECT_Scale_Row_M);
    fprintf(pfile, "RECT_Scale_Row_N = %d\n",
      eSPRectLogData.RECT_Scale_Row_N);
    //
    fprintf(pfile, "RECT_AvgErr = %.8f\n", eSPRectLogData.RECT_AvgErr);
    //
    fprintf(pfile, "nLineBuffers = %d\n",  eSPRectLogData.nLineBuffers);
    //
    printf("file ok\n");
    fprintf(pfile, "ReProjectMat = ");
    for (i = 0; i < 16; i++) {
      fprintf(pfile, "%.8f, ", eSPRectLogData.ReProjectMat[i]);
    }
    fprintf(pfile, "\n");
  }
  if(pfile != nullptr) {
    fclose(pfile);
  }
  return true;
}

void Device::SyncCameraCalibrations() {
  // if (SetCameraCalibrationBinFile(
  //     "/home/tiny/develop/MYNT-EYE-D-SDK/00000001rectify_log_read0")) {
  //   std::cout << "success" << std::endl;
  // }

  if (!ExpectOpened(__func__)) return;
  camera_calibrations_.clear();
  for (int index = 0; index < 2; index++) {
    eSPCtrl_RectLogData eSPRectLogData;
    int ret = EtronDI_GetRectifyMatLogData(handle_, &dev_sel_info_,
        &eSPRectLogData, index);
    if (ret != ETronDI_OK) {
      return;
    }
    // SetCameraCalibrationWithStruct(eSPRectLogData);
    int i;
    auto camera_calib = std::make_shared<CameraCalibration>();
    camera_calib->InImgWidth = eSPRectLogData.InImgWidth;
    camera_calib->InImgHeight = eSPRectLogData.InImgHeight;
    camera_calib->OutImgWidth = eSPRectLogData.OutImgWidth;
    camera_calib->OutImgHeight = eSPRectLogData.OutImgHeight;
    camera_calib->RECT_ScaleWidth = eSPRectLogData.RECT_ScaleWidth;
    camera_calib->RECT_ScaleHeight = eSPRectLogData.RECT_ScaleHeight;
    for (i=0; i < 9; i++) {
      camera_calib->CamMat1[i] = eSPRectLogData.CamMat1[i];
    }
    for (i=0; i < 8; i++) {
      camera_calib->CamDist1[i] = eSPRectLogData.CamDist1[i];
    }
    for (i=0; i < 9; i++) {
      camera_calib->CamMat2[i] = eSPRectLogData.CamMat2[i];
    }
    for (i=0; i < 8; i++) {
      camera_calib->CamDist2[i] = eSPRectLogData.CamDist2[i];
    }
    for (i=0; i < 9; i++) {
      camera_calib->RotaMat[i] = eSPRectLogData.RotaMat[i];
    }
    for (i=0; i < 3; i++) {
      camera_calib->TranMat[i] = eSPRectLogData.TranMat[i];
    }
    for (i=0; i < 9; i++) {
      camera_calib->LRotaMat[i] = eSPRectLogData.LRotaMat[i];
    }
    for (i=0; i < 9; i++) {
      camera_calib->RRotaMat[i] = eSPRectLogData.RRotaMat[i];
    }
    for (i=0; i < 12; i++) {
      camera_calib->NewCamMat1[i] = eSPRectLogData.NewCamMat1[i];
    }
    for (i=0; i < 12; i++) {
      camera_calib->NewCamMat2[i] = eSPRectLogData.NewCamMat2[i];
    }
    camera_calib->RECT_Crop_Row_BG = eSPRectLogData.RECT_Crop_Row_BG;
    camera_calib->RECT_Crop_Row_ED = eSPRectLogData.RECT_Crop_Row_ED;
    camera_calib->RECT_Crop_Col_BG_L = eSPRectLogData.RECT_Crop_Col_BG_L;
    camera_calib->RECT_Crop_Col_ED_L = eSPRectLogData.RECT_Crop_Col_ED_L;
    camera_calib->RECT_Scale_Col_M = eSPRectLogData.RECT_Scale_Col_M;
    camera_calib->RECT_Scale_Col_N = eSPRectLogData.RECT_Scale_Col_N;
    camera_calib->RECT_Scale_Row_M = eSPRectLogData.RECT_Scale_Row_M;
    camera_calib->RECT_Scale_Row_N = eSPRectLogData.RECT_Scale_Row_N;
    camera_calib->RECT_AvgErr = eSPRectLogData.RECT_AvgErr;
    camera_calib->nLineBuffers = eSPRectLogData.nLineBuffers;
    for (i = 0; i < 16; i++) {
      camera_calib->ReProjectMat[i] = eSPRectLogData.ReProjectMat[i];
    }
    camera_calibrations_.push_back(camera_calib);
  }
}

void Device::ReleaseBuf() {
  color_image_buf_ = nullptr;
  depth_image_buf_ = nullptr;
}

void Device::CompatibleUSB2(const OpenParams& params) {
  if (!IsUSB2() || params.color_stream_format
      == StreamFormat::STREAM_MJPG) {
    return;
  }

  switch (params.color_mode) {
    case ColorMode::COLOR_RECTIFIED:
      depth_data_type_ = ETronDI_DEPTH_DATA_8_BITS;
      break;
    case ColorMode::COLOR_RAW:
    default:
      depth_data_type_ = ETronDI_DEPTH_DATA_8_BITS_RAW;
      break;
  }

  if (params.dev_mode == DeviceMode::DEVICE_ALL) {
  // color + depth

    if (params.stream_mode == StreamMode::STREAM_2560x720) {
      // color 2560x720 yuyv, depth 640x720 yuyv, 8 bits rectify, fail
      goto usb2_error;
    } else if (params.stream_mode == StreamMode::STREAM_1280x720) {
      // color 1280x720 yuyv, depth 640x720 yuyv, 8 bits only 5 ok

      color_res_index_ =
          GetStreamIndex(stream_color_info_ptr_, 1280, 720, false);
      depth_res_index_ =
          GetStreamIndex(stream_depth_info_ptr_, 640, 720, false);
      framerate_ = 5;
      open_params_.framerate = 5;
    } else if (params.stream_mode == StreamMode::STREAM_1280x480) {
      // color 1280x480 yuyv, depth 320x480 yuyv, 8 bits only 15 fail
      goto usb2_error;
    } else if (params.stream_mode == StreamMode::STREAM_640x480) {
      // color 640x480 yuyv, depth 320x480 yuyv, 8 bits only 15 ok

      color_res_index_ =
          GetStreamIndex(stream_color_info_ptr_, 640, 480, false);
      depth_res_index_ =
          GetStreamIndex(stream_depth_info_ptr_, 320, 480, false);
      framerate_ = 15;
      open_params_.framerate = 15;
    } else {
      goto usb2_error;
    }
    if (color_res_index_ == -1 || depth_res_index_ == -1) {
      goto usb2_error;
    }
  } else if (params.dev_mode == DeviceMode::DEVICE_COLOR) {
  // color only
    if (params.stream_mode == StreamMode::STREAM_2560x720) {
      // color 2560x720 yuyv, only 5 ok
      color_res_index_ =
          GetStreamIndex(stream_color_info_ptr_, 2560, 720, false);
      framerate_ = 5;
      open_params_.framerate = 5;
    } else if (params.stream_mode == StreamMode::STREAM_1280x720) {
      // color 1280x720 yuyv, only 10 ok

      color_res_index_ =
          GetStreamIndex(stream_color_info_ptr_, 1280, 720, false);
      framerate_ = 10;
      open_params_.framerate = 10;
    } else if (params.stream_mode == StreamMode::STREAM_1280x480) {
      // color 1280x480 yuyv, only 15 ok

      color_res_index_ =
          GetStreamIndex(stream_color_info_ptr_, 1280, 480, false);
      framerate_ = 15;
      open_params_.framerate = 15;
    } else if (params.stream_mode == StreamMode::STREAM_1280x480) {
    } else if (params.stream_mode == StreamMode::STREAM_640x480) {
      // color 640x480 yuyv, only 15 ok

      color_res_index_ =
          GetStreamIndex(stream_color_info_ptr_, 640, 480, false);
      framerate_ = 15;
      open_params_.framerate = 15;
    } else {
      goto usb2_error;
    }
    if (color_res_index_ == -1) {
      goto usb2_error;
    }
  } else if (params.dev_mode == DeviceMode::DEVICE_DEPTH) {
  // depth only

    if (params.stream_mode == StreamMode::STREAM_2560x720 ||
        params.stream_mode == StreamMode::STREAM_1280x720) {
      // depth 640x720 yuyv, 8 bits only 5 ok
      depth_res_index_ =
          GetStreamIndex(stream_depth_info_ptr_, 640, 720, false);
      framerate_ = 5;
      open_params_.framerate = 5;
    } else if (params.stream_mode == StreamMode::STREAM_1280x480 ||
               params.stream_mode == StreamMode::STREAM_640x480) {
      // depth 320x480 yuyv, 8 bits only 15 ok
      depth_res_index_ =
          GetStreamIndex(stream_depth_info_ptr_, 320, 480, false);
      framerate_ = 15;
      open_params_.framerate = 15;
    } else if (params.stream_mode == StreamMode::STREAM_1280x480 ||
      params.stream_mode == StreamMode::STREAM_640x480) {
      // depth 320x480 yuyv, 8 bits only 15 ok
      depth_res_index_ =
        GetStreamIndex(stream_depth_info_ptr_, 320, 480, false);
      framerate_ = 15;
      open_params_.framerate = 15;
    } else {
      goto usb2_error;
    }
    if (depth_res_index_ == -1) {
      goto usb2_error;
    }
  } else {
    goto usb2_error;
  }

  LOGI("\nWARNING:: You are using the USB 2.0 interface. "
      "For bandwidth reasons, "
      "it is recommended to use the USB 3.0 interface.\n");
  return;

usb2_error:
  throw_error("\nNote:: You are using the USB 2.0 interface."
      " Current resolution or frame rate is not supported"
      " And you can refer to Resolution Support List"
      " in the documentation.\n");
}

void Device::CompatibleMJPG(const OpenParams& params) {
  if (!stream_color_info_ptr_[color_res_index_].bFormatMJPG)
    return;

  // using 8 bits, if mjpg
  switch (params.color_mode) {
    case ColorMode::COLOR_RECTIFIED:
      depth_data_type_ = ETronDI_DEPTH_DATA_8_BITS;
      break;
    case ColorMode::COLOR_RAW:
    default:
      depth_data_type_ = ETronDI_DEPTH_DATA_8_BITS_RAW;
      break;
  }

  switch (params.dev_mode) {
    case DeviceMode::DEVICE_ALL:
      goto mjpg_error;
      break;
    case DeviceMode::DEVICE_COLOR:
      if (params.stream_mode == StreamMode::STREAM_2560x720) {
        // color 2560x720 mjpg only 5
        framerate_ = 5;
        open_params_.framerate = 5;
        break;
      } else if (params.stream_mode == StreamMode::STREAM_1280x480) {
        // color 1280x480 mjpg only 15
        framerate_ = 15;
        open_params_.framerate = 15;
        break;
      } else if (params.stream_mode == StreamMode::STREAM_1280x720) {
        // color 1280x720 mjpg only 5
        framerate_ = 5;
        open_params_.framerate = 5;
        break;
      }
      break;
    case DeviceMode::DEVICE_DEPTH:
      goto mjpg_error;
      break;
    default:
      throw_error("Unknown DeviceMode.");
  }

  return;

mjpg_error:
  throw_error("\nNote:: You are using the mjpg mode."
      " Current resolution or frame rate is not supported"
      " And you can refer to Resolution Support List"
      " in the documentation.\n");
}

bool Device::IsUSB2() {
  if (stream_depth_info_ptr_ == nullptr) {
    return false;
  }
  return stream_depth_info_ptr_[1].nWidth == 320;
}

int Device::GetStreamIndex(PETRONDI_STREAM_INFO stream_info_ptr,
    int width, int height, bool mjpg) {

  PETRONDI_STREAM_INFO stream_temp_info_ptr = stream_info_ptr;
  int res_index = -1;
  int i = 0;
  while (i < 64) {
    if (stream_temp_info_ptr->nWidth == width &&
        stream_temp_info_ptr->nHeight == height &&
        stream_temp_info_ptr->bFormatMJPG == mjpg) {
      res_index = i;
      break;
    }
    stream_temp_info_ptr++;
    i++;
  }

  return res_index;
}

bool Device::SetSensorType(const SensorType &type) {
  if (!IsOpened()) {
    LOGE("\nERROR:: Device is not opened.\n");
    return false;
  }
  int sensor_type = get_sensor_type(type);

#ifdef MYNTEYE_OS_WIN
  if (EtronDI_SetSensorTypeName(
      handle_, (SENSOR_TYPE_NAME)sensor_type) == ETronDI_OK) {
    return true;
  } else {
    return false;
  }
#else
  if (EtronDI_SetSensorTypeName(
      handle_, &dev_sel_info_,
      (SENSOR_TYPE_NAME)sensor_type) == ETronDI_OK) {
    return true;
  } else {
    return false;
  }
#endif
}

bool Device::SetExposureTime(const float &value) {
  if (!IsOpened()) {
    LOGE("\nERROR:: Device is not opened.\n");
    return false;
  }
  if (!SetSensorType(SensorType::SENSOR_TYPE_AR0135))
    return false;

  int sensor_mode = get_sensor_mode(SensorMode::ALL);
  if (EtronDI_SetExposureTime(
        handle_, &dev_sel_info_,
        sensor_mode, value) == ETronDI_OK) {
    params_member_[ControlParams::EXPOSURE_TIME].fvalue = value;
    return true;
  } else {
    return false;
  }
}

bool Device::GetExposureTime(float &value) {
  if (!IsOpened()) {
    LOGE("\nERROR:: Device is not opened.\n");
    return false;
  }
  if (!SetSensorType(SensorType::SENSOR_TYPE_AR0135))
    return false;

  int sensor_mode = get_sensor_mode(SensorMode::ALL);
  if (EtronDI_GetExposureTime(
        handle_, &dev_sel_info_,
        sensor_mode, &value) == ETronDI_OK) {
    return true;
  } else {
    return false;
  }
}

bool Device::SetGlobalGain(const float &value) {
  if (!IsOpened()) {
    LOGE("\nERROR:: Device is not opened.\n");
    return false;
  }
  if (!SetSensorType(SensorType::SENSOR_TYPE_AR0135))
    return false;

  int sensor_mode = get_sensor_mode(SensorMode::ALL);
  if (EtronDI_SetGlobalGain(
        handle_, &dev_sel_info_,
        sensor_mode, value) == ETronDI_OK) {
    params_member_[ControlParams::GLOBAL_GAIN].fvalue = value;
    return true;
  } else {
    return false;
  }
}

bool Device::GetGlobalGain(float &value) {
  if (!IsOpened()) {
    LOGE("\nERROR:: Device is not opened.\n");
    return false;
  }
  if (!SetSensorType(SensorType::SENSOR_TYPE_AR0135))
    return false;

  int sensor_mode = get_sensor_mode(SensorMode::ALL);
  if (EtronDI_GetGlobalGain(
        handle_, &dev_sel_info_,
        sensor_mode, &value) == ETronDI_OK) {
    return true;
  } else {
    return false;
  }
}

void Device::SetSerialNumber(const std::string &sn) {
  unsigned char serial_n[48];

  for (int i = 0; i <= 23; i++) {
    serial_n[i * 2] = static_cast<unsigned char>(sn[i]);
    serial_n[i * 2 + 1] = 0x00;
  }
  EtronDI_SetSerialNumber(handle_, &dev_sel_info_, serial_n, 48);
}

std::string Device::GetSerialNumber() const {
  unsigned char serial_n[512];
  int len;
  EtronDI_GetSerialNumber(handle_, (PDEVSELINFO)&dev_sel_info_, serial_n, 512, &len);

  char tmp[25];
  memset(tmp, '\0', sizeof(tmp));
  for (int i = 0; i < len / 2; i++) {
    tmp[i] = serial_n[i * 2];
  }
  std::string s = tmp;
  return s;
}

float Device::GetSensorTemperature() {
  static bool inited = false;
  static std::uint16_t tempsens_calib = 0;
  if (!inited) {
    if (SetSensorRegister(0x30, 0x30b4, (1 << 0) | (1 << 4),
        FG_Address_2Byte | FG_Value_2Byte)) {
      if (GetSensorRegister(0x30, 0x30c8, &tempsens_calib,
          FG_Address_2Byte | FG_Value_2Byte)) {
        tempsens_calib = ReverseBytes(tempsens_calib);
      } else {
        LOGW("SensorRegister: tempsens calib read failed");
      }
      inited = true;
      return 0;  // temp is 0 first time, get it next time
    } else {
      LOGW("SensorRegister: tempsens power on failed");
      return 0;
    }
  }
  /*
  std::uint16_t data = 0;
  if (GetSensorRegister(0x30, 0x3000, &data,
      FG_Address_2Byte | FG_Value_2Byte)) {
    data = ReverseBytes(data);
    std::cout << std::hex << "chip_version_reg: 0x" <<  data << std::endl;
  }
  */
  std::uint16_t tempsens_data = 0;
  if (GetSensorRegister(0x30, 0x30b2, &tempsens_data,
      FG_Address_2Byte | FG_Value_2Byte)) {
    tempsens_data = ReverseBytes(tempsens_data) & 0x7ff;  // [10:0]
  } else {
    LOGW("SensorRegister: tempsens data read failed");
    inited = false;  // reinit if read temp failed
    return 0;
  }
  // std::cout << std::hex << "data: 0x" << tempsens_data
  //     << ", calib: 0x" << tempsens_calib << std::endl;
  // LOGI("GetSensorTemperature data: %d, calib: %d",
  //     tempsens_data, tempsens_calib);
  return 55 + 0.7 * (tempsens_data - tempsens_calib);
}

std::shared_ptr<ColorizerPrivate> Device::GetColorizer() const {
  return colorizer_;
}

bool Device::IsIRDepthOnly() {
  return ir_depth_only_enabled_;
}

bool Device::UpdateStreamInfos() {
  memset(stream_color_info_ptr_, 0, sizeof(ETRONDI_STREAM_INFO) * MAX_STREAM_COUNT);
  memset(stream_depth_info_ptr_, 0, sizeof(ETRONDI_STREAM_INFO) * MAX_STREAM_COUNT);

  DEVSELINFO dev_sel_info{open_params_.dev_index};
  EtronDI_GetDeviceResolutionList(handle_, &dev_sel_info, MAX_STREAM_COUNT,
      stream_color_info_ptr_, MAX_STREAM_COUNT, stream_depth_info_ptr_);

  return true;
}

void Device::ResumeParams() {
  auto &&it = params_member_.find(ControlParams::AUTO_EXPOSURE);
  if (it != params_member_.end()) {
    SetAutoExposureEnabled(it->second.enabled);
  }

  it = params_member_.find(ControlParams::AUTO_WHITE_BALANCE);
  if (it != params_member_.end()) {
    SetAutoWhiteBalanceEnabled(it->second.enabled);
  }

  it =  params_member_.find(ControlParams::IR_DEPTH_ONLY);
  if (it != params_member_.end()) {
    SetInfraredDepthOnly(open_params_);
  }

  it = params_member_.find(ControlParams::IR_INTENSITY);
  if (it != params_member_.end()) {
    SetInfraredIntensity(it->second.value);
  }

  it = params_member_.find(ControlParams::GLOBAL_GAIN);
  if (it != params_member_.end()) {
    SetGlobalGain(it->second.fvalue);
  }

  it = params_member_.find(ControlParams::EXPOSURE_TIME);
  if (it != params_member_.end()) {
    SetExposureTime(it->second.fvalue);
  }

  it = params_member_.find(ControlParams::HW_REGISTER);
  if (it != params_member_.end()) {
    SetHWRegister(it->second.address, it->second.value, it->second.flag);
  }

  it = params_member_.find(ControlParams::FW_REGISTER);
  if (it != params_member_.end()) {
    SetFWRegister(it->second.address, it->second.value, it->second.flag);
  }
}

bool Device::IsInitDevice() {
  return handle_ && dev_sel_info_.index != -1;
}

bool Device::UpdateDeviceStatus() {
  if ((!device_status_[COLOR_DEVICE] && is_actual_[COLOR_DEVICE]) ||
        (!device_status_[DEPTH_DEVICE] && is_actual_[DEPTH_DEVICE])) {

    if (check_times_ > 0) {
      --check_times_;
      return true;
    } else {
      check_times_ = MAX_CHECK_TIMES;
      return false;
    }
  }

  device_status_[COLOR_DEVICE] = false;
  device_status_[DEPTH_DEVICE] = false;
  check_times_ = MAX_CHECK_TIMES;

  return true;
}

bool Device::DepthDeviceOpened() {
  return depth_device_opened_;
}

std::uint16_t Device::ReverseBytes(const std::uint16_t &value) {
  return ((value >> 8) & 0xff) | ((value << 8) & 0xff00);
}
