/*! \file eSPDI_def.h
  	\brief main data structure, variable and macro definition and error definition
  	Copyright:
	This file copyright (C) 2017 by

	eYs3D an Etron company

	An unpublished work.  All rights reserved.

	This file is proprietary information, and may not be disclosed or
	copied without the prior permission of eYs3D an Etron company.
 */
#ifndef LIB_ETRONDI_DEF_H
#define LIB_ETRONDI_DEF_H

//define Error Code by Wolf 2015/08/07 +
#define  ETronDI_OK                        0
#define  ETronDI_NoDevice                 -1
#define  ETronDI_NullPtr                  -2
#define  ETronDI_ErrBufLen                -3
#define  ETronDI_Init_Fail                -4
#define  ETronDI_NoZDTable                -5 
#define  ETronDI_READFLASHFAIL            -6
#define  ETronDI_WRITEFLASHFAIL           -7 
#define  ETronDI_VERIFY_DATA_FAIL         -8
#define  ETronDI_KEEP_DATA_FAIL           -9
#define  ETronDI_RECT_DATA_LEN_FAIL      -10
#define  ETronDI_RECT_DATA_PARSING_FAIL  -11
#define  ETronDI_RET_BAD_PARAM           -12
#define  ETronDI_RET_OPEN_FILE_FAIL      -13
#define  ETronDI_NO_CALIBRATION_LOG      -14
#define  ETronDI_POSTPROCESS_INIT_FAIL   -15
#define  ETronDI_POSTPROCESS_NOT_INIT    -16
#define  ETronDI_POSTPROCESS_FRAME_FAIL  -17
#define  ETronDI_NotSupport              -18
#define  ETronDI_GET_RES_LIST_FAIL       -19
#define  ETronDI_READ_REG_FAIL           -20
#define  ETronDI_WRITE_REG_FAIL          -21
#define  ETronDI_SET_FPS_FAIL            -22
#define  ETronDI_VIDEO_RENDER_FAIL       -23
#define  ETronDI_OPEN_DEVICE_FAIL        -24
#define  ETronDI_FIND_DEVICE_FAIL        -25
#define  ETronDI_GET_IMAGE_FAIL          -26
#define  ETronDI_NOT_SUPPORT_RES         -27
#define  ETronDI_CALLBACK_REGISTER_FAIL  -28
#define  ETronDI_CLOSE_DEVICE_FAIL	 	 -29
#define  ETronDI_GET_CALIBRATIONLOG_FAIL -30
#define  ETronDI_SET_CALIBRATIONLOG_FAIL -31
#define  ETronDI_DEVICE_NOT_SUPPORT	     -32
#define  ETronDI_DEVICE_BUSY		     -33

// for 3D Scanner +    
#define  ETronDI_ILLEGAL_ANGLE                -40
#define  ETronDI_ILLEGAL_STEP                 -41
#define  ETronDI_ILLEGAL_TIMEPERSTEP          -42
#define  ETronDI_MOTOR_RUNNING                -43 
#define  ETronDI_GETSENSORREG_FAIL            -44
#define  ETronDI_SETSENSORREG_FAIL            -45
#define  ETronDI_READ_X_AXIS_FAIL             -46
#define  ETronDI_READ_Y_AXIS_FAIL             -47
#define  ETronDI_READ_Z_AXIS_FAIL             -48
#define  ETronDI_READ_PRESS_DATA_FAIL         -49
#define  ETronDI_READ_TEMPERATURE_FAIL        -50
#define  ETronDI_RETURNHOME_RUNNING           -51
#define  ETronDI_MOTOTSTOP_BY_HOME_INDEX      -52
#define  ETronDI_MOTOTSTOP_BY_PROTECT_SCHEME  -53
#define  ETronDI_MOTOTSTOP_BY_NORMAL          -54
#define  ETronDI_ILLEGAL_FIRMWARE_VERSION     -55
#define  ETronDI_ILLEGAL_STEPPERTIME          -56
// for 3D Scanner - 

// For AEAWB + 
#define  ETronDI_GET_PU_PROP_VAL_FAIL    	  -60
#define  ETronDI_SET_PU_PROP_VAL_FAIL         -61
#define  ETronDI_GET_CT_PROP_VAL_FAIL         -62
#define  ETronDI_SET_CT_PROP_VAL_FAIL    	  -63
#define  ETronDI_GET_CT_PROP_RANGE_STEP_FAIL  -64
#define  ETronDI_GET_PU_PROP_RANGE_STEP_FAIL  -65
// For AEAWB - 

// for Dewarping + Stitching +
#define  ETronDI_INVALID_USERDATA             -70
#define  ETronDI_MAP_LUT_FAIL                 -71
#define  ETronDI_APPEND_TO_FILE_FRONT_FAIL    -72
// for Dewarping + Stitching -

#define ETronDI_TOO_MANY_DEVICE               -80
#define ETronDI_ACCESS_MP4_EXTRA_DATA_FAIL    -81

//define Error Code by Wolf 2015/08/07 +

// define windows type +
#ifndef BYTE
typedef unsigned char BYTE;
#endif //BYTE

#ifndef BOOL
typedef signed int BOOL;
#endif //BOOL

#ifndef WORD
typedef unsigned short WORD;
#endif //WORD
// define windows type -

#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1<<(b)))
#define BIT_CHECK(a,b) ((a) & (1<<(b)))

#define FG_Address_1Byte 0x01
#define FG_Address_2Byte 0x02
#define FG_Value_1Byte   0x10
#define FG_Value_2Byte   0x20

// for Sensor mode +
typedef enum
{
  SENSOR_A = 0,
  SENSOR_B,
  SENSOR_BOTH
} SENSORMODE_INFO;
// for Sensor mode +

// register address define +
#define CHIPID_ADDR         0xf014
#define SERIAL_2BIT_ADDR    0xf0fe
// register address define -

// For Depth Data Type
#define ETronDI_DEPTH_DATA_OFF_RAW			0 /* raw (depth off, only raw color) */
#define ETronDI_DEPTH_DATA_DEFAULT			0 /* raw (depth off, only raw color) */
#define ETronDI_DEPTH_DATA_8_BITS				1 /* rectify, 1 byte per pixel */
#define ETronDI_DEPTH_DATA_14_BITS				2 /* rectify, 2 byte per pixel */
#define ETronDI_DEPTH_DATA_8_BITS_x80			3 /* rectify, 2 byte per pixel but using 1 byte only */
#define ETronDI_DEPTH_DATA_11_BITS				4 /* rectify, 2 byte per pixel but using 11 bit only */
#define ETronDI_DEPTH_DATA_OFF_RECTIFY		5 /* rectify (depth off, only rectify color) */
#define ETronDI_DEPTH_DATA_8_BITS_RAW			6 /* raw */
#define ETronDI_DEPTH_DATA_14_BITS_RAW		7 /* raw */
#define ETronDI_DEPTH_DATA_8_BITS_x80_RAW	8 /* raw */
#define ETronDI_DEPTH_DATA_11_BITS_RAW		9 /* raw */
#define ETronDI_DEPTH_DATA_11_BITS_COMBINED_RECTIFY     13// multi-baseline

// for Flash Read/Write +
// Firmware (size in KBytes)
#define ETronDI_READ_FLASH_TOTAL_SIZE			128
#define ETronDI_READ_FLASH_FW_PLUGIN_SIZE		104
#define ETronDI_WRITE_FLASH_TOTAL_SIZE			128

// PlugIn data (size in bytes)
#define ETronDI_Y_OFFSET_FILE_ID_0				30
#define ETronDI_Y_OFFSET_FILE_SIZE			    256
#define ETronDI_RECTIFY_FILE_ID_0				40
#define ETronDI_RECTIFY_FILE_SIZE				1024
#define ETronDI_ZD_TABLE_FILE_ID_0				50
#define ETronDI_ZD_TABLE_FILE_SIZE_8_BITS		512
#define ETronDI_ZD_TABLE_FILE_SIZE_11_BITS		4096
#define ETronDI_CALIB_LOG_FILE_ID_0				240
#define ETronDI_CALIB_LOG_FILE_SIZE				4096
#define ETronDI_USER_DATA_FILE_ID_0				200
#define ETronDI_USER_DATA_FILE_SIZE_0			1024
#define ETronDI_USER_DATA_FILE_SIZE_1			4096
// for Flash Read/Write -

// for device information +
typedef struct tagDEVINFORMATION {
  unsigned short wPID;
  unsigned short wVID;
  char *strDevName;
  unsigned short  nChipID;
  unsigned short  nDevType;
} DEVINFORMATION, *PDEVINFORMATION;
// for device information -

// for device selection information +
typedef struct tagDEVSEL
{
  int index;
} DEVSELINFO, *PDEVSELINFO;
// for device selection information -

// for output stream info +
typedef struct tagETRONDI_STREAM_INFO {
	int		nWidth;
	int		nHeight;
	BOOL	bFormatMJPG;
} ETRONDI_STREAM_INFO, *PETRONDI_STREAM_INFO;
// for output stream info -

// for ZD table info +
typedef struct tagZDTableInfo
{
  int nIndex;
  int nDataType;
} ZDTABLEINFO, *PZDTABLEINFO;
// for ZD table info -

// for device type +
typedef enum {
  OTHERS = 0,
  AXES1,
  PUMA,
  KIWI
}DEVICE_TYPE;
// for device type -

// for total and fw+plugin read/write +
typedef enum {	
    Total = 0,
    FW_PLUGIN,
    BOOTLOADER_ONLY,
    FW_ONLY,
    PLUGIN_ONLY
} FLASH_DATA_TYPE;
// for total and fw+plugin read/write -

// for user data +
typedef enum
{
  USERDATA_SECTION_0 = 0,
  USERDATA_SECTION_1,
  USERDATA_SECTION_2,
  USERDATA_SECTION_3,
  USERDATA_SECTION_4,
  USERDATA_SECTION_5,
  USERDATA_SECTION_6,
  USERDATA_SECTION_7,
  USERDATA_SECTION_8,
  USERDATA_SECTION_9
} USERDATA_SECTION_INDEX;
// for user data -

// for Calibration Log Type +
typedef enum {	
	ALL_LOG = 0,
    SERIAL_NUMBER,
    PRJFILE_LOG,
    STAGE_TIME_RESULT_LOG,
    SENSOR_OFFSET,
    AUTO_ADJUST_LOG,
    RECTIFY_LOG,
    ZD_LOG,
    DEPTHMAP_KOG
} CALIBRATION_LOG_TYPE;
// for Calibration Log Type -

typedef struct tagKEEP_DATA_CTRL {	
	bool  bIsSerialNumberKeep;
	bool  bIsSensorPositionKeep;
	bool  bIsRectificationTableKeep;
	bool  bIsZDTableKeep;
	bool  bIsCalibrationLogKeep;
} KEEP_DATA_CTRL;

typedef enum{
	IMAGE_SN_NONSYNC = 0,
	IMAGE_SN_SYNC
} CONTROL_MODE;

typedef enum{
	DEPTH_IMG_NON_TRANSFER,
	DEPTH_IMG_GRAY_TRANSFER,
	DEPTH_IMG_COLORFUL_TRANSFER
}DEPTH_TRANSFER_CTRL;


// for Sensor type name +
typedef enum {
    ETRONDI_SENSOR_TYPE_H22 = 0,
    ETRONDI_SENSOR_TYPE_OV7740,
    ETRONDI_SENSOR_TYPE_AR0134,
    ETRONDI_SENSOR_TYPE_AR0135,
    ETRONDI_SENSOR_TYPE_AR0330,
    ETRONDI_SENSOR_TYPE_OV9714,
    ETRONDI_SENSOR_TYPE_OV9282
} SENSOR_TYPE_NAME; 
// for Sensor type name -

typedef enum
{
  AE_ENABLE = 0,
  AE_DISABLE
} AE_STATUS, *PAE_STATUS;

typedef enum
{
  AWB_ENABLE = 0,
  AWB_DISABLE
} AWB_STATUS, *PAWB_STATUS;

//
// CT Property ID
//
#define	CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL       0
#define CT_PROPERTY_ID_AUTO_EXPOSURE_PRIORITY_CTRL   1
#define CT_PROPERTY_ID_EXPOSURE_TIME_ABSOLUTE_CTRL   2
#define CT_PROPERTY_ID_EXPOSURE_TIME_RELATIVE_CTRL   3
#define CT_PROPERTY_ID_FOCUS_ABSOLUTE_CTRL           4
#define CT_PROPERTY_ID_FOCUS_RELATIVE_CTRL           5
#define CT_PROPERTY_ID_FOCUS_AUTO_CTRL          	 6
#define CT_PROPERTY_ID_IRIS_ABSOLUTE_CTRL            7
#define CT_PROPERTY_ID_IRIS_RELATIVE_CTRL            8
#define CT_PROPERTY_ID_ZOOM_ABSOLUTE_CTRL            9
#define CT_PROPERTY_ID_ZOOM_RELATIVE_CTRL           10
#define CT_PROPERTY_ID_PAN_ABSOLUTE_CTRL            11
#define CT_PROPERTY_ID_PAN_RELATIVE_CTRL            12
#define CT_PROPERTY_ID_TILT_ABSOLUTE_CTRL           13
#define CT_PROPERTY_ID_TILT_RELATIVE_CTRL           14
#define CT_PROPERTY_ID_PRIVACY_CTRL                 15

//
// PU Property ID
//
#define PU_PROPERTY_ID_BACKLIGHT_COMPENSATION_CTRL   0
#define PU_PROPERTY_ID_BRIGHTNESS_CTRL      		 1
#define PU_PROPERTY_ID_CONTRAST_CTRL 			     2
#define PU_PROPERTY_ID_GAIN_CTRL 			     	 3
#define PU_PROPERTY_ID_POWER_LINE_FREQUENCY_CTRL 	 4
#define PU_PROPERTY_ID_HUE_CTRL 			         5
#define PU_PROPERTY_ID_HUE_AUTO_CTRL 			     6
#define PU_PROPERTY_ID_SATURATION_CTRL 			     7
#define PU_PROPERTY_ID_SHARPNESS_CTRL 			     8
#define PU_PROPERTY_ID_GAMMA_CTRL 			     	 9
#define PU_PROPERTY_ID_WHITE_BALANCE_CTRL 			10
#define PU_PROPERTY_ID_WHITE_BALANCE_AUTO_CTRL 	    11

// for Rectify Log +

typedef struct eSPCtrl_RectLogData {
	union {
		unsigned char uByteArray[1024];/**< union data defined as below struct { }*/
		struct {
			unsigned short	InImgWidth;/**< Input image width(SideBySide image) */
			unsigned short	InImgHeight;/**< Input image height */
			unsigned short	OutImgWidth;/**< Output image width(SideBySide image) */
			unsigned short	OutImgHeight;/**< Output image height */
			int		        RECT_ScaleEnable;/**< Rectified image scale */
			int		        RECT_CropEnable;/**< Rectified image crop */
			unsigned short	RECT_ScaleWidth;/**< Input image width(Single image) *RECT_Scale_Col_N /RECT_Scale_Col_M */
			unsigned short	RECT_ScaleHeight;/**< Input image height(Single image) *RECT_Scale_Row_N /RECT_Scale_Row_M */
			float	        CamMat1[9];/**< Left Camera Matrix
								fx, 0, cx, 0, fy, cy, 0, 0, 1 
								fx,fy : focus  ; cx,cy : principle point */
			float	        CamDist1[8];/**< Left Camera Distortion Matrix
								k1, k2, p1, p2, k3, k4, k5, k6 
								k1~k6 : radial distort ; p1,p2 : tangential distort */
			float			CamMat2[9];/**< Right Camera Matrix
								fx, 0, cx, 0, fy, cy, 0, 0, 1  
								fx,fy : focus  ; cx,cy : principle point */
			float			CamDist2[8];/**< Right Camera Distortion Matrix
								k1, k2, p1, p2, k3, k4, k5, k6 
								k1~k6 : radial distort ; p1,p2 : tangential distort */
			float			RotaMat[9];/**< Rotation matrix between the left and right camera coordinate systems. 
								| [0] [1] [2] |       |Xcr|
								| [3] [4] [5] |   *   |Ycr|            => cr = right camera coordinate
								| [6] [7] [8] |       |Zcr| */
			float			TranMat[3];/**< Translation vector between the coordinate systems of the cameras. 
								|[0]|      |Xcr|
								|[1]|   +  |Ycr|	             => cr = right camera coordinate
								|[2]|      |Zcr| */
			float			LRotaMat[9];/**< 3x3 rectification transform (rotation matrix) for the left camera. 
								| [0] [1] [2] |       |Xcl|
								| [3] [4] [5] |   *   |Ycl|            => cl = left camera coordinate
								| [6] [7] [8] |       |Zcl| */
			float			RRotaMat[9];/**< 3x3 rectification transform (rotation matrix) for the left camera.
								| [0] [1] [2] |       |Xcr|
								| [3] [4] [5] |   *   |Ycr|            => cr = right camera coordinate
								| [6] [7] [8] |       |Zcr| */
			float			NewCamMat1[12];/**< 3x4 projection matrix in the (rectified) coordinate systems for the left camera. 
								fx' 0 cx' 0 0 fy' cy' 0 0 0 1 0 
								fx',fy' : rectified focus ; cx', cy; : rectified principle point */
			float			NewCamMat2[12];/**< 3x4 projection matrix in the (rectified) coordinate systems for the rightt camera. 
								fx' 0 cx' TranMat[0]* 0 fy' cy' 0 0 0 1 0 
								fx',fy' : rectified focus ; cx', cy; : rectified principle point */
			unsigned short	RECT_Crop_Row_BG;/**< Rectidied image crop row begin */
			unsigned short	RECT_Crop_Row_ED;/**< Rectidied image crop row end */
			unsigned short	RECT_Crop_Col_BG_L;/**< Rectidied image crop column begin */
			unsigned short	RECT_Crop_Col_ED_L;/**< Rectidied image crop column end */
			unsigned char	RECT_Scale_Col_M;/**< Rectified image scale column factor M */
			unsigned char	RECT_Scale_Col_N;/**< Rectified image scale column factor N
								Rectified image scale column ratio =  Scale_Col_N/ Scale_Col_M */
			unsigned char	RECT_Scale_Row_M;/**< Rectified image scale row factor M */
			unsigned char	RECT_Scale_Row_N;/**< Rectified image scale row factor N */
			float			RECT_AvgErr;/**< Reprojection error */
			unsigned short	nLineBuffers;/**< Linebuffer for Hardware limitation < 60 */
            float ReProjectMat[16];
		};
	};
} eSPCtrl_RectLogData;

// for Rectify Log -

// for Post Process +

#define POSTPAR_HR_MODE 		5
#define POSTPAR_HR_CURVE_0 		6
#define POSTPAR_HR_CURVE_1 		7
#define POSTPAR_HR_CURVE_2 		8
#define POSTPAR_HR_CURVE_3 		9
#define POSTPAR_HR_CURVE_4 		10
#define POSTPAR_HR_CURVE_5 		11
#define POSTPAR_HR_CURVE_6 		12
#define POSTPAR_HR_CURVE_7 		13
#define POSTPAR_HR_CURVE_8 		14
#define POSTPAR_HF_MODE 		17
#define POSTPAR_DC_MODE 		20
#define POSTPAR_DC_CNT_THD 		21
#define POSTPAR_DC_GRAD_THD 	22
#define POSTPAR_SEG_MODE 		23
#define POSTPAR_SEG_THD_SUB 	24
#define POSTPAR_SEG_THD_SLP 	25
#define POSTPAR_SEG_THD_MAX 	26
#define POSTPAR_SEG_THD_MIN 	27
#define POSTPAR_SEG_FILL_MODE 	28
#define POSTPAR_HF2_MODE 		31
#define POSTPAR_GRAD_MODE 		34
#define POSTPAR_TEMP0_MODE 		37
#define POSTPAR_TEMP0_THD 		38
#define POSTPAR_TEMP1_MODE 		41
#define POSTPAR_TEMP1_LEVEL 	42
#define POSTPAR_TEMP1_THD 		43
#define POSTPAR_FC_MODE 		46
#define POSTPAR_FC_EDGE_THD 	47
#define POSTPAR_FC_AREA_THD 	48
#define POSTPAR_MF_MODE 		51
#define POSTPAR_ZM_MODE 		52
#define POSTPAR_RF_MODE 		53
#define POSTPAR_RF_LEVEL 		54

// for Post Process -

// for 3D Motor Control +

// for Gyro +

typedef enum
{
    DPS_245 = 0,
    DPS_500,
    DPS_2000
} SENSITIVITY_LEVEL_L3G;

typedef struct GyroTag
{
	short x; 		
	short y;
	short z;
} GYRO_ANGULAR_RATE_DATA;

// for Gyro -

// for accelerometer and magnetometer +

typedef struct AccelerationTag
{
	short x; 		
	short y;
	short z;
} ACCELERATION_DATA;

typedef struct CompassTag
{
	short x; 		
	short y;
	short z;
} COMPASS_DATA;

typedef enum
{
    _2G = 0,
    _4G,
    _6G,
    _8G,
    _16G
} SENSITIVITY_LEVEL_LSM;

// for accelerometer and magnetometer -

// for pressure +

typedef enum
{
    One_Shot = 0,
    _1_HZ_1_HZ,
    _7_HZ_1_HZ,
    _12_5_HZ_1HZ,
    _25_HZ_1_HZ,
    _7_HZ_7_HZ,
     _12_5_HZ_12_5_HZ,
    _25_HZ_25_HZ
} OUTPUT_DATA_RATE;

// for pressure -

// for LED and Laser +

typedef enum
{
    POWER_ON = 0,
    POWER_OFF
} POWER_STATE;

typedef enum
{
    LEVEL_0 = 0,
    LEVEL_1,
    LEVEL_2,
    LEVEL_3,
    LEVEL_4,
    LEVEL_5,
    LEVEL_6,
    LEVEL_7,
    LEVEL_8,
    LEVEL_9,
    LEVEL_10,
    LEVEL_11,
    LEVEL_12,
    LEVEL_13,
    LEVEL_14,
    LEVEL_15
} BRIGHTNESS_LEVEL;

// for LED and Laser -

// for 3D Motor Control -

// for Point Cloud
struct EtronDIImageType
{
    enum Value
    {
        IMAGE_UNKNOWN = -1,
        COLOR_YUY2 = 0,
        COLOR_RGB24,
        COLOR_MJPG,
        DEPTH_8BITS = 100,
        DEPTH_8BITS_0x80,
        DEPTH_11BITS,
        DEPTH_14BITS
    };

    static bool IsImageColor(EtronDIImageType::Value type)
    {
        return (type == COLOR_YUY2 || type == COLOR_RGB24 || type == COLOR_MJPG);
    }

    static bool IsImageDepth(EtronDIImageType::Value type)
    {
        return (type != IMAGE_UNKNOWN && !IsImageColor(type));
    }

    static EtronDIImageType::Value DepthDataTypeToDepthImageType(WORD dataType)
    {
        switch (dataType)
        {
        case ETronDI_DEPTH_DATA_8_BITS:
        case ETronDI_DEPTH_DATA_8_BITS_RAW:
            return EtronDIImageType::DEPTH_8BITS;
        case ETronDI_DEPTH_DATA_8_BITS_x80:
        case ETronDI_DEPTH_DATA_8_BITS_x80_RAW:
            return EtronDIImageType::DEPTH_8BITS_0x80;
        case ETronDI_DEPTH_DATA_11_BITS:
        case ETronDI_DEPTH_DATA_11_BITS_RAW:
        case ETronDI_DEPTH_DATA_11_BITS_COMBINED_RECTIFY:
            return EtronDIImageType::DEPTH_11BITS;
        case ETronDI_DEPTH_DATA_14_BITS:
        case ETronDI_DEPTH_DATA_14_BITS_RAW:
            return EtronDIImageType::DEPTH_14BITS;
        default: return EtronDIImageType::IMAGE_UNKNOWN;
        }
    }
};


#endif // LIB_ETRONDI_DEF_H
