/*! \file eSPDI_ErrCode.h
  	\brief definition of Etron SDK error code
  	Copyright:
	This file copyright (C) 2017 by

	eYs3D an Etron company

	An unpublished work.  All rights reserved.

	This file is proprietary information, and may not be disclosed or
	copied without the prior permission of eYs3D an Etron company.
 */
 
//define Error Code by Wolf 2013/08/30
#define  ETronDI_OK                             0
#define  ETronDI_NoDevice                      -1
#define  ETronDI_NullPtr                       -2
#define  ETronDI_ErrBufLen                     -3
#define  ETronDI_Init_Fail                     -4
#define  ETronDI_NoZDTable                     -5 
#define  ETronDI_READFLASHFAIL                 -6
#define  ETronDI_WRITEFLASHFAIL                -7 
#define  ETronDI_VERIFY_DATA_FAIL              -8
#define  ETronDI_KEEP_DATA_FAIL                -9
#define  ETronDI_RECT_DATA_LEN_FAIL           -10
#define  ETronDI_RECT_DATA_PARSING_FAIL       -11
#define  ETronDI_RET_BAD_PARAM                -12
#define  ETronDI_RET_OPEN_FILE_FAIL           -13
#define  ETronDI_NO_CALIBRATION_LOG           -14
#define  ETronDI_POSTPROCESS_INIT_FAIL        -15
#define  ETronDI_POSTPROCESS_NOT_INIT         -16
#define  ETronDI_POSTPROCESS_FRAME_FAIL       -17
#define  ETronDI_NotSupport                   -18
#define  ETronDI_OpenFileFail                 -19
#define  ETronDI_READ_REG_FAIL                -20
#define  ETronDI_WRITE_REG_FAIL               -21
#define  ETronDI_SET_FPS_FAIL                 -22
#define  ETronDI_VIDEO_RENDER_FAIL            -23
#define  ETronDI_OPEN_DEVICE_FAIL             -24
#define  ETronDI_FIND_DEVICE_FAIL             -25
#define  ETronDI_GET_IMAGE_FAIL               -26
#define  ETronDI_NOT_SUPPORT_RES              -27
#define  ETronDI_CALLBACK_REGISTER_FAIL       -28
#define  ETronDI_DEVICE_NOT_SUPPORT			  -29

// for 3D Scanner +    
#define  ETronDI_ILLEGAL_ANGLE                -30
#define  ETronDI_ILLEGAL_STEP                 -31
#define  ETronDI_ILLEGAL_TIMEPERSTEP          -32
#define  ETronDI_MOTOR_RUNNING                -33 
#define  ETronDI_GETSENSORREG_FAIL            -34
#define  ETronDI_SETSENSORREG_FAIL            -35
#define  ETronDI_READ_X_AXIS_FAIL             -36
#define  ETronDI_READ_Y_AXIS_FAIL             -37
#define  ETronDI_READ_Z_AXIS_FAIL             -38
#define  ETronDI_READ_PRESS_DATA_FAIL         -39
#define  ETronDI_READ_TEMPERATURE_FAIL        -40
#define  ETronDI_RETURNHOME_RUNNING           -41
#define  ETronDI_MOTOTSTOP_BY_HOME_INDEX      -42
#define  ETronDI_MOTOTSTOP_BY_PROTECT_SCHEME  -43
#define  ETronDI_MOTOTSTOP_BY_NORMAL          -44
#define  ETronDI_ILLEGAL_FIRMWARE_VERSION     -45
#define  ETronDI_ILLEGAL_STEPPERTIME          -46
// for 3D Scanner - 

// For AEAWB + 2015/01/28 by Wolf
#define  ETronDI_GET_PU_PROP_VAL              -50
#define  ETronDI_SET_PU_PROP_VAL              -51
#define  ETronDI_GET_CT_PROP_VAL              -52
#define  ETronDI_SET_CT_PROP_VAL              -53
// For AEAWB - 2015/01/28 by Wolf 

// for Dewarping + Stitching +
#define  ETronDI_INVALID_USERDATA             -70
#define  ETronDI_MAP_LUT_FAIL                 -71
#define  ETronDI_APPEND_TO_FILE_FRONT_FAIL    -72
// for Dewarping + Stitching -

#define ETronDI_TOO_MANY_DEVICE               -80
#define ETronDI_ACCESS_MP4_EXTRA_DATA_FAIL    -81