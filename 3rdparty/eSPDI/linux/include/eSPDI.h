
#ifndef LIB_ESPDI_H
#define LIB_ESPDI_H

#include "eSPDI_def.h"

//#ifdef __cplusplus
//extern "C"
//{
//#endif 

int  EtronDI_Init(void **ppHandleEtronDI, bool bIsLogEnabled);
int  EtronDI_FindDevice(void *pHandleEtronDI);
void EtronDI_Release(void **ppHandleEtronDI);
int  EtronDI_RefreshDevice(void *pHandleEtronDI);

int  EtronDI_GetDeviceNumber(void *pHandleEtronDI);
int  EtronDI_GetDeviceInfo(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo ,DEVINFORMATION* pdevinfo);
int  EtronDI_SelectDevice(void *pHandleEtronDI, int dev_index);

// register APIs +
int  EtronDI_GetSensorRegister( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nId, unsigned short address, unsigned short *pValue, int flag, SENSORMODE_INFO SensorMode);
int  EtronDI_SetSensorRegister( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nId, unsigned short address, unsigned short nValue,  int flag, SENSORMODE_INFO SensorMode);
int  EtronDI_GetFWRegister( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short address, unsigned short *pValue, int flag);
int  EtronDI_SetFWRegister( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short address, unsigned short nValue,  int flag);
int  EtronDI_GetHWRegister( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short address, unsigned short *pValue, int flag);
int  EtronDI_SetHWRegister( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short address, unsigned short nValue,  int flag);
// register APIs -

// File ID +
int  EtronDI_GetFwVersion(    void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, char *pszFwVersion, int nBufferSize, int *pActualLength);
int  EtronDI_GetPidVid(       void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short *pPidBuf, unsigned short *pVidBuf );
int  EtronDI_SetPidVid(       void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short *pPidBuf, unsigned short *pVidBuf );
int  EtronDI_GetSerialNumber (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned char* pData, int nbufferSize, int *pLen);
int  EtronDI_SetSerialNumber (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned char* pData, int nLen);
int  EtronDI_GetYOffset      (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BYTE *buffer, int BufferLength, int *pActualLength, int index);
int  EtronDI_GetRectifyTable (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BYTE *buffer, int BufferLength, int *pActualLength, int index);
int  EtronDI_GetZDTable      (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BYTE *buffer, int BufferLength, int *pActualLength, PZDTABLEINFO pZDTableInfo);
int  EtronDI_GetLogData      (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BYTE *buffer, int BufferLength, int *pActualLength, int index, CALIBRATION_LOG_TYPE type);
int  EtronDI_GetUserData     (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BYTE *buffer, int BufferLength, USERDATA_SECTION_INDEX usi);

int  EtronDI_SetYOffset      (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BYTE *buffer, int BufferLength, int *pActualLength, int index);
int  EtronDI_SetRectifyTable (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BYTE *buffer, int BufferLength, int *pActualLength, int index);
int  EtronDI_SetZDTable      (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BYTE *buffer, int BufferLength, int *pActualLength, PZDTABLEINFO pZDTableInfo);
int  EtronDI_SetLogData      (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BYTE *buffer, int BufferLength, int *pActualLength, int index);
int  EtronDI_SetUserData     (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BYTE *buffer, int BufferLength, USERDATA_SECTION_INDEX usi);
int  EtronDI_ReadFlashData   (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, FLASH_DATA_TYPE fdt,  
							  BYTE *pBuffer, unsigned long int BufferLength, unsigned long int *pActualLength);
int  EtronDI_WriteFlashData  (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, FLASH_DATA_TYPE fdt, BYTE *pBuffer, 
							  unsigned long int BufferLength, bool bIsDataVerify, KEEP_DATA_CTRL kdc);							  
// File ID -

// image +
int  EtronDI_GetDeviceResolutionList( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, 
									  int nMaxCount, ETRONDI_STREAM_INFO *pStreamInfo0, 
									  int nMaxCount1, ETRONDI_STREAM_INFO *pStreamInfo1);
									  
int  EtronDI_OpenDevice( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, 
						 int nWidth, int nHeight, bool bImageL, bool bDepth,
						 void *phWndNotice=0, CONTROL_MODE cm=IMAGE_SN_NONSYNC);									  
                                     			
int  EtronDI_OpenDevice2( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, 
						  int nEP0Width, int nEP0Height, bool bEP0MJPG, 
						  int nEP1Width, int nEP1Height, 
						  DEPTH_TRANSFER_CTRL dtc=DEPTH_IMG_NON_TRANSFER,
						  bool bIsOutputRGB24=false, void *phWndNotice=0, 
						  int *pFPS=0, CONTROL_MODE cm=IMAGE_SN_NONSYNC);	

int  EtronDI_CloseDevice( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);

int  EtronDI_GetImage ( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo,
                        BYTE *pBuf, unsigned long int *pImageSize,
                        int *pSerial = 0, int nDepthDataType =0);
int  EtronDI_Get2Image (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo,
                        BYTE *pColorImgBuf, BYTE *pDepthImgBuf,
                        unsigned long int *pColorImageSize, unsigned long int *pDepthImageSize,
                        int *pSerial = 0, int *pSerial2 = 0, int nDepthDataType =0);
// image -			

// for AEAWB Control +

#if 0
int  EtronDI_GetAEStatus  (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, PAE_STATUS pAEStatus);
int  EtronDI_GetAWBStatus (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, PAWB_STATUS pAWBStatus);
#endif

int  EtronDI_SetSensorTypeName( void *pHandleEtronDI, SENSOR_TYPE_NAME stn);
int  EtronDI_GetExposureTime( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nSensorMode, float *pfExpTimeMS);
int  EtronDI_SetExposureTime( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nSensorMode, float fExpTimeMS);
int  EtronDI_GetGlobalGain( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nSensorMode, float *pfGlobalGain);
int  EtronDI_SetGlobalGain( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nSensorMode, float fGlobalGain);
int  EtronDI_GetColorGain( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nSensorMode, float *pfGainR, float *pfGainG, float *pfGainB);
int  EtronDI_SetColorGain( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nSensorMode, float fGainR, float fGainG, float fGainB);
int  EtronDI_GetAccMeterValue( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int *pX, int *pY, int *pZ);

int  EtronDI_EnableAE     (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);
int  EtronDI_DisableAE    (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);
int  EtronDI_EnableAWB    (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);
int  EtronDI_DisableAWB   (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);

int  EtronDI_GetGPIOValue( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nGPIOIndex, BYTE *pValue);
int  EtronDI_SetGPIOValue( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nGPIOIndex, BYTE nValue);

int  EtronDI_GetCTPropVal( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nId, long int *pValue);
int  EtronDI_SetCTPropVal( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nId, long int nValue);

int  EtronDI_GetPUPropVal( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nId, long int *pValue);
int  EtronDI_SetPUPropVal( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nId, long int nValue);

#if 0
int  EtronDI_GetCTRangeAndStep( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nId, long int *pMax, long int *pMin, long int *pStep, long int *pDefault, long int *pFlags);
int  EtronDI_GetPURangeAndStep( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nId, long int *pMax, long int *pMin, long int *pStep, long int *pDefault, long int *pFlags);
#endif

// for AEAWB Control -		

// for depth data type selection +
int EtronDI_SetDepthDataType(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short nValue);
int EtronDI_GetDepthDataType(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short *pValue);
// for depth data type selection -

// IR support
int EtronDI_SetCurrentIRValue(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short nValue);
int EtronDI_GetCurrentIRValue(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short *pValue);
int EtronDI_GetIRMinValue(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short *pValue);
int EtronDI_GetIRMaxValue(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short *pValue);
int EtronDI_SetIRMode(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short nValue);
int EtronDI_GetIRMode(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short *pValue);
// ~IR support

// for Calibration Log +
int  EtronDI_GetRectifyLogData(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, eSPCtrl_RectLogData *pData);
// for Calibration Log -

// for Post Process +
int  EtronDI_EnablePostProcess( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, bool bEnable);
int  EtronDI_PostInitial( void *pHandleEtronDI);
int  EtronDI_PostEnd( void *pHandleEtronDI);
int  EtronDI_ProcessFrame( void *pHandleEtronDI, unsigned char *pYUY2Buf, unsigned char *pDepthBuf, unsigned char *OutputBuf, int width, int height);
int  EtronDI_PostSetParam( void *pHandleEtronDI, int Idx, int Val);	
int  EtronDI_PostGetParam( void *pHandleEtronDI, int Idx, int *pVal);	
// for Post Process -

// for sensorif +
int  EtronDI_EnableSensorIF( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, bool bIsEnable);
// for sensorif -

// for 3D Motor Control +

int  EtronDI_MotorInit( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short nMinSpeedPerStep); 

// motor_id :
//     0 : master
//     1 : slave

// direction :
//     true  : clockwise
//     false : anti-clockwise

// bIsStep :
//     true  : step
//     false : angle

// bIsTime :
//     true  : time per step
//     false : step per second

// bIsInfinnity :
//     true  : Infinity enable
//     false : Infinity disable
int  EtronDI_MotorStart ( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int motor_id, bool direction, double angle, long step, int timeperstep, float *steppersecond, bool bIsStep, bool bIsTime, bool bIsInfinity); 
int  EtronDI_MotorStart_IndexCheckOption( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int motor_id, bool direction, double angle, long step, int timeperstep, float *steppersecond, bool bIsStep, bool bIsTime, bool bIsInfinity, bool bIsIgnoreHomeIndexTouched); 
 
int  EtronDI_MotorStop( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);
int  EtronDI_GetMotorCurrentState( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, bool* bIsRunning);
int  EtronDI_GetMotorCurrentState_IndexCheckOption( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, bool* bIsRunning, long* nRemainStepNum, int *nHomeIndexNum);
int  EtronDI_GetMotorAngle( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, double *angle);
int  EtronDI_GetMotorStep( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, long *step);    
int  EtronDI_GetMotorTimePerStep( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short *nTimePerStep);
int  EtronDI_GetMotorStepPerSecond( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, float *fpStepPerSecond);
int  EtronDI_GetMotorDirection( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, bool *direction);
int  EtronDI_GetCurrentMotor( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int *motor_id); 

// for Gyro +
// bPowerMode :
//     true  : Normal
//     false : Power Down
// bIsZEnable :
//     true  : Enable
//     false : Disable
// bIsYEnable :
//     true  : Enable
//     false : Disable
// bIsXEnable :
//     true  : Enable
//     false : Disable
int  EtronDI_GyroInit( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, 
                       SENSITIVITY_LEVEL_L3G sll3g, 
					   bool bPowerMode, 
					   bool bIsZEnable, 
					   bool bIsYEnable, 
					   bool bIsXEnable);
					   
int  EtronDI_ReadGyro( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, GYRO_ANGULAR_RATE_DATA *gard);
// for Gyro -


// for accelerometer and magnetometer +
int  EtronDI_AccelInit( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, SENSITIVITY_LEVEL_LSM sllsm);
//DLLAPI int  ESPAPI Compass_Init( unsigned short nSensitivity_Level);
int  EtronDI_ReadAccel( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, ACCELERATION_DATA *ad);
int  EtronDI_ReadCompass( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, COMPASS_DATA *cd);
// for accelerometer and magnetometer -


// for pressure +
int  EtronDI_PsInit( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, OUTPUT_DATA_RATE odr);
int  EtronDI_ReadPressure( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int *nPressure);
int  EtronDI_ReadTemperature( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, short *nTemperature);
// for pressure -


// for LED and Laser +
int  EtronDI_SetLaserPowerState( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, POWER_STATE ps);
int  EtronDI_SetDesktopLEDPowerState( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, POWER_STATE ps);
int  EtronDI_GetLaserPowerState( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, POWER_STATE *ps);
int  EtronDI_GetDesktopLEDPowerState( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, POWER_STATE *ps);
int  EtronDI_SetMobileLEDBrightnessLevel( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BRIGHTNESS_LEVEL bl);
int  EtronDI_GetMobileLEDBrightnessLevel( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BRIGHTNESS_LEVEL *pbl);
// for LED and Laser -

// Return home +
int  EtronDI_MotorStartReturnHome( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);
int  EtronDI_IsMotorReturnHomeRunning( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);
// Return home -

// for 3D Motor Control -

//#ifdef __cplusplus
//}
//#endif

#endif // LIB_ESPDI_H
