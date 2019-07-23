/*! \file eSPDI.h
  	\brief Etron SDK API export functions, data structure and variable definition
  	Copyright:
	This file copyright (C) 2017 by

	eYs3D an Etron company

	An unpublished work.  All rights reserved.

	This file is proprietary information, and may not be disclosed or
	copied without the prior permission of eYs3D an Etron company.
 */
#ifndef LIB_ESPDI_H
#define LIB_ESPDI_H

#include "eSPDI_def.h"
#include "eSPDI_version.h"
#include <stdlib.h>
int  EtronDI_Init(void **ppHandleEtronDI, bool bIsLogEnabled);
int  EtronDI_FindDevice(void *pHandleEtronDI);
void EtronDI_Release(void **ppHandleEtronDI);
int  EtronDI_RefreshDevice(void *pHandleEtronDI);
int EtronDI_SwitchBaseline(int index);
bool EtronDI_IsMLBaseLine(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);
int EtronDI_DoFusion(unsigned char **pDepthBufList, double *pDepthMerge, unsigned char *pDepthMergeFlag, int nDWidth, int nDHeight, double fFocus, double *pBaseline, double *pWRNear, double *pWRFar, double *pWRFusion, int nMergeNum, bool bdepth2Byte11bit, int method);

int  EtronDI_GetDeviceNumber(void *pHandleEtronDI);
int  EtronDI_GetDeviceInfo(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo ,DEVINFORMATION* pdevinfo);
int  EtronDI_GetDeviceInfoMBL_15cm(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo ,DEVINFORMATION* pdevinfo);
int  EtronDI_SelectDevice(void *pHandleEtronDI, int dev_index);
bool EtronDI_IsInterleaveDevice(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);
int EtronDI_EnableInterleave(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, bool enable);

// register APIs +
int  EtronDI_GetSensorRegister(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nId, unsigned short address, unsigned short *pValue, int flag, SENSORMODE_INFO SensorMode);
int  EtronDI_SetSensorRegister(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nId, unsigned short address, unsigned short nValue,  int flag, SENSORMODE_INFO SensorMode);
int  EtronDI_GetFWRegister(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short address, unsigned short *pValue, int flag);
int  EtronDI_SetFWRegister(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short address, unsigned short nValue,  int flag);
int  EtronDI_GetHWRegister(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short address, unsigned short *pValue, int flag);
int  EtronDI_SetHWRegister(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short address, unsigned short nValue,  int flag);
int  EtronDI_GetHWRegisterMBL_15cm(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short address, unsigned short *pValue, int flag);
int  EtronDI_SetHWRegisterMBL_15cm(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short address, unsigned short nValue,  int flag);
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
int  EtronDI_GetDeviceResolutionList(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, 
									  int nMaxCount, ETRONDI_STREAM_INFO *pStreamInfo0, 
									  int nMaxCount1, ETRONDI_STREAM_INFO *pStreamInfo1);
int  EtronDI_Setup_v4l2_requestbuffers(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int cnt);
int  EtronDI_OpenDevice(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo,
                          int nEP0Width, int nEP0Height, bool bEP0MJPG,
                          int nEP1Width, int nEP1Height,
                          DEPTH_TRANSFER_CTRL dtc=DEPTH_IMG_NON_TRANSFER,
                          bool bIsOutputRGB24=false, void *phWndNotice=0,
                          int *pFPS=0, CONTROL_MODE cm=IMAGE_SN_NONSYNC);
                                     			
int  EtronDI_OpenDevice2(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, 
						  int nEP0Width, int nEP0Height, bool bEP0MJPG, 
						  int nEP1Width, int nEP1Height, 
						  DEPTH_TRANSFER_CTRL dtc=DEPTH_IMG_NON_TRANSFER,
						  bool bIsOutputRGB24=false, void *phWndNotice=0, 
						  int *pFPS=0, CONTROL_MODE cm=IMAGE_SN_NONSYNC);	
int  EtronDI_OpenDeviceMBL(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo,
                          int nEP0Width, int nEP0Height, bool bEP0MJPG,
                          int nEP1Width, int nEP1Height,
                          DEPTH_TRANSFER_CTRL dtc=DEPTH_IMG_NON_TRANSFER,
                          bool bIsOutputRGB24=false, void *phWndNotice=0,
                          int *pFPS=0, CONTROL_MODE cm=IMAGE_SN_NONSYNC);


int  EtronDI_CloseDeviceMBL(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);

int  EtronDI_CloseDevice(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);

int EtronDI_CloseDeviceEx(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);

int  EtronDI_GetImage(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo,
                        BYTE *pBuf, unsigned long int *pImageSize,
                        int *pSerial = 0, int nDepthDataType =0);

int  EtronDI_GetColorImage(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo,
                        BYTE *pBuf, unsigned long int *pImageSize,
                        int *pSerial = 0, int nDepthDataType =0);

int  EtronDI_GetDepthImage(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo,
                        BYTE *pBuf, unsigned long int *pImageSize,
                        int *pSerial = 0, int nDepthDataType =0);

int  EtronDI_Get_Color_30_mm_depth(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo,
                        BYTE *pBuf, unsigned long int *pImageSize,
                        int *pSerial = 0, int nDepthDataType =0);

int  EtronDI_Get_60_mm_depth(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo,
                        BYTE *pBuf, unsigned long int *pImageSize,
                        int *pSerial = 0, int nDepthDataType =0);

int  EtronDI_Get_150_mm_depth(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo,
                        BYTE *pBuf, unsigned long int *pImageSize,
                        int *pSerial = 0, int nDepthDataType =0);


int  EtronDI_Get2Image (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo,
                        BYTE *pColorImgBuf, BYTE *pDepthImgBuf,
                        unsigned long int *pColorImageSize, unsigned long int *pDepthImageSize,
                        int *pSerial = 0, int *pSerial2 = 0, int nDepthDataType =0);
// image -			

// for AEAWB Control +
int  EtronDI_GetExposureTime(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nSensorMode, float *pfExpTimeMS);
int  EtronDI_SetExposureTime(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nSensorMode, float fExpTimeMS);
int  EtronDI_GetGlobalGain(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nSensorMode, float *pfGlobalGain);
int  EtronDI_SetGlobalGain(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nSensorMode, float fGlobalGain);
int  EtronDI_SetSensorTypeName(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, SENSOR_TYPE_NAME stn);
int  EtronDI_GetColorGain(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nSensorMode, float *pfGainR, float *pfGainG, float *pfGainB);
int  EtronDI_SetColorGain(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nSensorMode, float fGainR, float fGainG, float fGainB);
#ifndef DOXYGEN_SHOULD_SKIP_THIS
int  EtronDI_GetAccMeterValue(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int *pX, int *pY, int *pZ);
#endif

int  EtronDI_EnableAE     (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);
int  EtronDI_DisableAE    (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);
int  EtronDI_EnableAWB    (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);
int  EtronDI_DisableAWB   (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);
int  EtronDI_GetAEStatus (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, PAE_STATUS pAEStatus);
int  EtronDI_GetAWBStatus (void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, PAWB_STATUS pAWBStatus);
int  EtronDI_GetGPIOValue(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nGPIOIndex, BYTE *pValue);
int  EtronDI_SetGPIOValue(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nGPIOIndex, BYTE nValue);
int  EtronDI_SetGPIOCtrl(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nGPIOIndex, BYTE nValue);

int  EtronDI_GetCTPropVal(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nId, long int *pValue);
int  EtronDI_SetCTPropVal(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nId, long int nValue);

int  EtronDI_GetPUPropVal(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nId, long int *pValue);
int  EtronDI_SetPUPropVal(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nId, long int nValue);

#if 0
int  EtronDI_GetCTRangeAndStep(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nId, long int *pMax, long int *pMin, long int *pStep, long int *pDefault, long int *pFlags);
int  EtronDI_GetPURangeAndStep(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int nId, long int *pMax, long int *pMin, long int *pStep, long int *pDefault, long int *pFlags);
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
int  EtronDI_GetRectifyLogData(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, eSPCtrl_RectLogData *pData, int index);
int  EtronDI_GetRectifyMatLogData(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, eSPCtrl_RectLogData *pData, int index);
// for Calibration Log -
// for Post Process +
#ifndef DOXYGEN_SHOULD_SKIP_THIS
int  EtronDI_EnablePostProcess(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, bool bEnable);
int  EtronDI_PostInitial(void *pHandleEtronDI);
int  EtronDI_PostEnd(void *pHandleEtronDI);
int  EtronDI_ProcessFrame(void *pHandleEtronDI, unsigned char *pYUY2Buf, unsigned char *pDepthBuf, unsigned char *OutputBuf, int width, int height);
int  EtronDI_PostSetParam(void *pHandleEtronDI, int Idx, int Val);	
int  EtronDI_PostGetParam(void *pHandleEtronDI, int Idx, int *pVal);	
#endif

int EtronDI_CreateSwPostProc(int depthBits, void** handle);
int EtronDI_ReleaseSwPostProc(void** handle);
int EtronDI_DoSwPostProc(void *pHandleEtronDI, unsigned char* colorBuf, bool isColorRgb24,
    unsigned char* depthBuf, unsigned char* outputBuf, int width, int height);
int EtronDI_Convert_Depth_Y_To_Buffer(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned char *depth_y, unsigned char *rgb, unsigned int width, unsigned int height, bool color, unsigned short nDepthDataType);
int EtronDI_Convert_Depth_Y_To_Buffer_offset(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned char *depth_y, unsigned char *rgb, unsigned int width, unsigned int height, bool color, unsigned short nDepthDataType, int offset);
// for Post Process -
// for sensorif +
int  EtronDI_EnableSensorIF(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, bool bIsEnable);
// for sensorif -

#ifndef __WEYE__ 
// for Gyro +
#ifndef UAC_NOT_SUPPORTED
int EtronDI_getUACNAME(char *input, char *output);
int EtronDI_InitialUAC(char *deviceName);
int EtronDI_WriteWaveHeader(int fd);
int EtronDI_WriteWaveEnd(int fd, size_t length);
int EtronDI_GetUACData(unsigned char *buffer, int length);
int EtronDI_ReleaseUAC(void);
#endif
int EtronDI_InitialFlexibleGyro(void* pHandleEtronDI, PDEVSELINFO pDevSelInfo);
int EtronDI_ReleaseFlexibleGyro(void* pHandleEtronDI, PDEVSELINFO pDevSelInfo);
int EtronDI_GetFlexibleGyroData(void* pHandleEtronDI,PDEVSELINFO pDevSelInfo, int length, unsigned char* pGyroData);
int EtronDI_GetFlexibleGyroLength(void* pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short* GyroLen);
int EtronDI_GetImageInterrupt(void);
int EtronDI_InitialHidGyro(void* pHandleEtronDI, PDEVSELINFO pDevSelInfo);
int EtronDI_ReleaseHidGyro(void* pHandleEtronDI, PDEVSELINFO pDevSelInfo);
int EtronDI_GetHidGyro(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned char *pBuffer, int length);
int EtronDI_SetupHidGyro(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned char *pCmdBuf, int cmdlength);
int EtronDI_GetInfoHidGyro(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned char *pCmdBuf, int cmdlength, unsigned char *pResponseBuf, int *resplength);
// for Gyro -
#endif

#ifndef TINY_VERSION
int EtronDI_GenerateLutFile(void* pHandleEtronDI, PDEVSELINFO pDevSelInfo, const char* filename);
int EtronDI_SaveLutData(void* pHandleEtronDI, PDEVSELINFO pDevSelInfo, const char* filename);
int EtronDI_GetLutData(void* pHandleEtronDI, PDEVSELINFO pDevSelInfo, BYTE* buffer, int nSize);

int EtronDI_EncryptMP4(void* pHandleEtronDI, PDEVSELINFO pDevSelInfo, const char* filename);
int EtronDI_DecryptMP4(void* pHandleEtronDI, PDEVSELINFO pDevSelInfo, const char* filename);
#ifndef DOXYGEN_SHOULD_SKIP_THIS
int EtronDI_InjectExtraDataToMp4(void* pHandleEtronDI, PDEVSELINFO pDevSelInfo, 
		const char* filename, const char* data, int dataLen);
int EtronDI_RetrieveExtraDataFromMp4(void* pHandleEtronDI, PDEVSELINFO pDevSelInfo, 
		const char* filename, char* data, int* dataLen);
int EtronDI_EncryptString(const char* src, char* dst);
int EtronDI_DecryptString(const char* src, char* dst);
int EtronDI_EncryptString(const char* src1, const char* src2, char* dst);
int EtronDI_DecryptString(const char* src, char* dst1, char* dst2);
#endif
#endif
int EtronDI_GetAutoExposureMode(void* pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short* mode);
int EtronDI_SetAutoExposureMode(void* pHandleEtronDI, PDEVSELINFO pDevSelInfo, unsigned short mode);
#ifndef OPENCV_NOT_SUPPORTED
int EtronDI_RotateImg90(EtronDIImageType::Value imgType, int width, int height, unsigned char *src, unsigned char *dst, int len, bool clockwise);
int EtronDI_RotateImg180(EtronDIImageType::Value imgType, int width, int height,unsigned char *src, unsigned char *dst, int len);
int EtronDI_ResizeImgToHalf(EtronDIImageType::Value imgType, int width, int height, unsigned char *src, unsigned char *dst, int len);
int EtronDI_ImgMirro(EtronDIImageType::Value imgType, int width, int height, unsigned char *src, unsigned char *dst);
int EtronDI_RGB2BMP(char *filename, int width, int height, unsigned char *data);
#endif
#endif // LIB_ESPDI_H
