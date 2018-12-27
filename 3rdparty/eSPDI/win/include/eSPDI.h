 /*! \file eSPDI_DM.h
  	\brief Etron SDK API export functions, data structure and variable definition for depth map module
  	\copyright This file copyright (C) 2017 by eYs3D an Etron company

	\details An unpublished work.  All rights reserved.
	This file is proprietary information, and may not be disclosed or
	copied without the prior permission of eYs3D an Etron company.
 */
#pragma once
#include "eSPDI_Common.h"

// for EtronDI_PostSetParam/EtronDI_PostGetParam
#define POSTPAR_HR_MODE        5
#define POSTPAR_HR_CURVE_0     6
#define POSTPAR_HR_CURVE_1     7
#define POSTPAR_HR_CURVE_2     8
#define POSTPAR_HR_CURVE_3     9
#define POSTPAR_HR_CURVE_4    10
#define POSTPAR_HR_CURVE_5    11
#define POSTPAR_HR_CURVE_6    12
#define POSTPAR_HR_CURVE_7    13
#define POSTPAR_HR_CURVE_8    14
#define POSTPAR_HF_MODE       17
#define POSTPAR_DC_MODE       20
#define POSTPAR_DC_CNT_THD    21
#define POSTPAR_DC_GRAD_THD   22
#define POSTPAR_SEG_MODE      23
#define POSTPAR_SEG_THD_SUB   24
#define POSTPAR_SEG_THD_SLP   25
#define POSTPAR_SEG_THD_MAX   26
#define POSTPAR_SEG_THD_MIN   27
#define POSTPAR_SEG_FILL_MODE 28
#define POSTPAR_HF2_MODE      31
#define POSTPAR_GRAD_MODE     34
#define POSTPAR_TEMP0_MODE    37
#define POSTPAR_TEMP0_THD     38
#define POSTPAR_TEMP1_MODE    41
#define POSTPAR_TEMP1_LEVEL   42
#define POSTPAR_TEMP1_THD     43
#define POSTPAR_FC_MODE       46
#define POSTPAR_FC_EDGE_THD   47
#define POSTPAR_FC_AREA_THD   48
#define POSTPAR_MF_MODE       51
#define POSTPAR_ZM_MODE       52
#define POSTPAR_RF_MODE       53
#define POSTPAR_RF_LEVEL      54

//
// C++ compatibility
//
#ifdef  __cplusplus
extern "C" {
#endif

/*! \fn int EtronDI_GetYOffset(
		void *pHandleEtronDI,
		PDEVSELINFO pDevSelInfo,
		BYTE *buffer,
		int BufferLength,
		int *pActualLength,
		int index)
	\brief get Y offset data
	\param pHandleEtronDI	 the pointer to the initilized EtronDI SDK instance
	\param pDevSelInfo	pointer of device select index
	\param buffer	buffer to store
	\param BufferLength	length of buffer
	\param pActualLength	actual byte of reading
	\param index	index of Y offset file ID
	\return success:EtronDI_OK, others:see eSPDI_ErrCode.h
*/
int ETRONDI_API EtronDI_GetYOffset(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BYTE *buffer, int BufferLength, int *pActualLength, int index);

/*!	\fn EtronDI_GetRectifyTable(
		void *pHandleEtronDI,
		PDEVSELINFO pDevSelInfo,
		BYTE *buffer,
		int BufferLength,
		int *pActualLength,
		int index)
	\brief get rectify values from flash
	\param pHandleEtronDI	 the pointer to the initilized EtronDI SDK instance
	\param pDevSelInfo	pointer of device select index
	\param buffer	buffer to store rectify table data
	\param BufferLength	input buffer length
	\param pActualLength	actual length has written to buffer
	\param index	index to identify rectify table for corresponding depth
	\return success:EtronDI_OK, others: see eSPDI_ErrCode.h
*/
int ETRONDI_API EtronDI_GetRectifyTable(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BYTE *buffer, int BufferLength, int *pActualLength, int index);

/*! \fn int EtronDI_GetZDTable(
		void *pHandleEtronDI,
		PDEVSELINFO pDevSelInfo,
		BYTE *buffer,
		int BufferLength,
		int *pActualLength,
		PZDTABLEINFO pZDTableInfo)
	\brief get disparity and Z values from flash
		1. if depth data type is ETronDI_DEPTH_DATA_14_BITS then
			just get Z value from depth buffer 
		2. if depth data type is ETronDI_ZD_TABLE_FILE_SIZE_11_BITS then
			using depth buffer value as a index to get Z value inside ZD table
		3. see GetZValue() of example.c to get Z value from different depth data type
	\param pHandleEtronDI	 the pointer to the initilized EtronDI SDK instance
	\param pDevSelInfo	pointer of device select index
	\param buffer	bufer to store ZD table
	\param BufferLength	input buffer length
	\param pActualLength	actual length has written to buffer
	\param pZDTableInfo	index to identify ZD table and data type for corrresponding depth
	\return success: EtronDI_OK, others: see eSPDI_ErrCode.h
*/
int ETRONDI_API EtronDI_GetZDTable(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BYTE *buffer, int BufferLength, int *pActualLength, PZDTABLEINFO pZDTableInfo);

/*! \fn int EtronDI_SetYOffset(
		void *pHandleEtronDI, 
		PDEVSELINFO pDevSelInfo, 
		BYTE *buffer, 
		int BufferLength, 
		int *pActualLength, 
		int index)
	\brief set Y offset data
	\param pHandleEtronDI	 the pointer to the initilized EtronDI SDK instance
	\param pDevSelInfo	pointer of device select index
	\param buffer	buffer to store
	\param BufferLength	length of buffer
	\param pActualLength	actual byte of reading
	\param index	index of Y offset file ID
	\return success:EtronDI_OK, others:see eSPDI_ErrCode.h
*/
int ETRONDI_API EtronDI_SetYOffset(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BYTE *buffer, int BufferLength, int *pActualLength, int index);

/*! \fn int EtronDI_SetRectifyTable(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BYTE *buffer, int BufferLength, int *pActualLength, int index)
	\brief get rectify data to flash, see EtronDI_GetRectifyTable except get
*/
int ETRONDI_API EtronDI_SetRectifyTable(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BYTE *buffer, int BufferLength, int *pActualLength, int index);

/*! \fn EtronDI_SetZDTable(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BYTE *buffer, int BufferLength, int *pActualLength, PZDTABLEINFO pZDTableInfo)
	\brief set disparity and Z values to flash, see EtronDI_GetZDTable except get
*/
int ETRONDI_API EtronDI_SetZDTable(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, BYTE *buffer, int BufferLength, int *pActualLength, PZDTABLEINFO pZDTableInfo);

/*! \fn int EtronDI_GetRectifyLogData(
		void *pHandleEtronDI,
		PDEVSELINFO pDevSelInfo,
		eSPCtrl_RectLogData *pData,
		int index)
	\brief get rectify log data from flash
	\param pHandleEtronDI	 the pointer to the initilized EtronDI SDK instance
	\param pDevSelInfo	pointer of device select index
	\param pData	 4096 bytes of rectify log data,
		see eSPCtrl_RectLogData for detailed members
	\param index	index to identify rectify log data for corresponding depth
	\return success: EtronDI_OK, others: see eSPDI_ErrCode.h
*/
int ETRONDI_API EtronDI_GetRectifyLogData(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, eSPCtrl_RectLogData *pData, int index);

/*! \fn int EtronDI_GetRectifyMatLogData(
		void *pHandleEtronDI,
		PDEVSELINFO pDevSelInfo,
		eSPCtrl_RectLogData *pData,
		int index)
	\brief get rectify log data from flash for Puma IC
	\param pHandleEtronDI	 the pointer to the initilized EtronDI SDK instance
	\param pDevSelInfo	pointer of device select index
	\param pData	 rectify log data, its buffer size is 4096 bytes  
		see eSPCtrl_RectLogData for detailed members
	\param index	index to identify rectify log data for corresponding depth
	\return success: EtronDI_OK, others: see eSPDI_ErrCode.h
*/
int ETRONDI_API EtronDI_GetRectifyMatLogData(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, eSPCtrl_RectLogData *pData, int index);

/*! \fn int EtronDI_SaveRectifyLogDataA( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, eSPCtrl_RectLogData *pData, char *lpszFileName)
	\brief save the rectify log data
*/
int ETRONDI_API EtronDI_SaveRectifyLogDataA(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, eSPCtrl_RectLogData *pData, char *lpszFileName);


/*! \fn int EtronDI_SaveRectifyLogDataW( void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, eSPCtrl_RectLogData *pData, WCHAR *lpwszFileName)
	\brief save the rectify log data in union-code
*/
int ETRONDI_API EtronDI_SaveRectifyLogDataW(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, eSPCtrl_RectLogData *pData, WCHAR *lpwszFileName);

/*! \fn EtronDI_SetDepthDataType(
		void *pHandleEtronDI,
		PDEVSELINFO pDevSelInfo,
		WORD wType)
	\brief set depth data type, 11 bit for disparity data, 14 bit for Z data
		notice: only PUMA type IC can support this setting
	\param pHandleEtronDI	 the pointer to the initilized EtronDI SDK instance
	\param pDevSelInfo	pointer of device select index
	\param wType	depth data type you want to set,
		see ETronDI_DEPTH_DATA_xxx in EtronDI_O.h
	\output success: EtronDI_OK, others: see eSPDI_ErrCode.h
*/
int ETRONDI_API EtronDI_SetDepthDataType(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, WORD wType);

/*! \fn int EtronDI_GetDepthDataType(
		void *pHandleEtronDI,
		PDEVSELINFO pDevSelInfo,
		WORD *pwType)
	\brief get current depth data type setting
	\param pHandleEtronDI	 the pointer to the initilized EtronDI SDK instance
	\param pDevSelInfo	pointer of device select index
	\param pwType	pointer of current depth data type in device
	\return success: EtronDI_OK, others: see eSPDI_ErrCode.h
*/
int ETRONDI_API EtronDI_GetDepthDataType(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, WORD *pwType);

/*! \fn int EtronDI_SetHWPostProcess(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, bool enable)
	\brief enable or disable internal chip post processing function
	\param pHandleEtronDI	 the pointer to the initilized EtronDI SDK instance
	\param pDevSelInfo	pointer of device select index
	\param enable	set true to enable post-process, or set false to disable post-process
	\return success: EtronDI_OK, others: see eSPDI_ErrCode.h
*/
int ETRONDI_API EtronDI_SetHWPostProcess(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, bool enable);

/*! \fn int EtronDI_GetHWPostProcess(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, bool* enable)
	\brief get hardware post processing status
	\param pHandleEtronDI	 the pointer to the initilized EtronDI SDK instance
	\param pDevSelInfo	pointer of device select index
	\param enable	returns current hardware post-process status
	\return success: EtronDI_OK, others: see eSPDI_ErrCode.h	
*/
int ETRONDI_API EtronDI_GetHWPostProcess(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, bool* enable);

/*! \fn int EtronDI_PostProcessInitial( void *pHandleEtronDI)
	\brief initialize software post processing
	\param pHandleEtronDI	 the pointer to the initilized EtronDI SDK instance
	\return success: EtronDI_OK, others: see eSPDI_ErrCode.h	
*/
int ETRONDI_API EtronDI_PostProcessInitial(void *pHandleEtronDI);

/*! \fn int EtronDI_PostProcessEnd( void *pHandleEtronDI)
	\brief release software post processing
	\param pHandleEtronDI	 the pointer to the initilized EtronDI SDK instance
	\return success: EtronDI_OK, others: see eSPDI_ErrCode.h
*/
int ETRONDI_API EtronDI_PostProcessEnd(void *pHandleEtronDI);

/*! \fn int EtronDI_ProcessFrame( void *pHandleEtronDI, unsigned char *pYUY2Buf, unsigned char *pDepthBuf, unsigned char *pOutputBuf, int width, int height)
	\brief by SW post process to enhance depth image
	\param pHandleEtronDI	 the pointer to the initilized EtronDI SDK instance
	\param pYUY2Buf	input YUY2 color image
	\param pDepthBuf	input depth buffer
	\param pOutputBuf	the result image
	\param width	image width
	\param height	image heigh
	\return success: EtronDI_OK, others: see eSPDI_ErrCode.h
*/
int ETRONDI_API EtronDI_ProcessFrame(void *pHandleEtronDI, unsigned char *pYUY2Buf, unsigned char *pDepthBuf, unsigned char *pOutputBuf, int width, int height);

/*! \fn int EtronDI_CreateSwPostProc(int depthBits, void** handle)
	\brief create a software post process class
	\param depthBits	depth bit to set
	\param handle	pointer to the handle of this software post process class
	\return success: EtronDI_OK, others: see eSPDI_ErrCode.h
*/
int ETRONDI_API EtronDI_CreateSwPostProc(int depthBits, void** handle);

/*! \fn int EtronDI_ReleaseSwPostProc(void** handle)
	\brief release a software post process class
	\param handle	pointer to the handle of this software post process class
	\return success: EtronDI_OK, others: see eSPDI_ErrCode.h
*/
int ETRONDI_API EtronDI_ReleaseSwPostProc(void** handle);

/*! \fn int EtronDI_DoSwPostProc(void* handle, unsigned char* colorBuf, bool isColorRgb24, unsigned char* depthBuf, unsigned char* outputBuf, int width, int height)
	\brief do software post process on a depth buffer
	\param handle	handle of this software post process class
	\param colorBuf	input color buffer
	\param isColorRgb24	is this color buffer RGB888
	\param depthBuf	input depth buffer
	\param outputBuf	output buffer
	\param width	image width
	\param height	image height
	\return success: EtronDI_OK, others: see eSPDI_ErrCode.h
*/
int ETRONDI_API EtronDI_DoSwPostProc(void* handle, unsigned char* colorBuf, bool isColorRgb24, 
    unsigned char* depthBuf, unsigned char* outputBuf, int width, int height);

/*! \fn bool EtronDI_HasMultiDepth0ColorPlusDepthForColor(
		void *pHandleEtronDI,
		PDEVSELINFO pDevSelInfo,
		int colorStreamIndex)
	\brief check module support multi-base line w/ color+depth on depth 0(S0) pin
	\param pHandleEtronDI	 the pointer to the initilized EtronDI SDK instance
	\param pDevSelInfo	pointer of device select index
	\param colorStreamIndex	color stream index to check
	\return true: support S0 w/ color combine with depth,but its output is the only depth and depth is color stream, false: not support
*/
bool ETRONDI_API EtronDI_HasMultiDepth0ColorPlusDepthForColor(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int colorStreamIndex);

/*! \fn bool EtronDI_HasMultiDepth0ColorPlusDepth(
		void *pHandleEtronDI,
		PDEVSELINFO pDevSelInfo,
		int colorStreamIndex)
	\brief check module support multi-base line w/ color+depth on depth 0(S0) pin
	\param pHandleEtronDI	 the pointer to the initilized EtronDI SDK instance
	\param pDevSelInfo	pointer of device select index
	\param colorStreamIndex	color stream index to check
	\return true: support S0 w/ color combine with depth, false: not support
*/
bool ETRONDI_API EtronDI_HasMultiDepth0ColorPlusDepth(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, int colorStreamIndex);

/*! \fn bool EtronDI_HasMultiDepth2(
		void *pHandleEtronDI,
		PDEVSELINFO pDevSelInfo)
	\brief check module support multi-base line w/ depth 2(S2) pin
	\param pHandleEtronDI	 the pointer to the initilized EtronDI SDK instance
	\param pDevSelInfo	pointer of device select index
	\return true: support S2, false: not support
*/
bool ETRONDI_API EtronDI_HasMultiDepth2(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);

/*! \fn bool EtronDI_IsDeocclusionDevice(
		void *pHandleEtronDI,
		PDEVSELINFO pDevSelInfo)
	\brief check module support de-occlusion function or not
	\param pHandleEtronDI	 the pointer to the initilized EtronDI SDK instance
	\param pDevSelInfo	pointer of device select index
	\return true: support deocclusion, false: not support
*/
bool ETRONDI_API EtronDI_IsDeocclusionDevice(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);

/*! \fn bool EtronDI_IsInterleaveDevice(
void *pHandleEtronDI,
PDEVSELINFO pDevSelInfo)
\brief check module support interleave function or not
\param pHandleEtronDI	 the pointer to the initilized EtronDI SDK instance
\param pDevSelInfo	pointer of device select index
\return true: support interleave, false: not support
*/
bool ETRONDI_API EtronDI_IsInterleaveDevice(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo);

/*! \fn int EtronDI_IsInterleaveDevice(
void *pHandleEtronDI,
PDEVSELINFO pDevSelInfo)
\brief enable or disable interleave function
\param pHandleEtronDI	 the pointer to the initilized EtronDI SDK instance
\param pDevSelInfo	pointer of device select index
\param enable	set true to enable interleave, or set false to disable interleave
\return success: EtronDI_OK, others: see eSPDI_ErrCode.h
*/
int ETRONDI_API EtronDI_EnableInterleave(void *pHandleEtronDI, PDEVSELINFO pDevSelInfo, bool enable);

#ifdef __cplusplus
}
#endif