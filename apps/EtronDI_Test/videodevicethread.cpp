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
#include "videodevicethread.h"
#include "videodevicedlg.h"
#include "mainwindow.h"

CVideoDeviceThread::CVideoDeviceThread(QWidget *parent) {
	
	m_VideoDeviceDlg = (CVideoDeviceDlg*)parent; 
	
	m_bIsSuccess = true;
}

void CVideoDeviceThread::run() {
    
    if(m_VideoDeviceDlg->m_funcSelect == READ_FLASH)  readFlash();
    if(m_VideoDeviceDlg->m_funcSelect == WRITE_FLASH) writeFlash();
}

void CVideoDeviceThread::readFlash() {
	
	BYTE *pBuffer = NULL;
    unsigned long int nActualLength = 0;
    unsigned long int BufferLength  = 0;
    DEVINFORMATION devInfo;
    
    char szFileName[64] = {0};

    int nRet = ETronDI_OK;

    EtronDI_GetDeviceInfo( m_VideoDeviceDlg->m_pMainWindow->m_pEtronDI, &(m_VideoDeviceDlg->m_DevSelInfo), &devInfo);
    
    if(m_VideoDeviceDlg->m_FlashDataType == Total) {
        BufferLength = 1024*128;
        sprintf( szFileName, "total(%d).bin", m_VideoDeviceDlg->m_DevSelInfo.index);
	}
	
	if(m_VideoDeviceDlg->m_FlashDataType == FW_PLUGIN) {
        if(devInfo.nDevType == AXES1) BufferLength = 1024*104;
        else BufferLength = 1024*128;
        sprintf( szFileName, "fwplugin(%d).bin", m_VideoDeviceDlg->m_DevSelInfo.index);
	}
	
	if(m_VideoDeviceDlg->m_FlashDataType == BOOTLOADER_ONLY) {
        BufferLength = 1024*24;
        sprintf( szFileName, "bootloader(%d).bin", m_VideoDeviceDlg->m_DevSelInfo.index);
	}
	
	if(m_VideoDeviceDlg->m_FlashDataType == FW_ONLY) {
        BufferLength = 1024*40;
        sprintf( szFileName, "fwonly(%d).bin", m_VideoDeviceDlg->m_DevSelInfo.index);
	}
	
	if(m_VideoDeviceDlg->m_FlashDataType == PLUGIN_ONLY) {
        BufferLength = 1024*56;
        sprintf( szFileName, "plugin(%d).bin", m_VideoDeviceDlg->m_DevSelInfo.index);
	}
	
	pBuffer = (BYTE*)malloc(sizeof(BYTE)*BufferLength);
	
    nRet = EtronDI_ReadFlashData( m_VideoDeviceDlg->m_pMainWindow->m_pEtronDI, &(m_VideoDeviceDlg->m_DevSelInfo),
                                  m_VideoDeviceDlg->m_FlashDataType, pBuffer, BufferLength, &nActualLength);
    
    if(nRet == ETronDI_OK) {
        
		FILE *fp;
		fp = fopen( szFileName, "wb");
		
		if(!fp) {
			qDebug("Open file failed !!\n");
			m_bIsSuccess = false;
		}
		else {				
			fseek( fp, 0, SEEK_SET);
			fwrite( pBuffer, sizeof(BYTE), BufferLength, fp);
			fclose(fp);
		}
    }

    else m_bIsSuccess = false;

    if(pBuffer != NULL) {
		free(pBuffer);
		pBuffer = NULL;
	}
	
	emit done(m_bIsSuccess);
}

void CVideoDeviceThread::writeFlash() {
	
	BYTE *pBuffer = NULL;
	unsigned long int BufferLength  = 0;
    DEVINFORMATION devInfo;
    
    char szFileName[64] = {0};
    
    int nRet = ETronDI_OK;

    EtronDI_GetDeviceInfo( m_VideoDeviceDlg->m_pMainWindow->m_pEtronDI, &(m_VideoDeviceDlg->m_DevSelInfo), &devInfo);
    
    if(m_VideoDeviceDlg->m_FlashDataType == Total) {		
        BufferLength = 1024*120;
        sprintf( szFileName, "total.bin");
	}

    if(m_VideoDeviceDlg->m_FlashDataType == FW_PLUGIN) {
        if(devInfo.nDevType == AXES1) BufferLength = 1024*96;
        else BufferLength = 1024*128;
        sprintf( szFileName, "fwplugin(%d).bin", m_VideoDeviceDlg->m_DevSelInfo.index);
    }
	
	if(m_VideoDeviceDlg->m_FlashDataType == BOOTLOADER_ONLY) {
        BufferLength = 1024*24;
        sprintf( szFileName, "bootloader.bin");
	}
	
	if(m_VideoDeviceDlg->m_FlashDataType == FW_ONLY) {
        BufferLength = 1024*40;
        sprintf( szFileName, "fwonly.bin");
	}
	
	if(m_VideoDeviceDlg->m_FlashDataType == PLUGIN_ONLY) {
        BufferLength = 1024*56;
        sprintf( szFileName, "plugin.bin");
	}
	
	pBuffer = (BYTE*)malloc(sizeof(BYTE)*BufferLength);
	memset(pBuffer, 0xff, BufferLength);
	
	FILE *fp;
	fp = fopen( szFileName, "rb");
	
	if(!fp) {
		qDebug("File doesn't exist !!");
		m_bIsSuccess = false;
	}
	else {
        /*
        fseek( fp, 0, SEEK_END);
        unsigned long int nFileLen = ftell(fp);
        if(nFileLen > BufferLength) {
            m_bIsSuccess = false;
            fclose(fp);
            return;
        }
        */
		fseek( fp, 0, SEEK_SET);
		
        fread( pBuffer, sizeof(BYTE), BufferLength, fp);
		fclose(fp);
		
		nRet = EtronDI_WriteFlashData( m_VideoDeviceDlg->m_pMainWindow->m_pEtronDI, &(m_VideoDeviceDlg->m_DevSelInfo),
                                       m_VideoDeviceDlg->m_FlashDataType, pBuffer, BufferLength, 
                                       false, m_VideoDeviceDlg->m_KeepDataCtrl);
        if(nRet != ETronDI_OK) m_bIsSuccess = false;     
    }

    if(pBuffer != NULL) {
        free(pBuffer);
        pBuffer = NULL;
    }

    emit done(m_bIsSuccess);
}

