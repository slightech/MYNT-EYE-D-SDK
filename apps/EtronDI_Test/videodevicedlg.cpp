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
#include "videodevicedlg.h"
#include "mainwindow.h"

CVideoDeviceDlg::CVideoDeviceDlg(int nVideoIndex, QWidget *parent) :
                         QDialog(parent)
{
    setupUi(this);

    m_pMainWindow = (MainWindow*)parent;
    m_DevSelInfo.index = nVideoIndex;
    m_nDepthDataType = 0;

    // check device type
    if(m_pMainWindow->m_pDevInfo[m_DevSelInfo.index].nDevType==PUMA){
        // set depth data type
        label_25->setEnabled(true);
        depthDataTypeCombo->setEnabled(true);
        m_nDepthDataType = 2;  // set default as 14 bits
        EtronDI_SetDepthDataType(m_pMainWindow->m_pEtronDI, &m_DevSelInfo, m_nDepthDataType);

        // set ZD table data type
        label_20->setEnabled(true);
        ZDDataTypeCombo->setEnabled(true);
    }

    // init data type combobox
    QStringList strList;
    strList << "Total" << "FW+PlugIn" << "BootLoader" << "FW Only" << "PlugIn Only";
    dataTypeComboBox->addItems(strList);
    
    // init depthmap output control
    strList.clear();
    strList << "Non-Transfer" << "Transfer-to-GrayRGB" << "Transfer-to-ColorfulRGB";
    depthOutputCtrlCombo->addItems(strList);
    if(m_pMainWindow->m_pDevInfo[m_DevSelInfo.index].nDevType==PUMA)
       depthOutputCtrlCombo->setCurrentIndex(2);

    // init depth data type combobox
    strList.clear();
    strList << "11 bits" << "14 bits";
    depthDataTypeCombo->addItems(strList);
    depthDataTypeCombo->setCurrentIndex(1);

    // init ZD table data type combobox
    strList.clear();
    strList << "8 bits" << "11 bits";
    ZDDataTypeCombo->addItems(strList);
    ZDDataTypeCombo->setCurrentIndex(0);
    m_nZDTableDataType = ETronDI_DEPTH_DATA_DEFAULT;

    // init color, depth resolution combobox +

    m_pStreamColorInfo = (PETRONDI_STREAM_INFO)malloc(sizeof(ETRONDI_STREAM_INFO)*64);
    m_pStreamDepthInfo = (PETRONDI_STREAM_INFO)malloc(sizeof(ETRONDI_STREAM_INFO)*64);

    m_nColorResIndex = 0;
    m_nDepthResIndex = 0;

	// init color, depth resolution combobox -
	
    connect( pidvidGetBtn,            SIGNAL(clicked(bool)),            this, SLOT(on_PID_Get_Click()));
    connect( pidvidSetBtn,            SIGNAL(clicked(bool)),            this, SLOT(on_PID_Set_Click()));
    connect( snGetBtn,                SIGNAL(clicked(bool)),            this, SLOT(on_SN_Get_Click()));
    connect( snSetBtn,                SIGNAL(clicked(bool)),            this, SLOT(on_SN_Set_Click()));
    connect( readFlashBtn,            SIGNAL(clicked(bool)),            this, SLOT(on_FLASH_Read_Click()));
    connect( writeFlashBtn,           SIGNAL(clicked(bool)),            this, SLOT(on_FLASH_Write_Click()));
    connect( previewBtn,              SIGNAL(clicked(bool)),            this, SLOT(on_PreviewBtn_Click()));
    connect( colorStreamCombo,        SIGNAL(currentIndexChanged(int)), this, SLOT(on_m_ColorRes_Combobox_currentIndexChanged(int)));
    connect( depthStreamCombo,        SIGNAL(currentIndexChanged(int)), this, SLOT(on_m_DepthRes_Combobox_currentIndexChanged(int)));
    connect( depthDataTypeCombo,      SIGNAL(currentIndexChanged(int)), this, SLOT(on_m_DepthDataType_Combobox_currentIndexChanged(int)));
    connect( readRegBtn,              SIGNAL(clicked(bool)),            this, SLOT(on_REG_Read_Click()));
    connect( writeRegBtn,             SIGNAL(clicked(bool)),            this, SLOT(on_REG_Write_Click()));
    connect( aeEnableChk,             SIGNAL(clicked(bool)),            this, SLOT(on_AE_State_Change(bool)));
    connect( awbEnableChk,            SIGNAL(clicked(bool)),            this, SLOT(on_AWB_State_Change(bool)));
    connect( updateAEAWBBtn,          SIGNAL(clicked(bool)),            this, SLOT(on_AE_AWB_State_update()));
    connect( readPropertyBtn,         SIGNAL(clicked(bool)),            this, SLOT(on_Read_Property_Click()));
    connect( writePropertyBtn,        SIGNAL(clicked(bool)),            this, SLOT(on_Write_Property_Click()));

    // for GPIO +
    connect( b7Btn,                   SIGNAL(clicked(bool)),            this, SLOT(on_GPIO1_BIT7_Click()));
    connect( b6Btn,                   SIGNAL(clicked(bool)),            this, SLOT(on_GPIO1_BIT6_Click()));
    connect( b5Btn,                   SIGNAL(clicked(bool)),            this, SLOT(on_GPIO1_BIT5_Click()));
    connect( b4Btn,                   SIGNAL(clicked(bool)),            this, SLOT(on_GPIO1_BIT4_Click()));
    connect( b3Btn,                   SIGNAL(clicked(bool)),            this, SLOT(on_GPIO1_BIT3_Click()));
    connect( b2Btn,                   SIGNAL(clicked(bool)),            this, SLOT(on_GPIO1_BIT2_Click()));
    connect( b1Btn,                   SIGNAL(clicked(bool)),            this, SLOT(on_GPIO1_BIT1_Click()));
    connect( b0Btn,                   SIGNAL(clicked(bool)),            this, SLOT(on_GPIO1_BIT0_Click()));
    connect( b7Btn_2,                 SIGNAL(clicked(bool)),            this, SLOT(on_GPIO2_BIT7_Click()));
    connect( b6Btn_2,                 SIGNAL(clicked(bool)),            this, SLOT(on_GPIO2_BIT6_Click()));
    connect( b5Btn_2,                 SIGNAL(clicked(bool)),            this, SLOT(on_GPIO2_BIT5_Click()));
    connect( b4Btn_2,                 SIGNAL(clicked(bool)),            this, SLOT(on_GPIO2_BIT4_Click()));
    connect( b3Btn_2,                 SIGNAL(clicked(bool)),            this, SLOT(on_GPIO2_BIT3_Click()));
    connect( b2Btn_2,                 SIGNAL(clicked(bool)),            this, SLOT(on_GPIO2_BIT2_Click()));
    connect( b1Btn_2,                 SIGNAL(clicked(bool)),            this, SLOT(on_GPIO2_BIT1_Click()));
    connect( b0Btn_2,                 SIGNAL(clicked(bool)),            this, SLOT(on_GPIO2_BIT0_Click()));
    connect( readGPIO1Btn,            SIGNAL(clicked(bool)),            this, SLOT(on_GPIO1Value_Read()));
    connect( writeGPIO1Btn,           SIGNAL(clicked(bool)),            this, SLOT(on_GPIO1Value_Write()));
    connect( readGPIO2Btn,            SIGNAL(clicked(bool)),            this, SLOT(on_GPIO2Value_Read()));
    connect( writeGPIO2Btn,           SIGNAL(clicked(bool)),            this, SLOT(on_GPIO2Value_Write()));
    //connect( propItemsBtn,            SIGNAL(clicked(bool)),            this, SLOT(on_Prop_Items_Click()));

	connect( readSensorOffBtn,        SIGNAL(clicked(bool)),            this, SLOT(on_Read_SensorOff_Click()));
    connect( writeSensorOffBtn,       SIGNAL(clicked(bool)),            this, SLOT(on_Write_SensorOff_Click()));
    connect( readRecTableBtn,         SIGNAL(clicked(bool)),            this, SLOT(on_Read_RecTable_Click()));
    connect( writeRecTableBtn,        SIGNAL(clicked(bool)),            this, SLOT(on_Write_RecTable_Click()));
    connect( readZDTableBtn,          SIGNAL(clicked(bool)),            this, SLOT(on_Read_ZDTable_Click()));
    connect( writeZDTableBtn,  		  SIGNAL(clicked(bool)),            this, SLOT(on_Write_ZDTable_Click()));
    connect( readCalibrationLogBtn,   SIGNAL(clicked(bool)),            this, SLOT(on_Read_Log_Click()));
    connect( writeCalibrationLogBtn,  SIGNAL(clicked(bool)),            this, SLOT(on_Write_Log_Click()));
    connect( ZDDataTypeCombo,         SIGNAL(currentIndexChanged(int)), this, SLOT(on_m_ZDDataType_Combobox_currentIndexChanged(int)));

    connect( readUserDataBtn,         SIGNAL(clicked(bool)),            this, SLOT(on_Read_UserData_Click()));
    connect( writeUserDataBtn,        SIGNAL(clicked(bool)),            this, SLOT(on_Write_UserData_Click()));

    // for GPIO -

    m_pVideoDeviceThread = NULL;

    m_Dtc = DEPTH_IMG_NON_TRANSFER;
    
    m_nFrameRate = 30;
    fpsLineEdit->setText(tr("30"));
    
    // for property ct pu +
    
    m_pPropertyItemArray = (PROPERTY_ITEM_INFO*)malloc(PROP_TOTAL_NUMBER*sizeof(PROPERTY_ITEM_INFO));
    
    // for property ct pu -
}

CVideoDeviceDlg::~CVideoDeviceDlg() {
    
    free(m_pStreamColorInfo);
    free(m_pStreamDepthInfo);
    
    if(m_pVideoDeviceThread != NULL) {
		delete m_pVideoDeviceThread;
		m_pVideoDeviceThread = NULL;
	}
	
	if(m_pPropertyItemArray != NULL) {
		delete m_pPropertyItemArray;
		m_pPropertyItemArray = NULL;
	}
}

void CVideoDeviceDlg::showEvent(QShowEvent *event) {

    QDialog::showEvent( event );
    
    //resize(1060,470);
    
    getFWVersion();
    on_PID_Get_Click();
    on_SN_Get_Click();
    getResolutionList();
    //getAEAWBStatus();
    initPropertyItemList();

    EtronDI_EnableAE( m_pMainWindow->m_pEtronDI, &m_DevSelInfo);
    aeEnableChk->setChecked(true);

    EtronDI_EnableAWB( m_pMainWindow->m_pEtronDI, &m_DevSelInfo);
    awbEnableChk->setChecked(true);
}

void CVideoDeviceDlg::closeEvent(QCloseEvent *event) {

    hide();
    event->ignore();
}

void CVideoDeviceDlg::on_PID_Get_Click() {

    char szBuf[256];

    unsigned short nPid = 0;
    unsigned short nVid = 0;
    EtronDI_GetPidVid(m_pMainWindow->m_pEtronDI, &m_DevSelInfo, &nPid, &nVid);

    sprintf(szBuf, "%04x", nPid);
    pidLineEdit->setText(szBuf);
    sprintf(szBuf, "%04x", nVid);
    vidLineEdit->setText(szBuf);
}

void CVideoDeviceDlg::on_PID_Set_Click() {

    unsigned short nPid = pidLineEdit->text().toInt(NULL, 16);
    unsigned short nVid = vidLineEdit->text().toInt(NULL, 16);
    EtronDI_SetPidVid(m_pMainWindow->m_pEtronDI, &m_DevSelInfo, &nPid, &nVid);
}

void CVideoDeviceDlg::on_SN_Get_Click() {

    unsigned char buffer[512];
    int  nActualLength;

    QString strSerialNum;
    int i = 0;

    if( ETronDI_OK == EtronDI_GetSerialNumber(m_pMainWindow->m_pEtronDI, &m_DevSelInfo, buffer, 512, &nActualLength)) {
        strSerialNum.resize(nActualLength/2);
        for(i=0;i<nActualLength/2;i++)
            strSerialNum[i] = QChar(buffer[i*2+1]*256+buffer[i*2]);
        snLineEdit->setText(strSerialNum);
    }
}

void CVideoDeviceDlg::on_SN_Set_Click() {

    int i = 0;
    QString strSerialNum = snLineEdit->text();
    int nSerialNum_Length = strSerialNum.size()*2;

    unsigned char *pdata = new unsigned char[nSerialNum_Length];

    QChar *chSerialNum = strSerialNum.data();

    for(i=0;i<nSerialNum_Length/2;i++) {

        pdata[i*2+1] = (chSerialNum+i)->unicode()/256;
        pdata[i*2] = (chSerialNum+i)->unicode()%256;
    }

    if(ETronDI_OK != EtronDI_SetSerialNumber(m_pMainWindow->m_pEtronDI, &m_DevSelInfo, pdata, nSerialNum_Length))
      QMessageBox::critical(NULL, "Error", "Set Serial Number failed .", QMessageBox::Yes , QMessageBox::Yes);
    else
      QMessageBox::information(NULL, "Info", "Set Serial Number Complete !!", QMessageBox::Yes, QMessageBox::Yes);

    if(pdata) delete pdata;
    pdata = NULL;
}

void CVideoDeviceDlg::on_FLASH_Read_Click() {
	
	m_funcSelect = READ_FLASH;
	int currentDataType = dataTypeComboBox->currentIndex();
	if(currentDataType == 0) m_FlashDataType = Total;
	if(currentDataType == 1) m_FlashDataType = FW_PLUGIN;
	if(currentDataType == 2) m_FlashDataType = BOOTLOADER_ONLY;
	if(currentDataType == 3) m_FlashDataType = FW_ONLY;
	if(currentDataType == 4) m_FlashDataType = PLUGIN_ONLY;
	
	if(m_pVideoDeviceThread == NULL) {
		m_pVideoDeviceThread = new CVideoDeviceThread(this);
		connect(m_pVideoDeviceThread, &CVideoDeviceThread::done, this, &CVideoDeviceDlg::threadDone);
		m_time.start();
		m_pVideoDeviceThread->start();
	}
}

void CVideoDeviceDlg::on_FLASH_Write_Click() {
	
	m_funcSelect = WRITE_FLASH;
	int currentDataType = dataTypeComboBox->currentIndex();
	if(currentDataType == 0) m_FlashDataType = Total;
	if(currentDataType == 1) m_FlashDataType = FW_PLUGIN;
	if(currentDataType == 2) m_FlashDataType = BOOTLOADER_ONLY;
	if(currentDataType == 3) m_FlashDataType = FW_ONLY;
	if(currentDataType == 4) m_FlashDataType = PLUGIN_ONLY;

    m_KeepDataCtrl.bIsSerialNumberKeep       = snChkBox->isChecked();
    m_KeepDataCtrl.bIsSensorPositionKeep     = offsetChkBox->isChecked();
    m_KeepDataCtrl.bIsRectificationTableKeep = rectificationChkBox->isChecked();
    m_KeepDataCtrl.bIsZDTableKeep            = zdChkBox->isChecked();
    m_KeepDataCtrl.bIsCalibrationLogKeep     = calibrationLogChkBox->isChecked();
	
	if(m_pVideoDeviceThread == NULL) {
		m_pVideoDeviceThread = new CVideoDeviceThread(this);
		connect(m_pVideoDeviceThread, &CVideoDeviceThread::done, this, &CVideoDeviceDlg::threadDone);
		m_time.start();
		m_pVideoDeviceThread->start();
	}
}

void CVideoDeviceDlg::threadDone(bool bIsSuccess){

	if(bIsSuccess) {
		if(m_funcSelect == READ_FLASH) {
			QString strDbg;
			strDbg = QString("Read flash success , %1 sec").arg(m_time.elapsed()/1000.0);
			QMessageBox::information(NULL, "Info", strDbg, QMessageBox::Yes, QMessageBox::Yes);	
		}

        if(m_funcSelect == WRITE_FLASH) {
            QString strDbg;
            strDbg = QString("Write flash success , %1 sec").arg(m_time.elapsed()/1000.0);
            QMessageBox::information(NULL, "Info", strDbg, QMessageBox::Yes, QMessageBox::Yes);
        }
	}
	else {
        if(m_funcSelect == READ_FLASH)  QMessageBox::critical(NULL, "Error", "Read flash failed .",  QMessageBox::Yes , QMessageBox::Yes);
        if(m_funcSelect == WRITE_FLASH) QMessageBox::critical(NULL, "Error", "Write flash failed .", QMessageBox::Yes , QMessageBox::Yes);
	}
	
	delete m_pVideoDeviceThread;
	m_pVideoDeviceThread = NULL;
}

void CVideoDeviceDlg::on_PreviewBtn_Click() {
	
    if(depthOutputCtrlCombo->currentIndex() == 0) m_Dtc = DEPTH_IMG_NON_TRANSFER;
    else if(depthOutputCtrlCombo->currentIndex() == 1) m_Dtc = DEPTH_IMG_GRAY_TRANSFER;
    else m_Dtc = DEPTH_IMG_COLORFUL_TRANSFER;

    m_nFrameRate = (fpsLineEdit->text()).toInt();

	if( ( colorStreamChk->isChecked() == true ) && ( depthStreamChk->isChecked() == false ) ) {
		
		int nRet = EtronDI_OpenDevice2( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, 
										m_pStreamColorInfo[m_nColorResIndex].nWidth,
                                        m_pStreamColorInfo[m_nColorResIndex].nHeight,
                                        m_pStreamColorInfo[m_nColorResIndex].bFormatMJPG,
                                        0, 0, m_Dtc, false, NULL, &m_nFrameRate);
        //qDebug("Frame rate = %d", m_nFrameRate);
        fpsLineEdit->setText(QString::number(m_nFrameRate));
		
		if(nRet == ETronDI_OK) {
			CColorDlg *m_pColorDlg = new CColorDlg(m_DevSelInfo.index, this);
			m_pColorDlg->setAttribute(Qt::WA_DeleteOnClose);
			m_pColorDlg->setWindowTitle(tr("ColorImg"));
			m_pColorDlg->show();
		}
	}

    if( ( colorStreamChk->isChecked() == false ) && ( depthStreamChk->isChecked() == true ) ) {

        int nRet = EtronDI_OpenDevice2( m_pMainWindow->m_pEtronDI, &m_DevSelInfo,
                                        0, 0, false,
                                        m_pStreamDepthInfo[m_nDepthResIndex].nWidth,
                                        m_pStreamDepthInfo[m_nDepthResIndex].nHeight,
                                        m_Dtc,
                                        false, NULL, &m_nFrameRate);

        fpsLineEdit->setText(QString::number(m_nFrameRate));

        if(nRet == ETronDI_OK) {
            CDepthDlg *m_pDepthDlg = new CDepthDlg(m_DevSelInfo.index, this);
            m_pDepthDlg->setAttribute(Qt::WA_DeleteOnClose);
            m_pDepthDlg->setWindowTitle(tr("DepthImg"));
            m_pDepthDlg->show();
        }
    }

    if( ( colorStreamChk->isChecked() == true ) && ( depthStreamChk->isChecked() == true ) ) {

        int nRet = EtronDI_OpenDevice2( m_pMainWindow->m_pEtronDI, &m_DevSelInfo,
                                        m_pStreamColorInfo[m_nColorResIndex].nWidth,
                                        m_pStreamColorInfo[m_nColorResIndex].nHeight,
                                        m_pStreamColorInfo[m_nColorResIndex].bFormatMJPG,
                                        m_pStreamDepthInfo[m_nDepthResIndex].nWidth,
                                        m_pStreamDepthInfo[m_nDepthResIndex].nHeight,
                                        m_Dtc,
                                        false, NULL, &m_nFrameRate);

        fpsLineEdit->setText(QString::number(m_nFrameRate));

        if(nRet == ETronDI_OK) {
            CBothDlg *m_pBothDlg = new CBothDlg(m_DevSelInfo.index, this);
            m_pBothDlg->setAttribute(Qt::WA_DeleteOnClose);
            m_pBothDlg->setWindowTitle(tr("BothImg"));
            m_pBothDlg->show();
        }
    }
}

void CVideoDeviceDlg::on_m_ColorRes_Combobox_currentIndexChanged(int index) {
    m_nColorResIndex = index;
}

void CVideoDeviceDlg::on_m_DepthRes_Combobox_currentIndexChanged(int index) {
    m_nDepthResIndex = index;
}

void CVideoDeviceDlg::on_m_DepthDataType_Combobox_currentIndexChanged(int index) {

    if(index==0)
         m_nDepthDataType = 4; // data type : 11 bits, the setting value is firmware dependent
    else m_nDepthDataType = 2; // data type : 14 bits, the setting value is firmware dependent

    EtronDI_SetDepthDataType(m_pMainWindow->m_pEtronDI, &m_DevSelInfo, m_nDepthDataType);

    //test
    unsigned short nVal;
     EtronDI_GetDepthDataType(m_pMainWindow->m_pEtronDI, &m_DevSelInfo, &nVal);

}

void CVideoDeviceDlg::getFWVersion() {
	
	char szBuf[256];
    int  nActualLength;

    if(ETronDI_OK == EtronDI_GetFwVersion( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, szBuf, 256, &nActualLength)) {
		FW_Version_Label->setText(szBuf);
	}
}

void CVideoDeviceDlg::getResolutionList() {

    colorStreamCombo->clear();
    depthStreamCombo->clear();

	memset(m_pStreamColorInfo, 0, sizeof(ETRONDI_STREAM_INFO)*64);
    memset(m_pStreamDepthInfo, 0, sizeof(ETRONDI_STREAM_INFO)*64);

    EtronDI_GetDeviceResolutionList(m_pMainWindow->m_pEtronDI, &m_DevSelInfo, 64, m_pStreamColorInfo, 64, m_pStreamDepthInfo);
        
    QString colorStr;
    QString depthStr;
    
    PETRONDI_STREAM_INFO pStreamTempInfo = m_pStreamColorInfo;
    
    int i = 0;
    while(i<64) {

        if(pStreamTempInfo->bFormatMJPG) {
            if(pStreamTempInfo->nWidth != 0) {
                colorStr.sprintf(" [ %4d x %4d ] MJPG", (pStreamTempInfo)->nWidth, (pStreamTempInfo)->nHeight);
                colorStreamCombo->addItem(colorStr);
            }
        }
        else {
            if(pStreamTempInfo->nWidth != 0) {
                colorStr.sprintf(" [ %4d x %4d ] YUYV", (pStreamTempInfo)->nWidth, (pStreamTempInfo)->nHeight);
                colorStreamCombo->addItem(colorStr);
            }
        }
        pStreamTempInfo++;
        i++;
    }

    pStreamTempInfo = m_pStreamDepthInfo;
    i = 0;

    while(i<64) {

        if(pStreamTempInfo->bFormatMJPG) {
            if(pStreamTempInfo->nWidth != 0) {
                depthStr.sprintf(" [ %4d x %4d ] MJPG", (pStreamTempInfo)->nWidth, (pStreamTempInfo)->nHeight);
                depthStreamCombo->addItem(depthStr);
            }
        }
        else {
            if(pStreamTempInfo->nWidth != 0) {
                depthStr.sprintf(" [ %4d x %4d ] YUYV", (pStreamTempInfo)->nWidth, (pStreamTempInfo)->nHeight);
                depthStreamCombo->addItem(depthStr);
            }
        }
        pStreamTempInfo++;
        i++;
    }
}

/*
void CVideoDeviceDlg::getAEAWBStatus() {

    AE_STATUS aeStatus;
    AWB_STATUS awbStatus;

    if( ETronDI_OK == EtronDI_GetAEStatus(m_pMainWindow->m_pEtronDI, &m_DevSelInfo, &aeStatus)) {
        if(aeStatus == AE_ENABLE) aeEnableChk->setChecked(true);
        else aeEnableChk->setChecked(false);
    }

    if( ETronDI_OK == EtronDI_GetAWBStatus(m_pMainWindow->m_pEtronDI, &m_DevSelInfo, &awbStatus)) {
        if(awbStatus == AWB_ENABLE) awbEnableChk->setChecked(true);
        else awbEnableChk->setChecked(false);
    }
}
*/

void CVideoDeviceDlg::initPropertyItemList() {

    QStringList strList;

    //long int nMax = 0;
    //long int nMin = 0;
    //long int nStep = 0;
    //long int nDefault = 0;
    //long int nFlags = 0;
    
    int i = 0;
    
    // ct    
    strcpy(m_pPropertyItemArray[0].szPropName, "[CT]Auto-Exposure Mode Control");
    m_pPropertyItemArray[0].nPropType = PROP_TYPE_CT;
    m_pPropertyItemArray[0].nPropID = CT_PROPERTY_ID_AUTO_EXPOSURE_MODE_CTRL;
    
    strcpy(m_pPropertyItemArray[1].szPropName, "[CT]Auto-Exposure Priority Control");
    m_pPropertyItemArray[1].nPropType = PROP_TYPE_CT;
    m_pPropertyItemArray[1].nPropID = CT_PROPERTY_ID_AUTO_EXPOSURE_PRIORITY_CTRL;
    
    strcpy(m_pPropertyItemArray[2].szPropName, "[CT]Exposure Time(Absolute) Control");
    m_pPropertyItemArray[2].nPropType = PROP_TYPE_CT;
    m_pPropertyItemArray[2].nPropID = CT_PROPERTY_ID_EXPOSURE_TIME_ABSOLUTE_CTRL;
    
    strcpy(m_pPropertyItemArray[3].szPropName, "[CT]Exposure Time(Relative) Control");
    m_pPropertyItemArray[3].nPropType = PROP_TYPE_CT;
    m_pPropertyItemArray[3].nPropID = CT_PROPERTY_ID_EXPOSURE_TIME_RELATIVE_CTRL;
    
    strcpy(m_pPropertyItemArray[4].szPropName, "[CT]Focus(Absolute) Control");
    m_pPropertyItemArray[4].nPropType = PROP_TYPE_CT;
    m_pPropertyItemArray[4].nPropID = CT_PROPERTY_ID_FOCUS_ABSOLUTE_CTRL;
    
    strcpy(m_pPropertyItemArray[5].szPropName, "[CT]Focus(Relative) Control");
    m_pPropertyItemArray[5].nPropType = PROP_TYPE_CT;
    m_pPropertyItemArray[5].nPropID = CT_PROPERTY_ID_FOCUS_RELATIVE_CTRL;
    
    strcpy(m_pPropertyItemArray[6].szPropName, "[CT]Focus Auto Control");
    m_pPropertyItemArray[6].nPropType = PROP_TYPE_CT;
    m_pPropertyItemArray[6].nPropID = CT_PROPERTY_ID_FOCUS_AUTO_CTRL;
    
    strcpy(m_pPropertyItemArray[7].szPropName, "[CT]Iris(Absolute) Control");
    m_pPropertyItemArray[7].nPropType = PROP_TYPE_CT;
    m_pPropertyItemArray[7].nPropID = CT_PROPERTY_ID_IRIS_ABSOLUTE_CTRL;
    
    strcpy(m_pPropertyItemArray[8].szPropName, "[CT]Iris(Relative) Control");
    m_pPropertyItemArray[8].nPropType = PROP_TYPE_CT;
    m_pPropertyItemArray[8].nPropID = CT_PROPERTY_ID_IRIS_RELATIVE_CTRL;
    
    strcpy(m_pPropertyItemArray[9].szPropName, "[CT]Zoom(Absolute) Control");
    m_pPropertyItemArray[9].nPropType = PROP_TYPE_CT;
    m_pPropertyItemArray[9].nPropID = CT_PROPERTY_ID_ZOOM_ABSOLUTE_CTRL;
    
    strcpy(m_pPropertyItemArray[10].szPropName, "[CT]Zoom(Relative) Control");
    m_pPropertyItemArray[10].nPropType = PROP_TYPE_CT;
    m_pPropertyItemArray[10].nPropID = CT_PROPERTY_ID_ZOOM_RELATIVE_CTRL;
    
    strcpy(m_pPropertyItemArray[11].szPropName, "[CT]PAN(Absolute) Control");
    m_pPropertyItemArray[11].nPropType = PROP_TYPE_CT;
    m_pPropertyItemArray[11].nPropID = CT_PROPERTY_ID_PAN_ABSOLUTE_CTRL;
    
    strcpy(m_pPropertyItemArray[12].szPropName, "[CT]PAN(Relative) Control");
    m_pPropertyItemArray[12].nPropType = PROP_TYPE_CT;
    m_pPropertyItemArray[12].nPropID = CT_PROPERTY_ID_PAN_RELATIVE_CTRL;
    
    strcpy(m_pPropertyItemArray[13].szPropName, "[CT]TILT(Absolute) Control");
    m_pPropertyItemArray[13].nPropType = PROP_TYPE_CT;
    m_pPropertyItemArray[13].nPropID = CT_PROPERTY_ID_TILT_ABSOLUTE_CTRL;
    
    strcpy(m_pPropertyItemArray[14].szPropName, "[CT]TILT(Relative) Control");
    m_pPropertyItemArray[14].nPropType = PROP_TYPE_CT;
    m_pPropertyItemArray[14].nPropID = CT_PROPERTY_ID_TILT_RELATIVE_CTRL;
    
    strcpy(m_pPropertyItemArray[15].szPropName, "[CT]PRIVACY Control");
    m_pPropertyItemArray[15].nPropType = PROP_TYPE_CT;
    m_pPropertyItemArray[15].nPropID = CT_PROPERTY_ID_PRIVACY_CTRL;
    
    // pu    
    strcpy(m_pPropertyItemArray[16].szPropName, "[PU]Backlight Compensation Control");
    m_pPropertyItemArray[16].nPropType = PROP_TYPE_PU;
    m_pPropertyItemArray[16].nPropID = PU_PROPERTY_ID_BACKLIGHT_COMPENSATION_CTRL;
    
    strcpy(m_pPropertyItemArray[17].szPropName, "[PU]Brightness Control");
    m_pPropertyItemArray[17].nPropType = PROP_TYPE_PU;
    m_pPropertyItemArray[17].nPropID = PU_PROPERTY_ID_BRIGHTNESS_CTRL;
    
    strcpy(m_pPropertyItemArray[18].szPropName, "[PU]Contrast Control");
    m_pPropertyItemArray[18].nPropType = PROP_TYPE_PU;
    m_pPropertyItemArray[18].nPropID = PU_PROPERTY_ID_CONTRAST_CTRL;
    
    strcpy(m_pPropertyItemArray[19].szPropName, "[PU]Gain Control");
    m_pPropertyItemArray[19].nPropType = PROP_TYPE_PU;
    m_pPropertyItemArray[19].nPropID = PU_PROPERTY_ID_GAIN_CTRL;
    
    strcpy(m_pPropertyItemArray[20].szPropName, "[PU]Power Line Freguence Control");
    m_pPropertyItemArray[20].nPropType = PROP_TYPE_PU;
    m_pPropertyItemArray[20].nPropID = PU_PROPERTY_ID_POWER_LINE_FREQUENCY_CTRL;
    
    strcpy(m_pPropertyItemArray[21].szPropName, "[PU]HUE Control");
    m_pPropertyItemArray[21].nPropType = PROP_TYPE_PU;
    m_pPropertyItemArray[21].nPropID = PU_PROPERTY_ID_HUE_CTRL;
    
    strcpy(m_pPropertyItemArray[22].szPropName, "[PU]HUE Auto Control");
    m_pPropertyItemArray[22].nPropType = PROP_TYPE_PU;
    m_pPropertyItemArray[22].nPropID = PU_PROPERTY_ID_HUE_AUTO_CTRL;
    
    strcpy(m_pPropertyItemArray[23].szPropName, "[PU]Saturation Control");
    m_pPropertyItemArray[23].nPropType = PROP_TYPE_PU;
    m_pPropertyItemArray[23].nPropID = PU_PROPERTY_ID_SATURATION_CTRL;
    
    strcpy(m_pPropertyItemArray[24].szPropName, "[PU]Sharpness Control");
    m_pPropertyItemArray[24].nPropType = PROP_TYPE_PU;
    m_pPropertyItemArray[24].nPropID = PU_PROPERTY_ID_SHARPNESS_CTRL;
    
    strcpy(m_pPropertyItemArray[25].szPropName, "[PU]Gamma Control");
    m_pPropertyItemArray[25].nPropType = PROP_TYPE_PU;
    m_pPropertyItemArray[25].nPropID = PU_PROPERTY_ID_GAMMA_CTRL;
    
    strcpy(m_pPropertyItemArray[26].szPropName, "[PU]White Balance Control");
    m_pPropertyItemArray[26].nPropType = PROP_TYPE_PU;
    m_pPropertyItemArray[26].nPropID = PU_PROPERTY_ID_WHITE_BALANCE_CTRL;
    
    strcpy(m_pPropertyItemArray[27].szPropName, "[PU]White Balance Auto Control");
    m_pPropertyItemArray[27].nPropType = PROP_TYPE_PU;
    m_pPropertyItemArray[27].nPropID = PU_PROPERTY_ID_WHITE_BALANCE_AUTO_CTRL;

	for(i=0;i<PROP_TOTAL_NUMBER;i++) {
		if(i<16) {
            //if( ETronDI_OK == EtronDI_GetCTRangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, m_pPropertyItemArray[i].nPropID, &nMax, &nMin, &nStep, &nDefault, &nFlags)) {
				strList << m_pPropertyItemArray[i].szPropName;
            //}
		}
		else {
            //if( ETronDI_OK == EtronDI_GetPURangeAndStep( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, m_pPropertyItemArray[i].nPropID, &nMax, &nMin, &nStep, &nDefault, &nFlags)) {
				strList << m_pPropertyItemArray[i].szPropName;
            //}
		}
	}

    propertyCombo->clear();
    propertyCombo->addItems(strList);
}

void CVideoDeviceDlg::on_REG_Read_Click() {

    unsigned short nAddr  = addressLineEdit->text().toInt(NULL, 16);

    unsigned short nValue = 0;

    int flag = 0;

    if(addrSizeChk->isChecked()) flag |= FG_Address_2Byte;
    else flag |= FG_Address_1Byte;

    if(calueSizeChk->isChecked()) flag |= FG_Value_2Byte;
    else flag |= FG_Value_1Byte;

    if(i2CRadioBtn->isChecked()) {

        int nId = slaveIDLineEdit->text().toInt(NULL, 16);;
        SENSORMODE_INFO SensorMode = SENSOR_BOTH;

        EtronDI_GetSensorRegister( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, nId, nAddr, &nValue, flag, SensorMode);
        valueLineEdit->setText(QString::number(nValue, 16));
    }

    if(hwRadioBtn->isChecked()) {

        EtronDI_GetHWRegister( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, nAddr, &nValue, flag);
        valueLineEdit->setText(QString::number(nValue, 16));
    }

    if(fwRadioBtn->isChecked()) {

        EtronDI_GetFWRegister( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, nAddr, &nValue, flag);
        valueLineEdit->setText(QString::number(nValue, 16));
    }
}

void CVideoDeviceDlg::on_REG_Write_Click() {

    unsigned short nAddr  = addressLineEdit->text().toInt(NULL, 16);

    unsigned short nValue = valueLineEdit->text().toInt(NULL, 16);

    int flag = 0;

    if(addrSizeChk->isChecked()) flag |= FG_Address_2Byte;
    else flag |= FG_Address_1Byte;

    if(calueSizeChk->isChecked()) flag |= FG_Value_2Byte;
    else flag |= FG_Value_1Byte;

    if(i2CRadioBtn->isChecked()) {

        int nId = slaveIDLineEdit->text().toInt(NULL, 16);;
        SENSORMODE_INFO SensorMode = SENSOR_BOTH;

        if( ETronDI_OK == EtronDI_SetSensorRegister( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, nId, nAddr, nValue, flag, SensorMode)) {
            qDebug("EtronDI_SetSensorRegister failed");
        }
    }

    if(hwRadioBtn->isChecked()) {

        if( ETronDI_OK == EtronDI_SetHWRegister( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, nAddr, nValue, flag)) {
            qDebug("EtronDI_SetHWRegister failed");
        }
    }

    if(fwRadioBtn->isChecked()) {

        if( ETronDI_OK == EtronDI_SetFWRegister( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, nAddr, nValue, flag)) {
            qDebug("EtronDI_SetFWRegister failed");
        }
    }
}

void CVideoDeviceDlg::on_AE_State_Change(bool bIsChecked) {

    if(bIsChecked) {
        EtronDI_EnableAE( m_pMainWindow->m_pEtronDI, &m_DevSelInfo);
    }
    else {
        EtronDI_DisableAE( m_pMainWindow->m_pEtronDI, &m_DevSelInfo);
    }
}

void CVideoDeviceDlg::on_AWB_State_Change(bool bIsChecked) {

    if(bIsChecked) {
        EtronDI_EnableAWB( m_pMainWindow->m_pEtronDI, &m_DevSelInfo);
    }
    else {
        EtronDI_DisableAWB( m_pMainWindow->m_pEtronDI, &m_DevSelInfo);
    }
}

void CVideoDeviceDlg::on_AE_AWB_State_update() {
	
	// get current AE and AWB state
	//getAEAWBStatus();
}

void CVideoDeviceDlg::on_GPIO1_BIT7_Click() {

    unsigned short nGPIO1Value = GPIO1LineEdit->text().toUShort(0,16);

    if( !BIT_CHECK(nGPIO1Value, 7)) {
        BIT_SET(nGPIO1Value, 7);
        b7Btn->setChecked(true);
    }
    else {
        BIT_CLEAR(nGPIO1Value, 7);
        b7Btn->setChecked(false);
    }

    GPIO1LineEdit->setText(QString::number(nGPIO1Value, 16));
}

void CVideoDeviceDlg::on_GPIO1_BIT6_Click() {

    unsigned short nGPIO1Value = GPIO1LineEdit->text().toUShort(0,16);

    if( !BIT_CHECK(nGPIO1Value, 6)) {
        BIT_SET(nGPIO1Value, 6);
        b6Btn->setChecked(true);
    }
    else {
        BIT_CLEAR(nGPIO1Value, 6);
        b6Btn->setChecked(false);
    }

    GPIO1LineEdit->setText(QString::number(nGPIO1Value, 16));

}

void CVideoDeviceDlg::on_GPIO1_BIT5_Click() {

    unsigned short nGPIO1Value = GPIO1LineEdit->text().toUShort(0,16);

    if( !BIT_CHECK(nGPIO1Value, 5)) {
        BIT_SET(nGPIO1Value, 5);
        b5Btn->setChecked(true);
    }
    else {
        BIT_CLEAR(nGPIO1Value, 5);
        b5Btn->setChecked(false);
    }

    GPIO1LineEdit->setText(QString::number(nGPIO1Value, 16));
}

void CVideoDeviceDlg::on_GPIO1_BIT4_Click() {

    unsigned short nGPIO1Value = GPIO1LineEdit->text().toUShort(0,16);

    if( !BIT_CHECK(nGPIO1Value, 4)) {
        BIT_SET(nGPIO1Value, 4);
        b4Btn->setChecked(true);
    }
    else {
        BIT_CLEAR(nGPIO1Value, 4);
        b4Btn->setChecked(false);
    }

    GPIO1LineEdit->setText(QString::number(nGPIO1Value, 16));
}

void CVideoDeviceDlg::on_GPIO1_BIT3_Click() {

    unsigned short nGPIO1Value = GPIO1LineEdit->text().toUShort(0,16);

    if( !BIT_CHECK(nGPIO1Value, 3)) {
        BIT_SET(nGPIO1Value, 3);
        b3Btn->setChecked(true);
    }
    else {
        BIT_CLEAR(nGPIO1Value, 3);
        b3Btn->setChecked(false);
    }

    GPIO1LineEdit->setText(QString::number(nGPIO1Value, 16));
}

void CVideoDeviceDlg::on_GPIO1_BIT2_Click() {

    unsigned short nGPIO1Value = GPIO1LineEdit->text().toUShort(0,16);

    if( !BIT_CHECK(nGPIO1Value, 2)) {
        BIT_SET(nGPIO1Value, 2);
        b2Btn->setChecked(true);
    }
    else {
        BIT_CLEAR(nGPIO1Value, 2);
        b2Btn->setChecked(false);
    }

    GPIO1LineEdit->setText(QString::number(nGPIO1Value, 16));
}

void CVideoDeviceDlg::on_GPIO1_BIT1_Click() {

    unsigned short nGPIO1Value = GPIO1LineEdit->text().toUShort(0,16);

    if( !BIT_CHECK(nGPIO1Value, 1)) {
        BIT_SET(nGPIO1Value, 1);
        b1Btn->setChecked(true);
    }
    else {
        BIT_CLEAR(nGPIO1Value, 1);
        b1Btn->setChecked(false);
    }

    GPIO1LineEdit->setText(QString::number(nGPIO1Value, 16));
}

void CVideoDeviceDlg::on_GPIO1_BIT0_Click() {

    unsigned short nGPIO1Value = GPIO1LineEdit->text().toUShort(0,16);

    if( !BIT_CHECK(nGPIO1Value, 0)) {
        BIT_SET(nGPIO1Value, 0);
        b0Btn->setChecked(true);
    }
    else {
        BIT_CLEAR(nGPIO1Value, 0);
        b0Btn->setChecked(false);
    }

    GPIO1LineEdit->setText(QString::number(nGPIO1Value, 16));
}

void CVideoDeviceDlg::on_GPIO2_BIT7_Click() {

    unsigned short nGPIO2Value = GPIO2LineEdit->text().toUShort(0,16);

    if( !BIT_CHECK(nGPIO2Value, 7)) {
        BIT_SET(nGPIO2Value, 7);
        b7Btn_2->setChecked(true);
    }
    else {
        BIT_CLEAR(nGPIO2Value, 7);
        b7Btn_2->setChecked(false);
    }

    GPIO2LineEdit->setText(QString::number(nGPIO2Value, 16));
}

void CVideoDeviceDlg::on_GPIO2_BIT6_Click() {

    unsigned short nGPIO2Value = GPIO2LineEdit->text().toUShort(0,16);

    if( !BIT_CHECK(nGPIO2Value, 6)) {
        BIT_SET(nGPIO2Value, 6);
        b6Btn_2->setChecked(true);
    }
    else {
        BIT_CLEAR(nGPIO2Value, 6);
        b6Btn_2->setChecked(false);
    }

    GPIO2LineEdit->setText(QString::number(nGPIO2Value, 16));
}

void CVideoDeviceDlg::on_GPIO2_BIT5_Click() {

    unsigned short nGPIO2Value = GPIO2LineEdit->text().toUShort(0,16);

    if( !BIT_CHECK(nGPIO2Value, 5)) {
        BIT_SET(nGPIO2Value, 5);
        b5Btn_2->setChecked(true);
    }
    else {
        BIT_CLEAR(nGPIO2Value, 5);
        b5Btn_2->setChecked(false);
    }

    GPIO2LineEdit->setText(QString::number(nGPIO2Value, 16));
}

void CVideoDeviceDlg::on_GPIO2_BIT4_Click() {

    unsigned short nGPIO2Value = GPIO2LineEdit->text().toUShort(0,16);

    if( !BIT_CHECK(nGPIO2Value, 4)) {
        BIT_SET(nGPIO2Value, 4);
        b4Btn_2->setChecked(true);
    }
    else {
        BIT_CLEAR(nGPIO2Value, 4);
        b4Btn_2->setChecked(false);
    }

    GPIO2LineEdit->setText(QString::number(nGPIO2Value, 16));
}

void CVideoDeviceDlg::on_GPIO2_BIT3_Click() {

    unsigned short nGPIO2Value = GPIO2LineEdit->text().toUShort(0,16);

    if( !BIT_CHECK(nGPIO2Value, 3)) {
        BIT_SET(nGPIO2Value, 3);
        b3Btn_2->setChecked(true);
    }
    else {
        BIT_CLEAR(nGPIO2Value, 3);
        b3Btn_2->setChecked(false);
    }

    GPIO2LineEdit->setText(QString::number(nGPIO2Value, 16));
}

void CVideoDeviceDlg::on_GPIO2_BIT2_Click() {

    unsigned short nGPIO2Value = GPIO2LineEdit->text().toUShort(0,16);

    if( !BIT_CHECK(nGPIO2Value, 2)) {
        BIT_SET(nGPIO2Value, 2);
        b2Btn_2->setChecked(true);
    }
    else {
        BIT_CLEAR(nGPIO2Value, 2);
        b2Btn_2->setChecked(false);
    }

    GPIO2LineEdit->setText(QString::number(nGPIO2Value, 16));
}

void CVideoDeviceDlg::on_GPIO2_BIT1_Click() {

    unsigned short nGPIO2Value = GPIO2LineEdit->text().toUShort(0,16);

    if( !BIT_CHECK(nGPIO2Value, 1)) {
        BIT_SET(nGPIO2Value, 1);
        b1Btn_2->setChecked(true);
    }
    else {
        BIT_CLEAR(nGPIO2Value, 1);
        b1Btn_2->setChecked(false);
    }

    GPIO2LineEdit->setText(QString::number(nGPIO2Value, 16));
}

void CVideoDeviceDlg::on_GPIO2_BIT0_Click() {

    unsigned short nGPIO2Value = GPIO2LineEdit->text().toUShort(0,16);

    if( !BIT_CHECK(nGPIO2Value, 0)) {
        BIT_SET(nGPIO2Value, 0);
        b0Btn_2->setChecked(true);
    }
    else {
        BIT_CLEAR(nGPIO2Value, 0);
        b0Btn_2->setChecked(false);
    }

    GPIO2LineEdit->setText(QString::number(nGPIO2Value, 16));
}

void CVideoDeviceDlg::on_GPIO1Value_Read() {

    BYTE nValue = 0;
    EtronDI_GetGPIOValue( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, 1, &nValue);

    if( BIT_CHECK(nValue, 7)) b7Btn->setChecked(true);
    else b7Btn->setChecked(false);

    if( BIT_CHECK(nValue, 6)) b6Btn->setChecked(true);
    else b6Btn->setChecked(false);

    if( BIT_CHECK(nValue, 5)) b5Btn->setChecked(true);
    else b5Btn->setChecked(false);

    if( BIT_CHECK(nValue, 4)) b4Btn->setChecked(true);
    else b4Btn->setChecked(false);

    if( BIT_CHECK(nValue, 3)) b3Btn->setChecked(true);
    else b3Btn->setChecked(false);

    if( BIT_CHECK(nValue, 2)) b2Btn->setChecked(true);
    else b2Btn->setChecked(false);

    if( BIT_CHECK(nValue, 1)) b1Btn->setChecked(true);
    else b1Btn->setChecked(false);

    if( BIT_CHECK(nValue, 0)) b0Btn->setChecked(true);
    else b0Btn->setChecked(false);

    GPIO1LineEdit->setText(QString::number(nValue, 16));
}

void CVideoDeviceDlg::on_GPIO1Value_Write() {

    BYTE nValue = GPIO1LineEdit->text().toInt(0, 16);

    EtronDI_SetGPIOValue( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, 1, nValue);

    if( BIT_CHECK(nValue, 7)) b7Btn->setChecked(true);
    else b7Btn->setChecked(false);

    if( BIT_CHECK(nValue, 6)) b6Btn->setChecked(true);
    else b6Btn->setChecked(false);

    if( BIT_CHECK(nValue, 5)) b5Btn->setChecked(true);
    else b5Btn->setChecked(false);

    if( BIT_CHECK(nValue, 4)) b4Btn->setChecked(true);
    else b4Btn->setChecked(false);

    if( BIT_CHECK(nValue, 3)) b3Btn->setChecked(true);
    else b3Btn->setChecked(false);

    if( BIT_CHECK(nValue, 2)) b2Btn->setChecked(true);
    else b2Btn->setChecked(false);

    if( BIT_CHECK(nValue, 1)) b1Btn->setChecked(true);
    else b1Btn->setChecked(false);

    if( BIT_CHECK(nValue, 0)) b0Btn->setChecked(true);
    else b0Btn->setChecked(false);
}

void CVideoDeviceDlg::on_GPIO2Value_Read() {

    BYTE nValue = 0;
    EtronDI_GetGPIOValue( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, 2, &nValue);

    if( BIT_CHECK(nValue, 7)) b7Btn_2->setChecked(true);
    else b7Btn_2->setChecked(false);

    if( BIT_CHECK(nValue, 6)) b6Btn_2->setChecked(true);
    else b6Btn_2->setChecked(false);

    if( BIT_CHECK(nValue, 5)) b5Btn_2->setChecked(true);
    else b5Btn_2->setChecked(false);

    if( BIT_CHECK(nValue, 4)) b4Btn_2->setChecked(true);
    else b4Btn_2->setChecked(false);

    if( BIT_CHECK(nValue, 3)) b3Btn_2->setChecked(true);
    else b3Btn_2->setChecked(false);

    if( BIT_CHECK(nValue, 2)) b2Btn_2->setChecked(true);
    else b2Btn_2->setChecked(false);

    if( BIT_CHECK(nValue, 1)) b1Btn_2->setChecked(true);
    else b1Btn_2->setChecked(false);

    if( BIT_CHECK(nValue, 0)) b0Btn_2->setChecked(true);
    else b0Btn_2->setChecked(false);

    GPIO2LineEdit->setText(QString::number(nValue, 16));
}

void CVideoDeviceDlg::on_GPIO2Value_Write() {

    BYTE nValue = GPIO2LineEdit->text().toInt(0, 16);

    EtronDI_SetGPIOValue( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, 2, nValue);

    if( BIT_CHECK(nValue, 7)) b7Btn_2->setChecked(true);
    else b7Btn_2->setChecked(false);

    if( BIT_CHECK(nValue, 6)) b6Btn_2->setChecked(true);
    else b6Btn_2->setChecked(false);

    if( BIT_CHECK(nValue, 5)) b5Btn_2->setChecked(true);
    else b5Btn_2->setChecked(false);

    if( BIT_CHECK(nValue, 4)) b4Btn_2->setChecked(true);
    else b4Btn_2->setChecked(false);

    if( BIT_CHECK(nValue, 3)) b3Btn_2->setChecked(true);
    else b3Btn_2->setChecked(false);

    if( BIT_CHECK(nValue, 2)) b2Btn_2->setChecked(true);
    else b2Btn_2->setChecked(false);

    if( BIT_CHECK(nValue, 1)) b1Btn_2->setChecked(true);
    else b1Btn_2->setChecked(false);

    if( BIT_CHECK(nValue, 0)) b0Btn_2->setChecked(true);
    else b0Btn_2->setChecked(false);
}

void CVideoDeviceDlg::on_Read_Property_Click() {

    long int nValue = 0;

    QString str = propertyCombo->currentText();

	for(int i=0;i<PROP_TOTAL_NUMBER;i++) {
		if( 0 == QString::compare( str, m_pPropertyItemArray[i].szPropName, Qt::CaseSensitive)) {
			if(i<16) {
				EtronDI_GetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, m_pPropertyItemArray[i].nPropID, &nValue);
				break;
			}
			else {
				EtronDI_GetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, m_pPropertyItemArray[i].nPropID, &nValue);
				break;
			}
		}
	}

    propertyLineEdit->setText(QString::number(nValue));
}

void CVideoDeviceDlg::on_Write_Property_Click() {

    long int nValue = propertyLineEdit->text().toInt();

    QString str = propertyCombo->currentText();

    for(int i=0;i<PROP_TOTAL_NUMBER;i++) {
		if( 0 == QString::compare( str, m_pPropertyItemArray[i].szPropName, Qt::CaseSensitive)) {
			if(i<16) {
				EtronDI_SetCTPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, m_pPropertyItemArray[i].nPropID, nValue);
				break;
			}
			else {
				EtronDI_SetPUPropVal( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, m_pPropertyItemArray[i].nPropID, nValue);
				break;
			}
		}
	}
}

void CVideoDeviceDlg::on_Prop_Items_Click() {
    CPropertyItems *pDlg = new CPropertyItems(m_DevSelInfo.index, this);
    pDlg->setAttribute(Qt::WA_DeleteOnClose);
    pDlg->setWindowTitle(tr("Property Items Dialog"));
    pDlg->show();
}

void CVideoDeviceDlg::on_Read_SensorOff_Click() {
	
	unsigned char Buffer[256] = {0};
	int nActualLength = 0;

	if( ETronDI_OK == EtronDI_GetYOffset( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, Buffer, 256, &nActualLength, 0)) {
			
		QString fileName = QFileDialog::getSaveFileName(this,
		tr("Save file"),
		tr("SensorOffset"),
		tr("Bin Files (*.bin)"));

		if(fileName.isEmpty()) {
			
			QMessageBox::critical(NULL, "Error", "File path empty .", QMessageBox::Yes , QMessageBox::Yes);
			return;
		}
		else {
			
			fileName += ".bin";
			qDebug() << fileName << "\n";

			QFile file(fileName);
			file.open(QIODevice::WriteOnly);
			QDataStream out(&file);

			for(int i=0;i<nActualLength;i++) {
				
				out << Buffer[i];
			}
			file.close();
			
			QMessageBox::information(NULL, "Info", "Read Sensor Offset Complete !!", QMessageBox::Yes, QMessageBox::Yes);
		}
	}
}

void CVideoDeviceDlg::on_Write_SensorOff_Click() {
	
	QString fileName;
    fileName = QFileDialog::getOpenFileName( this, 
	tr("Open file"),
    tr(""), 
    tr("Bin Files (*.bin)"));
    
    if(fileName.isEmpty()) { 
		
		QMessageBox::critical(NULL, "Error", "File path empty .", QMessageBox::Yes , QMessageBox::Yes);
	}
	else {
		
		QFile file(fileName);
		file.open(QIODevice::ReadOnly);
		
		unsigned char *Buffer = (unsigned char*)calloc( file.size(), sizeof(unsigned char));
		
		QDataStream in(&file);
		
        long int i=0;
        quint8 temp = 0;
		int nActualLength = 0;
		
        for(i=0;i<file.size();i++) {
			in >> temp;
			Buffer[i] = (unsigned char)temp;
		}
		
		if( ETronDI_OK != EtronDI_SetYOffset( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, Buffer, file.size(), &nActualLength, 0)) {
			
			qDebug("EtronDI_SetYOffset failed !!");
		}
		
		QMessageBox::information(NULL, "Info", "Write Sensor offset Complete !!", QMessageBox::Yes, QMessageBox::Yes);
	}
}

void CVideoDeviceDlg::on_Read_RecTable_Click() {
	
	unsigned char Buffer[1024] = {0};
	int nActualLength = 0;

	if(ETronDI_OK == EtronDI_GetRectifyTable( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, Buffer, 1024, &nActualLength, 0)) {
			
		QString fileName = QFileDialog::getSaveFileName(this,
		tr("Save file"),
		tr("RecTable"),
		tr("Bin Files (*.bin)"));

		if(fileName.isEmpty()) {
			
			QMessageBox::critical(NULL, "Error", "File path empty .", QMessageBox::Yes , QMessageBox::Yes);
			return;
		}
		else {
			
			fileName += ".bin";
			qDebug() << fileName << "\n";

			QFile file(fileName);
			file.open(QIODevice::WriteOnly);
			QDataStream out(&file);

			for(int i=0;i<nActualLength;i++) {
				
				out << Buffer[i];
			}
			file.close();
			
			QMessageBox::information(NULL, "Info", "Read Rectify Table Complete !!", QMessageBox::Yes, QMessageBox::Yes);
		}
	}
}

void CVideoDeviceDlg::on_Write_RecTable_Click() {
	
	QString fileName;
    fileName = QFileDialog::getOpenFileName( this, 
	tr("Open file"),
    tr(""), 
    tr("Bin Files (*.bin)"));
    
    if(fileName.isEmpty()) { 
		
		QMessageBox::critical(NULL, "Error", "File path empty .", QMessageBox::Yes , QMessageBox::Yes);
	}
	else {
		
		QFile file(fileName);
		file.open(QIODevice::ReadOnly);
		
		unsigned char *Buffer = (unsigned char*)calloc( file.size(), sizeof(unsigned char));
		
		QDataStream in(&file);
		
        long int i=0;
        quint8 temp = 0;
		int nActualLength = 0;
		
        for(i=0;i<file.size();i++) {
			in >> temp;
			Buffer[i] = (unsigned char)temp;
		}
		
		if( ETronDI_OK != EtronDI_SetRectifyTable( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, Buffer, file.size(), &nActualLength, 0)) {
			
			qDebug("EtronDI_SetRectifyTable failed !!");
		}
		
		QMessageBox::information(NULL, "Info", "Write Rectify Table Complete !!", QMessageBox::Yes, QMessageBox::Yes);
	}
}

void CVideoDeviceDlg::on_Read_ZDTable_Click() {
	
    ZDTABLEINFO zdTableInfo;
    unsigned char *buffer;
    int nBufferLength = 0;
	int nActualLength = 0;

    zdTableInfo.nDataType = m_nZDTableDataType;

    // note : current arrangment; index 0 : 8 bits, 512 bytes; index 1 : 11 bits, 4096 bytes
    if( m_nZDTableDataType == ETronDI_DEPTH_DATA_DEFAULT){
        zdTableInfo.nIndex = 0;
        buffer = (unsigned char*)malloc(ETronDI_ZD_TABLE_FILE_SIZE_8_BITS);
        nBufferLength = ETronDI_ZD_TABLE_FILE_SIZE_8_BITS;
    }
    else{
         zdTableInfo.nIndex = 1;
         buffer = (unsigned char*)malloc(ETronDI_ZD_TABLE_FILE_SIZE_11_BITS);
         nBufferLength = ETronDI_ZD_TABLE_FILE_SIZE_11_BITS;
    }

    if( ETronDI_OK == EtronDI_GetZDTable( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, buffer, nBufferLength, &nActualLength, &zdTableInfo)) {
			
		QString fileName = QFileDialog::getSaveFileName(this,
		tr("Save file"),
		tr("ZDTable"),
		tr("Bin Files (*.bin)"));

		if(fileName.isEmpty()) {
			
			QMessageBox::critical(NULL, "Error", "File path empty .", QMessageBox::Yes , QMessageBox::Yes);
			return;
		}
		else {
			
			fileName += ".bin";
			qDebug() << fileName << "\n";

			QFile file(fileName);
			file.open(QIODevice::WriteOnly);
			QDataStream out(&file);

			for(int i=0;i<nActualLength;i++) {
				
                out << buffer[i];
			}
			file.close();
			
			QMessageBox::information(NULL, "Info", "Read ZD Table Complete !!", QMessageBox::Yes, QMessageBox::Yes);
		}
	}

    free(buffer);
}

void CVideoDeviceDlg::on_Write_ZDTable_Click() {
	
	QString fileName;
    ZDTABLEINFO zdTableInfo;
    fileName = QFileDialog::getOpenFileName( this, 
	tr("Open file"),
    tr(""), 
    tr("Bin Files (*.bin)"));
    
    if(fileName.isEmpty()) { 
		
		QMessageBox::critical(NULL, "Error", "File path empty .", QMessageBox::Yes , QMessageBox::Yes);
	}
	else {
		
		QFile file(fileName);
		file.open(QIODevice::ReadOnly);
		
		unsigned char *Buffer = (unsigned char*)calloc( file.size(), sizeof(unsigned char));
		
		QDataStream in(&file);
		
        long int i=0;
        quint8 temp = 0;
		int nActualLength = 0;
		
        for(i=0;i<file.size();i++) {
			in >> temp;
			Buffer[i] = (unsigned char)temp;
		}

        zdTableInfo.nDataType = m_nZDTableDataType;

        // note : current arrangment; index 0 : 8 bits, 512 bytes; index 1 : 11 bits, 4096 bytes
        if( m_nZDTableDataType == ETronDI_DEPTH_DATA_DEFAULT) zdTableInfo.nIndex = 0;
        else zdTableInfo.nIndex = 1;
		
        if( ETronDI_OK != EtronDI_SetZDTable( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, Buffer, file.size(), &nActualLength, &zdTableInfo)) {
			
			qDebug("EtronDI_SetZDTable failed !!");
		}
		
		QMessageBox::information(NULL, "Info", "Write ZD Table Complete !!", QMessageBox::Yes, QMessageBox::Yes);
	}
}

void CVideoDeviceDlg::on_m_ZDDataType_Combobox_currentIndexChanged(int index){

    if(index) m_nZDTableDataType = ETronDI_DEPTH_DATA_11_BITS;
    else m_nZDTableDataType = ETronDI_DEPTH_DATA_DEFAULT;
}

void CVideoDeviceDlg::on_Read_Log_Click() {
	
	if( outputTxtChk->isChecked() == false ) {
	
		unsigned char Buffer[4096] = {0};
		int nActualLength = 0;

		if( ETronDI_OK == EtronDI_GetLogData( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, Buffer, 4096, &nActualLength, 0, RECTIFY_LOG)) {
				
			QString fileName = QFileDialog::getSaveFileName(this,
			tr("Save file"),
			tr("Log"),
			tr("Bin Files (*.bin)"));

			if(fileName.isEmpty()) {
				
				QMessageBox::critical(NULL, "Error", "File path empty .", QMessageBox::Yes , QMessageBox::Yes);
				return;
			}
			else {
				
				fileName += ".bin";
				qDebug() << fileName << "\n";

				QFile file(fileName);
				file.open(QIODevice::WriteOnly);
				QDataStream out(&file);

				for(int i=0;i<nActualLength;i++) {
					
					out << Buffer[i];
				}
				file.close();
				
				QMessageBox::information(NULL, "Info", "Read Log Complete !!", QMessageBox::Yes, QMessageBox::Yes);
			}
		}

		else {
			QMessageBox::critical(NULL, "Error", "Get Log Failed .", QMessageBox::Yes , QMessageBox::Yes);
			return;
		}
	}
	
	else {
	
		// for parse log test
		eSPCtrl_RectLogData eSPRectLogData;

        int nRet = EtronDI_GetRectifyLogData( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, &eSPRectLogData);
        qDebug("nRet = %d", nRet);

		FILE *pfile;
		pfile = fopen("log.txt", "wt");
		if(pfile != NULL) {

			int i;

			//
			fprintf(pfile, "InImgWidth = %d\n",        eSPRectLogData.InImgWidth);
			fprintf(pfile, "InImgHeight = %d\n",       eSPRectLogData.InImgHeight);
			fprintf(pfile, "OutImgWidth = %d\n",       eSPRectLogData.OutImgWidth);
			fprintf(pfile, "OutImgHeight = %d\n",      eSPRectLogData.OutImgHeight);
			//
			fprintf(pfile, "RECT_ScaleWidth = %d\n",   eSPRectLogData.RECT_ScaleWidth);
			fprintf(pfile, "RECT_ScaleHeight = %d\n",  eSPRectLogData.RECT_ScaleHeight);
			//
			fprintf(pfile, "CamMat1 = ");
			for (i=0; i<9; i++) {
				fprintf(pfile, "%.8f, ",  eSPRectLogData.CamMat1[i]);
			}
			fprintf(pfile, "\n");
			//
			fprintf(pfile, "CamDist1 = ");
			for (i=0; i<8; i++) {
				fprintf(pfile, "%.8f, ",  eSPRectLogData.CamDist1[i]);
			}
			fprintf(pfile, "\n");
			//
			fprintf(pfile, "CamMat2 = ");
			for (i=0; i<9; i++) {
				fprintf(pfile, "%.8f, ",  eSPRectLogData.CamMat2[i]);
			}
			fprintf(pfile, "\n");
			//
			fprintf(pfile, "CamDist2 = ");
			for (i=0; i<8; i++) {
				fprintf(pfile, "%.8f, ",  eSPRectLogData.CamDist2[i]);
			}
			fprintf(pfile, "\n");
			//
			fprintf(pfile, "RotaMat = ");
			for (i=0; i<9; i++) {
				fprintf(pfile, "%.8f, ",  eSPRectLogData.RotaMat[i]);
			}
			fprintf(pfile, "\n");
			//
			fprintf(pfile, "TranMat = ");
			for (i=0; i<3; i++) {
				fprintf(pfile, "%.8f, ",  eSPRectLogData.TranMat[i]);
			}
			fprintf(pfile, "\n");
			//
			fprintf(pfile, "LRotaMat = ");
			for (i=0; i<9; i++) {
				fprintf(pfile, "%.8f, ",  eSPRectLogData.LRotaMat[i]);
			}
			fprintf(pfile, "\n");
			//
			fprintf(pfile, "RRotaMat = ");
			for (i=0; i<9; i++) {
				fprintf(pfile, "%.8f, ",  eSPRectLogData.RRotaMat[i]);
			}
			fprintf(pfile, "\n");
			//
			fprintf(pfile, "NewCamMat1 = ");
			for (i=0; i<12; i++) {
				fprintf(pfile, "%.8f, ",  eSPRectLogData.NewCamMat1[i]);
			}
			fprintf(pfile, "\n");
			//
			fprintf(pfile, "NewCamMat2 = ");
			for (i=0; i<12; i++) {
				fprintf(pfile, "%.8f, ",  eSPRectLogData.NewCamMat2[i]);
			}
			fprintf(pfile, "\n");
			//
			fprintf(pfile, "RECT_Crop_Row_BG = %d\n",   eSPRectLogData.RECT_Crop_Row_BG);
			fprintf(pfile, "RECT_Crop_Row_ED = %d\n",   eSPRectLogData.RECT_Crop_Row_ED);
			fprintf(pfile, "RECT_Crop_Col_BG_L = %d\n", eSPRectLogData.RECT_Crop_Col_BG_L);
			fprintf(pfile, "RECT_Crop_Col_ED_L = %d\n", eSPRectLogData.RECT_Crop_Col_ED_L);
			fprintf(pfile, "RECT_Scale_Col_M = %d\n",   eSPRectLogData.RECT_Scale_Col_M);
			fprintf(pfile, "RECT_Scale_Col_N = %d\n",   eSPRectLogData.RECT_Scale_Col_N);
			fprintf(pfile, "RECT_Scale_Row_M = %d\n",   eSPRectLogData.RECT_Scale_Row_M);
			fprintf(pfile, "RECT_Scale_Row_N = %d\n",   eSPRectLogData.RECT_Scale_Row_N);
			//
			fprintf(pfile, "RECT_AvgErr = %.8f\n", eSPRectLogData.RECT_AvgErr);
			//
			fprintf(pfile, "nLineBuffers = %d\n",  eSPRectLogData.nLineBuffers);
			//
		}

		fclose(pfile);

        QMessageBox::information(NULL, "Info", "Read Rectify Log Complete !!", QMessageBox::Yes, QMessageBox::Yes);
	}
}

void CVideoDeviceDlg::on_Write_Log_Click() {
	
	QString fileName;
    fileName = QFileDialog::getOpenFileName( this, 
	tr("Open file"),
    tr(""), 
    tr("Bin Files (*.bin)"));
    
    if(fileName.isEmpty()) { 
		
		QMessageBox::critical(NULL, "Error", "File path empty .", QMessageBox::Yes , QMessageBox::Yes);
	}
	else {
		
		QFile file(fileName);
		file.open(QIODevice::ReadOnly);
		
		unsigned char *Buffer = (unsigned char*)calloc( file.size(), sizeof(unsigned char));
		
		QDataStream in(&file);
		
        long int i=0;
        quint8 temp = 0;
		int nActualLength = 0;
		
        for(i=0;i<file.size();i++) {
			in >> temp;
			Buffer[i] = (unsigned char)temp;
		}
		
        if( ETronDI_OK != EtronDI_SetLogData( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, Buffer, file.size(), &nActualLength, 0)) {
			
			qDebug("EtronDI_SetLogData failed !!");
		}
		
		QMessageBox::information(NULL, "Info", "Write Log Complete !!", QMessageBox::Yes, QMessageBox::Yes);
	}
}

void CVideoDeviceDlg::on_Read_UserData_Click() {
	
	unsigned char Buffer[422] = {0};
	USERDATA_SECTION_INDEX usi = USERDATA_SECTION_0;

	if( ETronDI_OK == EtronDI_GetUserData( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, Buffer, 422, usi)) {

		QString fileName = QFileDialog::getSaveFileName(this,
														tr("Save file"),
														tr("UserData"),
														tr("Bin Files (*.bin)"));

		if(fileName.isEmpty()) {

			QMessageBox::critical(NULL, "Error", "File path empty .", QMessageBox::Yes , QMessageBox::Yes);
			return;
		}
		else {

			fileName += ".bin";
			qDebug() << fileName << "\n";

			QFile file(fileName);
			file.open(QIODevice::WriteOnly);
			QDataStream out(&file);

			for(int i=0;i<422;i++) {

				out << Buffer[i];
			}
			file.close();

			QMessageBox::information(NULL, "Info", "Read User Data Complete !!", QMessageBox::Yes, QMessageBox::Yes);
		}
	}
}

void CVideoDeviceDlg::on_Write_UserData_Click() {
	
	QString fileName;
	fileName = QFileDialog::getOpenFileName( this, 
	tr("Open file"),
	tr(""), 
	tr("Bin Files (*.bin)"));

	if(fileName.isEmpty()) { 
		
		QMessageBox::critical(NULL, "Error", "File path empty .", QMessageBox::Yes , QMessageBox::Yes);
	}
	else {
		
		QFile file(fileName);
		file.open(QIODevice::ReadOnly);
		
		unsigned char *Buffer = (unsigned char*)calloc( file.size(), sizeof(unsigned char));
		
		QDataStream in(&file);
		
		long int i=0;
		quint8 temp = 0;
		
		for(i=0;i<file.size();i++) {
			in >> temp;
			Buffer[i] = (unsigned char)temp;
		}
		USERDATA_SECTION_INDEX usi = USERDATA_SECTION_0;
		if( ETronDI_OK != EtronDI_SetUserData( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, Buffer, file.size(), usi)) {
			
			qDebug("EtronDI_SetUserData failed !!");
		}
		
		QMessageBox::information(NULL, "Info", "Write User Data Complete !!", QMessageBox::Yes, QMessageBox::Yes);
	}
}
