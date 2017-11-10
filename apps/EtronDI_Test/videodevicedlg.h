#ifndef CVIDEODEVICEDLG_H
#define CVIDEODEVICEDLG_H

#include <QDialog>
#include <QLibrary>
#include <QMessageBox>
#include <QDateTime>
#include <QFileDialog>

#include "ui_dialog.h"

#include <stdio.h>

#include "videodevicethread.h"
#include "colordlg.h"
#include "depthdlg.h"
#include "bothdlg.h"
#include "propertyitems.h"

// for CT PU +
#define PROP_TOTAL_NUMBER 28

#define PROP_TYPE_CT	0
#define PROP_TYPE_PU	1

typedef struct tagPROPERTYITEMINFO {
	char	szPropName[64];
	int		nPropType;
	int		nPropID;
} PROPERTY_ITEM_INFO;

// for CT PU -

// for libjpeg +

#include <setjmp.h>

extern "C" {
    #include "jpeglib.h"
}

struct my_error_mgr {
  struct jpeg_error_mgr pub;  /* "public" fields */

  jmp_buf setjmp_buffer;    /* for return to caller */
};

typedef struct my_error_mgr * my_error_ptr;

// for libjpeg -

typedef enum {	
    READ_FLASH = 0,
    WRITE_FLASH
} FUNCTION_SELECT;

class MainWindow;

class CVideoDeviceDlg : public QDialog,
                        public Ui_Dialog
{
    Q_OBJECT

public:
    explicit CVideoDeviceDlg(int nVideoIndex, QWidget *parent = 0);
    ~CVideoDeviceDlg();
    void closeEvent(QCloseEvent *event);
    void showEvent(QShowEvent* event);

private:    
	void getFWVersion();
	void getResolutionList();
    void getAEAWBStatus();
    void initPropertyItemList();

public:
    MainWindow *m_pMainWindow;
    DEVSELINFO m_DevSelInfo;

    FLASH_DATA_TYPE m_FlashDataType;
    KEEP_DATA_CTRL m_KeepDataCtrl;

    QTime m_time;
    FUNCTION_SELECT m_funcSelect;

    CVideoDeviceThread *m_pVideoDeviceThread;

    PETRONDI_STREAM_INFO m_pStreamColorInfo;
    PETRONDI_STREAM_INFO m_pStreamDepthInfo;
    int m_nColorResIndex;
    int m_nDepthResIndex;
    int m_nDepthDataType;
    int m_nZDTableDataType;

    DEPTH_TRANSFER_CTRL m_Dtc;
    
    int m_nFrameRate;
    
    PROPERTY_ITEM_INFO *m_pPropertyItemArray;

public slots:
    void on_PID_Get_Click();
    void on_PID_Set_Click();
    void on_SN_Get_Click();
    void on_SN_Set_Click();
    void on_FLASH_Read_Click();
    void on_FLASH_Write_Click();
    void threadDone(bool bIsSuccess);
    void on_PreviewBtn_Click();
    void on_m_ColorRes_Combobox_currentIndexChanged(int index);
    void on_m_DepthRes_Combobox_currentIndexChanged(int index);
    void on_m_DepthDataType_Combobox_currentIndexChanged(int index);
    void on_REG_Read_Click();
    void on_REG_Write_Click();
    void on_AE_State_Change (bool bIsChecked);
    void on_AWB_State_Change(bool bIsChecked);
    void on_AE_AWB_State_update();
    void on_Read_Property_Click();
    void on_Write_Property_Click();
    void on_Prop_Items_Click();

    void on_Read_SensorOff_Click();
    void on_Write_SensorOff_Click();
    void on_Read_RecTable_Click();
    void on_Write_RecTable_Click();
    void on_Read_ZDTable_Click();
    void on_Write_ZDTable_Click();
    void on_Read_Log_Click();
    void on_Write_Log_Click();
    void on_Read_UserData_Click();
    void on_Write_UserData_Click();
    void on_m_ZDDataType_Combobox_currentIndexChanged(int index);

    // for GPIO +
    void on_GPIO1_BIT7_Click();
    void on_GPIO1_BIT6_Click();
    void on_GPIO1_BIT5_Click();
    void on_GPIO1_BIT4_Click();
    void on_GPIO1_BIT3_Click();
    void on_GPIO1_BIT2_Click();
    void on_GPIO1_BIT1_Click();
    void on_GPIO1_BIT0_Click();

    void on_GPIO2_BIT7_Click();
    void on_GPIO2_BIT6_Click();
    void on_GPIO2_BIT5_Click();
    void on_GPIO2_BIT4_Click();
    void on_GPIO2_BIT3_Click();
    void on_GPIO2_BIT2_Click();
    void on_GPIO2_BIT1_Click();
    void on_GPIO2_BIT0_Click();

    void on_GPIO1Value_Read();
    void on_GPIO1Value_Write();
    void on_GPIO2Value_Read();
    void on_GPIO2Value_Write();
    // for GPIO -
};

#endif // CVIDEODEVICEDLG_H
