#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QList<QToolBar *> allToolBars = findChildren<QToolBar *>();
    foreach(QToolBar *tb, allToolBars) {
        // This does not delete the tool bar.
        removeToolBar(tb);
    }

    m_pEtronDI = NULL;

    int nRet = EtronDI_Init(&m_pEtronDI, false);
    qDebug("nRet = %d", nRet);

    //nRet = EtronDI_FindDevice(m_pEtronDI);
    //qDebug("nRet = %d", nRet);

    int nDevCount = EtronDI_GetDeviceNumber(m_pEtronDI);
    qDebug("nDevCount = %d", nDevCount);
    m_pDevInfo = (DEVINFORMATION*)malloc(sizeof(DEVINFORMATION)*nDevCount);

	int i = 0;

    for( i = 0 ; i < 200 ; i++) m_pVideoDevDlg[i] = NULL;
	
    for( i = 0 ; i < nDevCount ; i++) {
        m_DevSelInfo.index = i;
        EtronDI_GetDeviceInfo(m_pEtronDI, &m_DevSelInfo ,m_pDevInfo+i);
        qDebug("Device Name = %s", m_pDevInfo[i].strDevName);
        qDebug("PID = 0x%04x", m_pDevInfo[i].wPID);
        qDebug("VID = 0x%04x", m_pDevInfo[i].wVID);
        qDebug("Chip ID = 0x%x", m_pDevInfo[i].nChipID);
        
        char szBuf[256];
        int nActualLength = 0;
        if( ETronDI_OK == EtronDI_GetFwVersion( m_pEtronDI, &m_DevSelInfo, szBuf, 256, &nActualLength)) {
            qDebug("FW Version = %s\n", szBuf);
            ui->m_Device_Combobox->addItem(m_pDevInfo[i].strDevName);
        }
        
        //m_pVideoDevThread[i] = new CVideoDevThread(this, &m_DevSelInfo);
    }
    m_VideoIndex = 0;
}

MainWindow::~MainWindow()
{
    delete ui;
    EtronDI_Release(&m_pEtronDI);

}

void MainWindow::on_m_Device_Combobox_currentIndexChanged(int index) {
	
	m_VideoIndex = index;
}

void MainWindow::on_OpenBtn_clicked() {

    m_DevSelInfo.index = m_VideoIndex;

    if(m_pVideoDevDlg[m_VideoIndex] == NULL ) {

        m_pVideoDevDlg[m_VideoIndex] = new CVideoDeviceDlg( m_VideoIndex, this);
        m_pVideoDevDlg[m_VideoIndex]->setAttribute(Qt::WA_DeleteOnClose);
        m_pVideoDevDlg[m_VideoIndex]->setWindowTitle(tr(m_pDevInfo[m_VideoIndex].strDevName));
    }

    m_pVideoDevDlg[m_VideoIndex]->show();

    //connect( m_pVideoDevDlg[m_VideoIndex], SIGNAL(destroyed()), this, SLOT(On_Dialog_Destroy()));
}
