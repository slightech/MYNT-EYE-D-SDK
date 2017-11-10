#include "depthdlg.h"
#include "videodevicedlg.h"
#include "mainwindow.h"

CDepthDlg::CDepthDlg(int nVideoIndex, QWidget *parent) :
                    QDialog(parent) {

    CVideoDeviceDlg *pDlg = (CVideoDeviceDlg*)parent;

    m_pMainWindow = pDlg->m_pMainWindow;
    m_DevSelInfo.index = nVideoIndex;

    m_Dtc = pDlg->m_Dtc;
    
    m_nSerialNumber = 0;
    m_nImageSize = 0;

    m_nImageWidth  = (unsigned int)(pDlg->m_pStreamDepthInfo[pDlg->m_nDepthResIndex].nWidth);
    m_nImageHeight = (unsigned int)(pDlg->m_pStreamDepthInfo[pDlg->m_nDepthResIndex].nHeight);
    m_nDepthDataType = pDlg->m_nDepthDataType;

    if( m_Dtc == DEPTH_IMG_COLORFUL_TRANSFER || m_Dtc == DEPTH_IMG_GRAY_TRANSFER ) {
		m_pDepthImgBuf = (unsigned char*)calloc(m_nImageWidth*2*m_nImageHeight*3, sizeof(unsigned char));
		m_pRGBBuf = NULL;
	}
    else {
		m_pDepthImgBuf = (unsigned char*)calloc(m_nImageWidth*m_nImageHeight*2, sizeof(unsigned char));
		m_pRGBBuf = (unsigned char*)calloc(m_nImageWidth*m_nImageHeight*3, sizeof(unsigned char));
	}

    resize(m_nImageWidth, m_nImageHeight);

	m_pSnLabel = new QLabel();
    m_pSnLabel->setText(tr("Serial number:"));
    QFont font("Courier 10 Pitch", 10, QFont::Bold);
    m_pSnLabel->setFont(font);

    m_pDepthImgLabel = new QLabel();

    QVBoxLayout *vLayout = new QVBoxLayout();
    vLayout->addWidget(m_pSnLabel);
    vLayout->addWidget(m_pDepthImgLabel);
    setLayout(vLayout);

    m_pTimer = new QTimer(this);
    connect(m_pTimer,SIGNAL(timeout()),this,SLOT(update()));
    m_pTimer->start(30);
}

CDepthDlg::~CDepthDlg() {

    if(m_pDepthImgBuf != NULL) {
        delete m_pDepthImgBuf;
        m_pDepthImgBuf = NULL;
    }

    if(m_pRGBBuf != NULL) {
        delete m_pRGBBuf;
        m_pRGBBuf = NULL;
    }
}

void CDepthDlg::closeEvent(QCloseEvent *event) {

    EtronDI_CloseDevice(m_pMainWindow->m_pEtronDI, &m_DevSelInfo);
    event->accept();
}

void CDepthDlg::paintEvent(QPaintEvent *) {

    QImage img;
    DEVINFORMATION devInfo;

    EtronDI_GetDeviceInfo( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, &devInfo);

    if( ETronDI_OK == EtronDI_GetImage( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, (BYTE*)m_pDepthImgBuf, &m_nImageSize, &m_nSerialNumber, m_nDepthDataType) ) {
		
        QString str;
        str.sprintf("Serial number:%03d", m_nSerialNumber);
		m_pSnLabel->setText(str);
        //qDebug("Serial number =%03d", m_nSerialNumber);
		
		if(m_Dtc == DEPTH_IMG_COLORFUL_TRANSFER || m_Dtc == DEPTH_IMG_GRAY_TRANSFER ) {
			
            if(devInfo.nDevType != AXES1){
                img = QImage( m_pDepthImgBuf, m_nImageWidth, m_nImageHeight, QImage::Format_RGB888);
                m_pDepthImgLabel->setFixedWidth(m_nImageWidth);
            }
            else{
                img = QImage( m_pDepthImgBuf, m_nImageWidth*2, m_nImageHeight, QImage::Format_RGB888);
                m_pDepthImgLabel->setFixedWidth(m_nImageWidth*2);
            }
        }
        else {
			
			convert_yuv_to_rgb_buffer( m_pDepthImgBuf, m_pRGBBuf, m_nImageWidth, m_nImageHeight);
			
            img = QImage( m_pRGBBuf, m_nImageWidth, m_nImageHeight, QImage::Format_RGB888);
            m_pDepthImgLabel->setFixedWidth(m_nImageWidth); 
		}
		
		m_pDepthImgLabel->setFixedHeight(m_nImageHeight);
		m_pDepthImgLabel->setPixmap(QPixmap::fromImage(*(&img),Qt::AutoColor));
	}
}

// YUYV to RGB +
int CDepthDlg::convert_yuv_to_rgb_buffer(unsigned char *yuv, unsigned char *rgb, unsigned int width, unsigned int height)
{
    unsigned int in, out = 0;
    unsigned int pixel_16;
    unsigned char pixel_24[3];
    unsigned int pixel32;
    int y0, u, y1, v;
    for(in = 0; in < width * height * 2; in += 4) {
        pixel_16 =
        yuv[in + 3] << 24 |
        yuv[in + 2] << 16 |
        yuv[in + 1] <<  8 |
        yuv[in + 0];

        y0 = (pixel_16 & 0x000000ff);
        u  = (pixel_16 & 0x0000ff00) >>  8;
        y1 = (pixel_16 & 0x00ff0000) >> 16;
        v  = (pixel_16 & 0xff000000) >> 24;

        pixel32 = convert_yuv_to_rgb_pixel(y0, u, v);

        pixel_24[0] = (pixel32 & 0x000000ff);
        pixel_24[1] = (pixel32 & 0x0000ff00) >> 8;
        pixel_24[2] = (pixel32 & 0x00ff0000) >> 16;
        rgb[out++] = pixel_24[0];
        rgb[out++] = pixel_24[1];
        rgb[out++] = pixel_24[2];

        pixel32 = convert_yuv_to_rgb_pixel(y1, u, v);

        pixel_24[0] = (pixel32 & 0x000000ff);
        pixel_24[1] = (pixel32 & 0x0000ff00) >> 8;
        pixel_24[2] = (pixel32 & 0x00ff0000) >> 16;
        rgb[out++] = pixel_24[0];
        rgb[out++] = pixel_24[1];
        rgb[out++] = pixel_24[2];
    }

    return 0;
}

int CDepthDlg::convert_yuv_to_rgb_pixel(int y, int u, int v)
{
    unsigned int pixel32 = 0;
    unsigned char *pixel = (unsigned char *)&pixel32;
    int r, g, b;

    r = y + (1.370705 * (v-128));
    g = y - (0.698001 * (v-128)) - (0.337633 * (u-128));
    b = y + (1.732446 * (u-128));

    if(r > 255) r = 255;
    if(g > 255) g = 255;
    if(b > 255) b = 255;

    if(r < 0) r = 0;
    if(g < 0) g = 0;
    if(b < 0) b = 0;

    pixel[0] = r * 220 / 256;
    pixel[1] = g * 220 / 256;
    pixel[2] = b * 220 / 256;

    return pixel32;
}
// YUYV to RGB -

