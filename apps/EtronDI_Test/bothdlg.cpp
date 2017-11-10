#include "bothdlg.h"
#include "videodevicedlg.h"
#include "mainwindow.h"

METHODDEF(void)
my_error_exit (j_common_ptr cinfo)
{
  /* cinfo->err really points to a my_error_mgr struct, so coerce pointer */
  my_error_ptr myerr = (my_error_ptr) cinfo->err;

  /* Always display the message. */
  /* We could postpone this until after returning, if we chose. */
  (*cinfo->err->output_message) (cinfo);

  /* Return control to the setjmp point */
  longjmp(myerr->setjmp_buffer, 1);
}

CBothDlg::CBothDlg(int nVideoIndex, QWidget *parent) :
                   QDialog(parent) {

    CVideoDeviceDlg *pDlg = (CVideoDeviceDlg*)parent;

    m_pMainWindow = pDlg->m_pMainWindow;
    m_DevSelInfo.index = nVideoIndex;

    m_Dtc = pDlg->m_Dtc;

    m_nColorSerialNumber = 0;
    m_nDepthSerialNumber = 0;

    m_nColorImageSize = 0;
    m_nDepthImageSize = 0;

    m_nColorImageWidth  = (unsigned int)(pDlg->m_pStreamColorInfo[pDlg->m_nColorResIndex].nWidth);
    m_nColorImageHeight = (unsigned int)(pDlg->m_pStreamColorInfo[pDlg->m_nColorResIndex].nHeight);
    m_nDepthImageWidth  = (unsigned int)(pDlg->m_pStreamDepthInfo[pDlg->m_nDepthResIndex].nWidth);
    m_nDepthImageHeight = (unsigned int)(pDlg->m_pStreamDepthInfo[pDlg->m_nDepthResIndex].nHeight);

    m_bIsMJPEG     = pDlg->m_pStreamColorInfo[pDlg->m_nColorResIndex].bFormatMJPG;
    m_nDepthDataType = pDlg->m_nDepthDataType;

    m_pColorImgBuf = (unsigned char*)calloc( m_nColorImageWidth*m_nColorImageHeight*2, sizeof(unsigned char));
    m_pColorRGBBuf = (unsigned char*)calloc( m_nColorImageWidth*m_nColorImageHeight*3, sizeof(unsigned char));
    
    if(m_Dtc == DEPTH_IMG_COLORFUL_TRANSFER || m_Dtc == DEPTH_IMG_GRAY_TRANSFER ) {
		m_pDepthImgBuf = (unsigned char*)calloc( m_nDepthImageWidth*2*m_nDepthImageHeight*3, sizeof(unsigned char));
		m_pDepthRGBBuf = NULL;
	}
    else {
		m_pDepthImgBuf = (unsigned char*)calloc( m_nDepthImageWidth*m_nDepthImageHeight*2, sizeof(unsigned char));
		m_pDepthRGBBuf = (unsigned char*)calloc( m_nDepthImageWidth*m_nDepthImageHeight*3, sizeof(unsigned char));
	}

    if(m_nColorImageHeight > m_nDepthImageHeight)
        resize(m_nColorImageWidth+m_nDepthImageWidth, m_nColorImageHeight);
    else
        resize(m_nColorImageWidth+m_nDepthImageWidth, m_nDepthImageHeight);

	m_pColorSnLabel = new QLabel();
	m_pDepthSnLabel = new QLabel();
    m_pColorSnLabel->setText(tr("Color Serial number:"));
    m_pDepthSnLabel->setText(tr("Depth Serial number:"));
    QFont font("Courier 10 Pitch", 10, QFont::Bold);
    m_pColorSnLabel->setFont(font);
    m_pDepthSnLabel->setFont(font);

    m_pColorImgLabel = new QLabel();
    m_pDepthImgLabel = new QLabel();

    QGridLayout *gridLayout = new QGridLayout();
    gridLayout->addWidget(m_pColorSnLabel, 0, 0, Qt::AlignTop);
    gridLayout->addWidget(m_pDepthSnLabel, 0, 1, Qt::AlignTop);
    gridLayout->addWidget(m_pColorImgLabel, 1, 0, Qt::AlignTop);
    gridLayout->addWidget(m_pDepthImgLabel, 1, 1, Qt::AlignTop);
    setLayout(gridLayout);

    //QDesktopWidget widget;
    //QRect mainScreenSize = widget.availableGeometry(widget.primaryScreen());
    //m_ScreenSize = QApplication::desktop()->screenGeometry();

    m_pTimer = new QTimer(this);
    connect(m_pTimer,SIGNAL(timeout()),this,SLOT(update()));
    m_pTimer->start(10);
}

CBothDlg::~CBothDlg() {

    if(m_pColorImgBuf != NULL) {
        delete m_pColorImgBuf;
        m_pColorImgBuf = NULL;
    }

    if(m_pDepthImgBuf != NULL) {
        delete m_pDepthImgBuf;
        m_pDepthImgBuf = NULL;
    }

    if(m_pColorRGBBuf != NULL) {
        delete m_pColorRGBBuf;
        m_pColorRGBBuf = NULL;
    }

    if(m_pDepthRGBBuf != NULL) {
        delete m_pDepthRGBBuf;
        m_pDepthRGBBuf = NULL;
    }
}

void CBothDlg::closeEvent(QCloseEvent *event) {

    EtronDI_CloseDevice(m_pMainWindow->m_pEtronDI, &m_DevSelInfo);
    event->accept();
}

void CBothDlg::paintEvent(QPaintEvent *) {

    QImage img;
    QString str;

    if( ETronDI_OK == EtronDI_Get2Image( m_pMainWindow->m_pEtronDI, &m_DevSelInfo,
                                         (BYTE*)m_pColorImgBuf, (BYTE*)m_pDepthImgBuf,
                                         &m_nColorImageSize, &m_nDepthImageSize,
                                         &m_nColorSerialNumber, &m_nDepthSerialNumber, m_nDepthDataType)) {
        int i;
        int m_fFrameRate;
        curTime = QTime::currentTime();
        //QString strCurTime = curTime.toString("hh:mm:ss.zzz");
        //qDebug() << strCurTime;

        i = m_nFrameCount & 0x0F;
        m_dwFrameTime[i] = curTime.minute()* 60000 + curTime.second()*1000 +curTime.msec();

        if (m_nFrameCount < 16) {
            m_fFrameRate =  (1000.0*m_nFrameCount) /(m_dwFrameTime[i]-m_dwFrameTime[0]);
        } else {
            m_fFrameRate= (15*1000) / (m_dwFrameTime[i]-m_dwFrameTime[(i+1)&(0x0f)]);
        }

        m_nFrameCount++;
        if (m_nFrameCount>10000) m_nFrameCount=0;

        str.sprintf("[FPS] %d , Color Serial number:%d", m_fFrameRate, m_nColorSerialNumber);
        //qDebug("Color Serial number:%d", m_nColorSerialNumber);
        m_pColorSnLabel->setText(str);
        if(m_bIsMJPEG) {

            MJPEG_TO_RGB24_LIBJPEG( m_pColorImgBuf, m_nColorImageSize, m_pColorRGBBuf);
        }
        else {

            convert_yuv_to_rgb_buffer( m_pColorImgBuf, m_pColorRGBBuf, m_nColorImageWidth, m_nColorImageHeight);
            //YUVToRGB24_FFmpeg( m_pColorImgBuf, m_pColorRGBBuf, m_nColorImageWidth, m_nColorImageHeight);
        }

        img = QImage( m_pColorRGBBuf, m_nColorImageWidth, m_nColorImageHeight, QImage::Format_RGB888);

        m_pColorImgLabel->setFixedWidth(m_nColorImageWidth);
        m_pColorImgLabel->setFixedHeight(m_nColorImageHeight);
        m_pColorImgLabel->setPixmap(QPixmap::fromImage(*(&img),Qt::AutoColor));

        str.sprintf("Depth Serial number:%d", m_nDepthSerialNumber);
        //qDebug("Depth Serial number:%d", m_nDepthSerialNumber);
        m_pDepthSnLabel->setText(str);

        if(m_Dtc == DEPTH_IMG_COLORFUL_TRANSFER || m_Dtc == DEPTH_IMG_GRAY_TRANSFER ) {
            if(m_nDepthDataType) //11bits&14bits
            {
              img = QImage( m_pDepthImgBuf, m_nDepthImageWidth, m_nDepthImageHeight, QImage::Format_RGB888);
              m_pDepthImgLabel->setFixedWidth(m_nDepthImageWidth);
            } else { //8bits
             //convert_depth_y_to_rgb_buffer( m_pDepthImgBuf, m_pDepthRGBBuf, m_nDepthImageWidth, m_nDepthImageHeight);
              img = QImage( m_pDepthImgBuf, m_nDepthImageWidth*2, m_nDepthImageHeight, QImage::Format_RGB888);
              m_pDepthImgLabel->setFixedWidth(m_nDepthImageWidth*2);
            }
        }
        else {
            convert_yuv_to_rgb_buffer( m_pDepthImgBuf, m_pDepthRGBBuf, m_nDepthImageWidth, m_nDepthImageHeight);
            img = QImage( m_pDepthRGBBuf, m_nDepthImageWidth, m_nDepthImageHeight, QImage::Format_RGB888);
            m_pDepthImgLabel->setFixedWidth(m_nDepthImageWidth);
        }

        m_pDepthImgLabel->setFixedHeight(m_nDepthImageHeight);
        m_pDepthImgLabel->setPixmap(QPixmap::fromImage(*(&img),Qt::AutoColor));
    }
}

bool CBothDlg::YUVToRGB24_FFmpeg(unsigned char* pYUV, unsigned char* pRGB24, int width, int height) {
	
    if (width < 1 || height < 1 || pYUV == NULL || pRGB24 == NULL) return false;
    //int srcNumBytes,dstNumBytes;
    //uint8_t *pSrc,*pDst;
    AVPicture pFrameYUV,pFrameRGB;
    
    //pFrameYUV = avpicture_alloc();
    //srcNumBytes = avpicture_get_size(PIX_FMT_YUV420P,width,height);
    //pSrc = (uint8_t *)malloc(sizeof(uint8_t) * srcNumBytes);
    avpicture_fill( &pFrameYUV, pYUV, PIX_FMT_YUYV422 ,width, height);

    //Exchange U,V
    uint8_t *ptmp = pFrameYUV.data[1];
    pFrameYUV.data[1] = pFrameYUV.data[2];
    pFrameYUV.data[2] = ptmp;

    //pFrameRGB = avcodec_alloc_frame();
    //dstNumBytes = avpicture_get_size(PIX_FMT_BGR24,width,height);
    //pDst = (uint8_t *)malloc(sizeof(uint8_t) * dstNumBytes);
    avpicture_fill(&pFrameRGB, pRGB24, PIX_FMT_RGB24 ,width, height);

    struct SwsContext* imgCtx = NULL;
    imgCtx = sws_getContext( width, height, PIX_FMT_YUYV422, width, height, PIX_FMT_RGB24, SWS_BILINEAR, 0, 0, 0);

    if (imgCtx != NULL){
        sws_scale( imgCtx, pFrameYUV.data, pFrameYUV.linesize, 0, height, pFrameRGB.data, pFrameRGB.linesize);
        if(imgCtx){
            sws_freeContext(imgCtx);
            imgCtx = NULL;
        }
        return true;
    }
    else{
        sws_freeContext(imgCtx);
        imgCtx = NULL;
        return false;
    }
}

// YUYV to RGB +
int CBothDlg::convert_yuv_to_rgb_buffer(unsigned char *yuv, unsigned char *rgb, unsigned int width, unsigned int height)
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

int CBothDlg::convert_yuv_to_rgb_pixel(int y, int u, int v)
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

int CBothDlg::MJPEG_TO_RGB24_LIBJPEG(unsigned char *jpg, int nJpgSize,unsigned char *rgb) {
	struct jpeg_decompress_struct cinfo;
	struct my_error_mgr jerr;

	int rc;
	int row_stride, width, height, pixel_size;

	cinfo.err = jpeg_std_error(&jerr.pub);
	jerr.pub.error_exit = my_error_exit;

	if (setjmp(jerr.setjmp_buffer)) {
		jpeg_destroy_decompress(&cinfo);
		return 0;
	}
	
	jpeg_create_decompress(&cinfo);
	jpeg_mem_src(&cinfo, jpg, nJpgSize);

	rc = jpeg_read_header(&cinfo, TRUE);

	if (rc != 1) {
		qDebug("File does not seem to be a normal JPEG !!");
	}
	
	jpeg_start_decompress(&cinfo);
  
	width = cinfo.output_width;
	height = cinfo.output_height;
	pixel_size = cinfo.output_components;

	qDebug("Proc: Image is %d by %d with %d components", width, height, pixel_size);

	row_stride = width * pixel_size;

	while (cinfo.output_scanline < cinfo.output_height) {
		unsigned char *buffer_array[1];
		//buffer_array[0] = rgb + (width * height * 3) - (cinfo.output_scanline) * row_stride;
		buffer_array[0] = rgb + (cinfo.output_scanline) * row_stride;

		jpeg_read_scanlines(&cinfo, buffer_array, 1);
	}

	jpeg_finish_decompress(&cinfo);
	jpeg_destroy_decompress(&cinfo);

	return 0;
}
