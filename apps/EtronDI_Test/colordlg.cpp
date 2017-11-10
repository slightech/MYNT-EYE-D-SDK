#include "colordlg.h"
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

CColorDlg::CColorDlg(int nVideoIndex, QWidget *parent) :
                    QDialog(parent) {

    CVideoDeviceDlg *pDlg = (CVideoDeviceDlg*)parent;

    m_pMainWindow = pDlg->m_pMainWindow;
    m_DevSelInfo.index = nVideoIndex;

    m_nSerialNumber = 0;
    m_nImageSize = 0;
    m_nFrameCount = 0;

    m_nImageWidth  = (unsigned int)(pDlg->m_pStreamColorInfo[pDlg->m_nColorResIndex].nWidth);
    m_nImageHeight = (unsigned int)(pDlg->m_pStreamColorInfo[pDlg->m_nColorResIndex].nHeight);
    m_bIsMJPEG     = pDlg->m_pStreamColorInfo[pDlg->m_nColorResIndex].bFormatMJPG;
    m_nDepthDataType = pDlg->m_nDepthDataType;

    m_pColorImgBuf = (unsigned char*)calloc(m_nImageWidth*m_nImageHeight*2, sizeof(unsigned char));
    m_pRGBBuf      = (unsigned char*)calloc(m_nImageWidth*m_nImageHeight*3, sizeof(unsigned char));

    resize(m_nImageWidth, m_nImageHeight);

    m_pSnLabel = new QLabel();
    m_pSnLabel->setText(tr("Serial number:"));
    QFont font("Courier 10 Pitch", 10, QFont::Bold);
    m_pSnLabel->setFont(font);

    m_pColorImgLabel = new QLabel();
    QVBoxLayout *vLayout = new QVBoxLayout();
    vLayout->addWidget(m_pSnLabel);
    vLayout->addWidget(m_pColorImgLabel);
    setLayout(vLayout);

    m_pTimer = new QTimer(this);
    connect(m_pTimer,SIGNAL(timeout()),this,SLOT(update()));
    m_pTimer->start(30);
}

CColorDlg::~CColorDlg() {

    if(m_pColorImgBuf != NULL) {
        delete m_pColorImgBuf;
        m_pColorImgBuf = NULL;
    }

    if(m_pRGBBuf != NULL) {
        delete m_pRGBBuf;
        m_pRGBBuf = NULL;
    }
}

void CColorDlg::closeEvent(QCloseEvent *event) {

    qDebug("closeEvent +");
    EtronDI_CloseDevice(m_pMainWindow->m_pEtronDI, &m_DevSelInfo);
    event->accept();
    qDebug("closeEvent -");
}

void CColorDlg::paintEvent(QPaintEvent *) {

    QImage img;
    QString str;
    int i;
    int m_fFrameRate = 0;

    int nRet = EtronDI_GetImage( m_pMainWindow->m_pEtronDI, &m_DevSelInfo, (BYTE*)m_pColorImgBuf, &m_nImageSize, &m_nSerialNumber, 0);

    if( nRet == ETronDI_OK) {

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

        str.sprintf("[FPS] %d, Serial number:%d", m_fFrameRate, m_nSerialNumber);
        m_pSnLabel->setText(str);
        if(m_bIsMJPEG) {
            MJPEG_TO_RGB24_LIBJPEG( m_pColorImgBuf, m_nImageSize, m_pRGBBuf);
        }
        else {
            //convert_yuv_to_rgb_buffer( m_pColorImgBuf, m_pRGBBuf, m_nImageWidth, m_nImageHeight);
            YUVToRGB24_FFmpeg( m_pColorImgBuf, m_pRGBBuf, m_nImageWidth, m_nImageHeight);
        }

        img = QImage( m_pRGBBuf, m_nImageWidth, m_nImageHeight, QImage::Format_RGB888);

        m_pColorImgLabel->setFixedWidth(m_nImageWidth);
        m_pColorImgLabel->setFixedHeight(m_nImageHeight);
        m_pColorImgLabel->setPixmap(QPixmap::fromImage(*(&img),Qt::AutoColor));
    }
}

bool CColorDlg::YUVToRGB24_FFmpeg(unsigned char* pYUV, unsigned char* pRGB24, int width, int height) {
	
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
int CColorDlg::convert_yuv_to_rgb_buffer(unsigned char *yuv, unsigned char *rgb, unsigned int width, unsigned int height)
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

int CColorDlg::convert_yuv_to_rgb_pixel(int y, int u, int v)
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

int CColorDlg::MJPEG_TO_RGB24_LIBJPEG(unsigned char *jpg, int nJpgSize,unsigned char *rgb) {
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

    //qDebug("Proc: Image is %d by %d with %d components", width, height, pixel_size);

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
