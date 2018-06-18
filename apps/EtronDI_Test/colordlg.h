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
#ifndef CCOLORDLG_H
#define CCOLORDLG_H

#include <QDialog>
#include <QTimer>
#include <QTime>
#include <QLabel>

#include "eSPDI.h"

extern "C"{
    #include "libavcodec/avcodec.h"
    #include "libavformat/avformat.h"
    #include "libavutil/avutil.h"
    #include "libavutil/mem.h"
    #include "libavutil/fifo.h"
    #include "libswscale/swscale.h"
}

class MainWindow;

class CColorDlg : public QDialog
{
    Q_OBJECT

public:
    explicit CColorDlg(int nVideoIndex, QWidget *parent = 0);
    ~CColorDlg();

    void closeEvent(QCloseEvent *event);

    int convert_yuv_to_rgb_pixel(int y, int u, int v);
    int convert_yuv_to_rgb_buffer(unsigned char *yuv, unsigned char *rgb, unsigned int width, unsigned int height);
    int MJPEG_TO_RGB24_LIBJPEG(unsigned char *jpg, int nJpgSize, unsigned char *rgb);
    bool YUVToRGB24_FFmpeg(unsigned char* pYUV, unsigned char* pRGB24, int width, int height);

private:
    void paintEvent(QPaintEvent *);

public:
    MainWindow *m_pMainWindow;
    DEVSELINFO m_DevSelInfo;

    QTimer *m_pTimer;
    QLabel *m_pColorImgLabel;
    QLabel *m_pSnLabel;

    unsigned int m_nImageWidth;
    unsigned int m_nImageHeight;
    bool m_bIsMJPEG;
    unsigned short m_nDepthDataType;

    unsigned char *m_pColorImgBuf;
    unsigned char *m_pRGBBuf;

    QTime curTime;

    int m_nSerialNumber;
    int m_nFrameCount;
    int	m_fFrameRate;
    int m_dwFrameTime[16];

    unsigned long int m_nImageSize;
};

#endif // CCOLORDLG_H
