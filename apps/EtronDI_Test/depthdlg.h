#ifndef CDEPTHDLG_H
#define CDEPTHDLG_H

#include <QDialog>
#include <QTimer>
#include <QLabel>

#include "eSPDI.h"

class MainWindow;

class CDepthDlg : public QDialog
{
    Q_OBJECT

public:
    explicit CDepthDlg(int nVideoIndex, QWidget *parent = 0);
    ~CDepthDlg();

    void closeEvent(QCloseEvent *event);

    int convert_yuv_to_rgb_pixel(int y, int u, int v);
    int convert_yuv_to_rgb_buffer(unsigned char *yuv, unsigned char *rgb, unsigned int width, unsigned int height);

private:
    void paintEvent(QPaintEvent *);

public:
    MainWindow *m_pMainWindow;
    DEVSELINFO m_DevSelInfo;

    QTimer *m_pTimer;
    QLabel *m_pDepthImgLabel;
    QLabel *m_pSnLabel;

    unsigned int m_nImageWidth;
    unsigned int m_nImageHeight;
    int m_nDepthDataType;
    //bool m_bIsMJPEG;

    unsigned char *m_pDepthImgBuf;
    unsigned char *m_pRGBBuf;
    
    int m_nSerialNumber;

    unsigned long int m_nImageSize;

    DEPTH_TRANSFER_CTRL m_Dtc;
};

#endif // CDEPTHDLG_H
