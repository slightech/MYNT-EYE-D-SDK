#ifndef CVIDEODEVICETHREAD_H
#define CVIDEODEVICETHREAD_H

#include <QThread>

#include "eSPDI.h"

class CVideoDeviceDlg;

class CVideoDeviceThread : public QThread
{
	Q_OBJECT
public:
    CVideoDeviceThread(QWidget *parent = 0);
    void readFlash();
    void writeFlash();

protected:
    void run();

signals:
    void done(bool bIsSuccess);    

public:
	CVideoDeviceDlg *m_VideoDeviceDlg;
	bool m_bIsSuccess;
};

#endif // CVIDEODEVICETHREAD_H
