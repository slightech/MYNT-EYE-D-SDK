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
