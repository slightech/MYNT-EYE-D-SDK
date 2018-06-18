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
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLibrary>
#include <QDebug>
#include <QCloseEvent>

#include "videodevicedlg.h"

class QCloseEvent;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

public:
    void *m_pEtronDI;
    DEVSELINFO m_DevSelInfo;
    DEVINFORMATION *m_pDevInfo;
    CVideoDeviceDlg *m_pVideoDevDlg[200];
    int m_VideoIndex;

private slots:
    void on_m_Device_Combobox_currentIndexChanged(int index);
    void on_OpenBtn_clicked();
};

#endif // MAINWINDOW_H
