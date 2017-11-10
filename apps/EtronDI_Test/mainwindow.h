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
