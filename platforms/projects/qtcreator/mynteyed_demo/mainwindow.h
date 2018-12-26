#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QLabel>
#include <QMainWindow>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QTimer>

#include <mynteyed/camera.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

    Q_PROPERTY(bool leftEnabled READ isLeftEnabled)
    Q_PROPERTY(bool rightEnabled READ isRightEnabled)
    Q_PROPERTY(bool depthEnabled READ isDepthEnabled)

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    bool isLeftEnabled() const { return leftEnabled; }
    bool isRightEnabled() const { return rightEnabled; }
    bool isDepthEnabled() const { return depthEnabled; }

    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

    void closeLater();

private:
    void openCamera();
    bool selectDevice(MYNTEYE_NAMESPACE::DeviceInfo *info);

    void setupUi(const MYNTEYE_NAMESPACE::OpenParams &params);

    void drawImage(QImage &image);
    void drawDatas(std::vector<MYNTEYE_NAMESPACE::MotionData> datas);

    Ui::MainWindow *ui;

    MYNTEYE_NAMESPACE::Camera cam;
    bool leftEnabled;
    bool rightEnabled;
    bool depthEnabled;

    QLabel *leftLabel;
    QLabel *rightLabel;
    QLabel *depthLabel;

    QTimer *timer;
};

#endif // MAINWINDOW_H
