#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDebug>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QMessageBox>
#include <QPainter>
#include <QTimer>

#include <mynteyed/utils.h>

MYNTEYE_USE_NAMESPACE

namespace {

QDebug operator <<(QDebug debug, const DeviceInfo &info) {
    QDebugStateSaver saver(debug);
    debug.nospace().noquote() << "index: " << info.index
       << ", name: " << info.name.c_str()
       << ", type: " << QString("%1").arg(info.type, 0, 16)
       << ", pid: 0x" << QString("%1").arg(info.pid, 0, 16)
       << ", vid: 0x" << QString("%1").arg(info.vid, 0, 16)
       << ", chip_id: 0x" << QString("%1").arg(info.chip_id, 0, 16)
       << ", fw_version: " << info.fw_version.c_str();
    return debug;
}

}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    leftEnabled(false),
    rightEnabled(false),
    depthEnabled(false)
{
    ui->setupUi(this);

    openCamera();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    if (leftEnabled) {
        auto &&left = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
        if (left.img) {
            auto &&img = left.img->To(ImageFormat::COLOR_RGB);
            QImage image(img->data(), img->width(), img->height(),
                         QImage::Format_RGB888);
            drawImage(image);
            leftLabel->setPixmap(QPixmap::fromImage(image));
        }
    }
    if (rightEnabled) {
        auto &&right = cam.GetStreamData(ImageType::IMAGE_RIGHT_COLOR);
        if (right.img) {
            auto &&img = right.img->To(ImageFormat::COLOR_RGB);
            QImage image(img->data(), img->width(), img->height(),
                         QImage::Format_RGB888);
            drawImage(image);
            rightLabel->setPixmap(QPixmap::fromImage(image));
        }
    }
    if (depthEnabled) {
        auto &&depth = cam.GetStreamData(ImageType::IMAGE_DEPTH);
        if (depth.img) {
            auto &&img = depth.img->To(ImageFormat::COLOR_RGB);
            QImage image(img->data(), img->width(), img->height(),
                         QImage::Format_RGB888);
            drawImage(image);
            depthLabel->setPixmap(QPixmap::fromImage(image));
        }
    }
    if (cam.IsMotionDatasEnabled()) {
        auto &&datas = cam.GetMotionDatas();
        if (!datas.empty()) {
            drawDatas(datas);
        }
    }
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    Q_UNUSED(event);
    qDebug() << __func__;

    QRect rect(QPoint(0, 0), ui->centralWidget->size());
    rect.adjust(0, 0, 0, ui->menuBar->size().height());
    rect.adjust(0, 0, 0, ui->mainToolBar->size().height());
    rect.adjust(0, 0, 0, ui->statusBar->size().height());
    QSize size = rect.size();
    setFixedSize(size);
    setMinimumSize(size);
    setMaximumSize(size);
}

void MainWindow::closeLater()
{
    QTimer::singleShot(0, this, SLOT(close()));
}

void MainWindow::openCamera()
{
    DeviceInfo dev_info;
    bool ok = selectDevice(&dev_info);
    if (ok) {
        qDebug().nospace() << "Open device: " << dev_info.index
                           << ", " << dev_info.name.c_str();

        OpenParams params(dev_info.index);
        {
            // Framerate: 10(default), [0,60], [0,30](STREAM_2560x720)
            params.framerate = 10;

            // Device mode, default DEVICE_ALL
            //   DEVICE_COLOR: IMAGE_LEFT_COLOR ✓ IMAGE_RIGHT_COLOR ? IMAGE_DEPTH x
            //   DEVICE_DEPTH: IMAGE_LEFT_COLOR x IMAGE_RIGHT_COLOR x IMAGE_DEPTH ✓
            //   DEVICE_ALL:   IMAGE_LEFT_COLOR ✓ IMAGE_RIGHT_COLOR ? IMAGE_DEPTH ✓
            // Note: ✓: available, x: unavailable, ?: depends on #stream_mode
            //params.dev_mode = DeviceMode::DEVICE_ALL;

            // Color mode: raw(default), rectified
            //params.color_mode = ColorMode::COLOR_RECTIFIED;

            // Depth mode: colorful(default), gray, raw
            //params.depth_mode = DepthMode::DEPTH_GRAY;

            // Stream mode: left color only
            //params.stream_mode = StreamMode::STREAM_640x480;  // vga
            //params.stream_mode = StreamMode::STREAM_1280x720;  // hd
            // Stream mode: left+right color
            params.stream_mode = StreamMode::STREAM_1280x480;  // vga
            //params.stream_mode = StreamMode::STREAM_2560x720;  // hd

            // Auto-exposure: true(default), false
            //params.state_ae = false;

            // Auto-white balance: true(default), false
            //params.state_awb = false;

            // IR Depth Only: true, false(default)
            // Note: IR Depth Only mode support frame rate between 15fps and 30fps.
            //     When dev_mode != DeviceMode::DEVICE_ALL,
            //       IR Depth Only mode not be supported.
            //     When stream_mode == StreamMode::STREAM_2560x720,
            //       frame rate only be 15fps in this mode.
            //     When frame rate less than 15fps or greater than 30fps,
            //       IR Depth Only mode will be not available.
            //params.ir_depth_only = true;

            // Infrared intensity: 0(default), [0,10]
            params.ir_intensity = 4;
        }

        // Enable what process logics
        //cam.EnableProcessMode(ProcessMode::PROC_IMU_ALL);

        // Enable image infos
        //cam.EnableImageInfo(true);

        // Enable this will cache the motion datas until you get them
        cam.EnableMotionDatas();

        cam.Open(params);

        if (!cam.IsOpened()) {
            QMessageBox::warning(this,
                                 tr("Error"),
                                 tr("Open camera failed"),
                                 QMessageBox::Ok);
            return;
        }
        qDebug() << "Open device success";

        setupUi(params);
    } else {
        closeLater();
    }
}

bool MainWindow::selectDevice(DeviceInfo *dev_info)
{
    QString dashes(80, '-');

    std::vector<DeviceInfo> dev_infos = cam.GetDeviceInfos();

    std::size_t n = dev_infos.size();
    if (n <= 0) {
        QMessageBox::warning(this,
                             tr("Error"),
                             tr("There is no MYNT® EYE cameras"),
                             QMessageBox::Ok);
        return false;
    }
    {
        qDebug().noquote() << dashes;
        qDebug() << "Index | Device Information";
        qDebug().noquote() << dashes;
        for (auto &&info : dev_infos) {
          qDebug().noquote() << QString("%1").arg(info.index, 5)
                             << "|" << info;
        }
        qDebug().noquote() << dashes;
    }
    if (n == 1) {
        *dev_info = dev_infos[0];
        return true;
    } else {  // > 1
        QStringList items;
        for (auto &&info : dev_infos) {
            items << QString("%1 | %2").arg(info.index).arg(info.name.c_str());
        }

        bool ok;
        QString item = QInputDialog::getItem(this,
            tr("Select"), tr("MYNT® EYE cameras"), items, 0, false, &ok);
        if (ok) {
            QStringList parts = item.split("|");
            if (parts.length() > 0) {
                QString index =  parts[0].trimmed();
                for (auto &&info : dev_infos) {
                    if (QString::number(info.index) == index) {
                        *dev_info = info;
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

void MainWindow::setupUi(const OpenParams &params)
{
    leftEnabled = cam.IsStreamDataEnabled(ImageType::IMAGE_LEFT_COLOR);
    rightEnabled= cam.IsStreamDataEnabled(ImageType::IMAGE_RIGHT_COLOR);
    depthEnabled = cam.IsStreamDataEnabled(ImageType::IMAGE_DEPTH) &&
            params.depth_mode == DepthMode::DEPTH_COLORFUL;

    qint32 width, height;
    if (params.stream_mode == StreamMode::STREAM_640x480 ||
            params.stream_mode == StreamMode::STREAM_1280x480) {
        width = 640;
        height = 480;
    } else if (params.stream_mode == StreamMode::STREAM_1280x720 ||
               params.stream_mode == StreamMode::STREAM_2560x720) {
        width = 1280;
        height = 720;
    } else {
        QMessageBox::warning(this,
                             tr("Error"),
                             tr("Unaccepted stream mode"),
                             QMessageBox::Ok);
        closeLater();
        return;
    }

    QWidget *widget = ui->centralWidget;

    QGridLayout *layout = new QGridLayout(widget);
    layout->setMargin(0);
    layout->setSpacing(0);
    layout->setSizeConstraint(QLayout::SetFixedSize);

    if (leftEnabled) {
        leftLabel = new QLabel(widget);
        leftLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        leftLabel->setFixedSize(width, height);
        layout->addWidget(leftLabel, 0, 0);
    }
    if (rightEnabled) {
        rightLabel = new QLabel(widget);
        rightLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        rightLabel->setFixedSize(width, height);
        layout->addWidget(rightLabel, 0, 1);
    }
    if (depthEnabled) {
        depthLabel = new QLabel(widget);
        depthLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        depthLabel->setFixedSize(width, height);
        layout->addWidget(depthLabel, 1, 0);
    }

    widget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    widget->setFixedSize(width * 2, height * 2);

    QLabel *statusLabel = new QLabel(this);
    statusLabel->setText(QStringList({
        cam.GetDescriptor(Descriptor::DEVICE_NAME).c_str(),
        QString("SN: ").append(cam.GetDescriptor(Descriptor::SERIAL_NUMBER).c_str()),
        QString("FV: ").append(cam.GetDescriptor(Descriptor::FIRMWARE_VERSION).c_str()),
    }).join(", "));
    ui->statusBar->addWidget(statusLabel);

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(1000 / params.framerate);
}

void MainWindow::drawImage(QImage &image)
{
    QPen pen(Qt::SolidLine);
    pen.setColor(Qt::white);
    pen.setWidth(2);

    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(pen);

    QRect rect(0, 0, image.width(), image.height());
    painter.drawText(rect.adjusted(6, 6, -6, -6),
                     QString("%1x%2").arg(image.width()).arg(image.height()));

    //painter.drawRect(rect.adjusted(6, 6, -6, -6));
}

void MainWindow::drawDatas(std::vector<MotionData> datas) {
    QWidget *widget = ui->centralWidget;

    QSize size = widget->size();
    qint32 x = size.width() / 2;
    qint32 y = size.height() / 2;

    QRect rect(widget->mapToParent(QPoint(x, y)), QSize(x, y));
    rect.adjust(6, 6, -6, -6);

    QPen pen(Qt::SolidLine);
    pen.setColor(Qt::black);
    pen.setWidth(2);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(pen);

    std::size_t n = datas.size();

    QRect boundingRect;
    painter.drawText(rect, Qt::AlignLeft,
                     QString("num: %1").arg(n), &boundingRect);

    std::shared_ptr<ImuData> accel = nullptr;
    std::shared_ptr<ImuData> gyro = nullptr;
    for (auto &&data : datas) {
        if (!data.imu) continue;

        if (data.imu->flag == MYNTEYE_IMU_ACCEL && !accel) {
            accel = data.imu;
        } else if (data.imu->flag == MYNTEYE_IMU_GYRO && !gyro) {
            gyro = data.imu;
        } else {
            continue;
        }

        if (accel && gyro) break;

        //rect.adjust(0, boundingRect.height(), 0, 0);
        //painter.drawText(rect, Qt::AlignLeft, s, &boundingRect);
    }

    if (accel) {
        QString s = QString("[accel] stamp: %1\n  x: %2\n  y: %3\n  z: %4\n  temp: %5")
                .arg(accel->timestamp)
                .arg(accel->accel[0])
                .arg(accel->accel[1])
                .arg(accel->accel[2])
                .arg(accel->temperature);
        rect.adjust(0, boundingRect.height() + 6, 0, 0);
        painter.drawText(rect, Qt::AlignLeft | Qt::TextWordWrap, s, &boundingRect);
    }
    if (gyro) {
        QString s = QString("[gyro] stamp: %1\n  x: %2\n  y: %3\n  z: %4\n  temp: %5")
                .arg(gyro->timestamp)
                .arg(gyro->gyro[0])
                .arg(gyro->gyro[1])
                .arg(gyro->gyro[2])
                .arg(gyro->temperature);
        rect.adjust(0, boundingRect.height() + 6, 0, 0);
        painter.drawText(rect, Qt::AlignLeft | Qt::TextWordWrap, s, &boundingRect);
    }

    //painter.drawRect(rect);
}
