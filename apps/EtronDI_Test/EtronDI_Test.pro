#-------------------------------------------------
#
# Project created by QtCreator 2015-08-05T14:28:09
#
#-------------------------------------------------

QT += core gui

CONFIG += console

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = EtronDI_Test
TEMPLATE = app

SOURCES += main.cpp \
    mainwindow.cpp \
    videodevicedlg.cpp \
    videodevicethread.cpp \
    colordlg.cpp \
    depthdlg.cpp \
    bothdlg.cpp \
    propertyitems.cpp

HEADERS += mainwindow.h \
    videodevicedlg.h \
    videodevicethread.h \
    colordlg.h \
    depthdlg.h \
    bothdlg.h \
    propertyitems.h

FORMS += mainwindow.ui \
    dialog.ui

ESPDI_DIR = ../../3rdparty/eSPDI

INCLUDEPATH += $$ESPDI_DIR/include
INCLUDEPATH += /usr/include/x86_64-linux-gnu

#LIBS += -L$$ESPDI_DIR/linux/x64 -l:libeSPDI.so.3.0.14
LIBS += $$ESPDI_DIR/linux/x64/libeSPDI.so.3.0.14
LIBS += /usr/lib/x86_64-linux-gnu/libjpeg.so.8
LIBS += /usr/lib/x86_64-linux-gnu/libavcodec.so
LIBS += /usr/lib/x86_64-linux-gnu/libavformat.so
LIBS += /usr/lib/x86_64-linux-gnu/libavutil.so
LIBS += /usr/lib/x86_64-linux-gnu/libswscale.so
LIBS += /usr/lib/x86_64-linux-gnu/libswresample.so
