#-------------------------------------------------
#
# Project created by QtCreator 2018-02-25T09:58:50
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ImageProcess
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        mainwindow.cpp \
    cynsinglecirclesubpixeledge.cpp \
    matchimage.cpp \
    mainwindow.cpp

HEADERS += \
        mainwindow.h \
    cynsinglecirclesubpixeledge.h \
    matchimage.h \
    mainwindow.h

#OpenCV动态链接库
INCLUDEPATH += c:\opencv\build\include
INCLUDEPATH += c:\opencv\build\include\opencv\
INCLUDEPATH += c:\opencv\build\include\opencv2\

CONFIG(debug,debug|release) {
LIBS += -Lc:\opencv\build\x86\vc12\lib \
    -lopencv_core2413d \
    -lopencv_highgui2413d \
    -lopencv_imgproc2413d \
    -lopencv_features2d2413d \
    -lopencv_calib3d2413d \
    -lopencv_nonfree2413d
} else {
LIBS += -Lc:\opencv\build\x86\vc12\lib \
    -lopencv_core2413 \
    -lopencv_highgui2413 \
    -lopencv_imgproc2413 \
    -lopencv_features2d2413 \
    -lopencv_calib3d2413 \
    -lopencv_nonfree2413
}
#-------------------------------------------------

FORMS += \
    mainwindow.ui
