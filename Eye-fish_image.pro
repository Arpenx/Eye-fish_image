TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    xp.cpp

INCLUDEPATH += -I /usr/local/opencv/340/include \
                /usr/local/opencv/340/include/opencv \
                /usr/local/opencv/340/include/opencv2

LIBS += -L /usr/local/opencv/340/lib/ -lopencv_highgui -lopencv_imgproc -lopencv_core -lopencv_imgcodecs

HEADERS += \
    xp.h

DISTFILES += \
    camera.bmp
