#-------------------------------------------------
#
# Project created by QtCreator 2013-07-31T18:57:25
#
#-------------------------------------------------

QT       += core gui
QT += xml

TARGET = IGVC_Control
TEMPLATE = app
QT += gui declarative

INCLUDEPATH += ../src/

SOURCES += \
    ../src/common/config/configmanager.cpp \
    ../src/common/logger/logger.cpp \
    ../src/common/events/EventGenerator.cpp \
    ../src/gui/systemstatusindicator.cpp \
    ../src/gui/mdiwindow.cpp \
    ../src/gui/mainwindow.cpp \
    ../src/gui/main.cpp \
    ../src/gui/configtreemodel.cpp \
    ../src/gui/batteryindicator.cpp \
    ../src/hardware/sensors/gps/nmea.cpp \
    ../src/hardware/sensors/gps/HemisphereA100GPS.cpp \
    ../src/hardware/sensors/joystick/Joystick.cpp \
    ../src/hardware/sensors/lidar/SimulatedLidar.cpp \
    ../src/hardware/sensors/lidar/NAV200.cpp \
    ../src/hardware/sensors/DataStructures/SensorData.cpp \
    ../src/hardware/sensors/DataStructures/GPSData.cpp \
    ../src/hardware/sensors/DataStructures/DataArray.cpp \
    ../src/hardware/serial/ASIOSerialPort.cpp

HEADERS  += \
    ../src/common/config/configmanager.h \
    ../src/common/logger/logger.h \
    ../src/common/events/EventGenerator.h \
    ../src/common/events/Event.hpp \
    ../src/common/events/Delegate.hpp \
    ../src/gui/systemstatusindicator.h \
    ../src/gui/mdiwindow.h \
    ../src/gui/mainwindow.h \
    ../src/gui/configtreemodel.h \
    ../src/gui/batteryindicator.h \
    ../src/hardware/sensors/gps/nmea.hpp \
    ../src/hardware/sensors/gps/HemisphereA100GPS.h \
    ../src/hardware/sensors/gps/GPS.hpp \
    ../src/hardware/sensors/joystick/Joystick.h \
    ../src/hardware/sensors/lidar/SimulatedLidar.h \
    ../src/hardware/sensors/lidar/NAV200.h \
    ../src/hardware/sensors/lidar/Lidar.h \
    ../src/hardware/sensors/DataStructures/VisOdomData.hpp \
    ../src/hardware/sensors/DataStructures/StereoImageData.hpp \
    ../src/hardware/sensors/DataStructures/SensorData.h \
    ../src/hardware/sensors/DataStructures/IMUData.hpp \
    ../src/hardware/sensors/DataStructures/ImageData.hpp \
    ../src/hardware/sensors/DataStructures/GPSData.h \
    ../src/hardware/sensors/DataStructures/GPSAccuracy.hpp \
    ../src/hardware/sensors/DataStructures/DataPoint.hpp \
    ../src/hardware/sensors/DataStructures/DataArray.hpp \
    ../src/hardware/serial/ASIOSerialPort.h

FORMS    += \
    ../src/gui/mainwindow.ui

RESOURCES += \
    ../src/gui/resources.qrc

OTHER_FILES += \
    config.xml

unix:!macx: LIBS += -L/usr/lib/ -lboost_system

INCLUDEPATH += /usr/include
DEPENDPATH += /usr/include

unix:!macx: PRE_TARGETDEPS += /usr/lib/libboost_system.a

unix:!macx: LIBS += -L/usr/lib/x86_64-linux-gnu/ -lusb-1.0

INCLUDEPATH += /usr/lib/x86_64-linux-gnu
DEPENDPATH += /usr/lib/x86_64-linux-gnu

unix:!macx: PRE_TARGETDEPS += /usr/lib/x86_64-linux-gnu/libusb-1.0.a

unix:!macx: LIBS += -L/usr/lib/ -lboost_thread

INCLUDEPATH += /usr/include
DEPENDPATH += /usr/include

unix:!macx: PRE_TARGETDEPS += /usr/lib/libboost_thread.a
