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

CONFIG += c++11

INCLUDEPATH += ../src/ \
    ../src/gui/

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
    ../src/hardware/serial/ASIOSerialPort.cpp \
    ../src/gui/adapters/joystickadapter.cpp \
    ../src/hardware/actuators/motors/MotorEncoderDriver2013.cpp \
    ../src/common/utils/StringUtils.cpp \
    ../src/intelligence/pathplanning/searchlocation.cpp \
    ../src/intelligence/pathplanning/searchmove.cpp \
    ../src/intelligence/pathplanning/igvcsearchproblem.cpp \
    ../src/hardware/sensors/IMU/Ardupilot.cpp \
    ../src/gui/adapters/mapadapter.cpp \
    ../src/intelligence/mapping/mapping.cpp \
    ../src/hardware/sensors/gps/simulatedgps.cpp \
    ../src/intelligence/posetracking/RobotPosition.cpp \
    ../src/common/utils/ImageUtils.cpp \
    ../src/common/Robot.cpp \
    ../src/hardware/sensors/camera/CameraInfo.cpp

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
    ../src/hardware/serial/ASIOSerialPort.h \
    ../src/gui/adapters/joystickadapter.h \
    ../src/hardware/actuators/motors/MotorEncoderDriver2013.h \
    ../src/hardware/actuators/motors/MotorDriver.hpp \
    ../src/common/utils/StringUtils.hpp \
    ../src/intelligence/JoystickDriver.hpp \
    ../src/intelligence/pathplanning/SearchProblem.hpp \
    ../src/intelligence/pathplanning/GraphSearch.hpp \
    ../src/intelligence/pathplanning/searchlocation.h \
    ../src/intelligence/pathplanning/searchmove.h \
    ../src/intelligence/pathplanning/igvcsearchproblem.h \
    ../src/hardware/sensors/IMU/Ardupilot.h \
    ../src/gui/adapters/mapadapter.h \
    ../src/intelligence/mapping/mapping.h \
    ../src/hardware/sensors/gps/simulatedgps.h \
    ../src/hardware/sensors/IMU/IMU.h \
    ../src/intelligence/posetracking/RobotPosition.h \
    ../src/common/utils/ImageUtils.h \
    ../src/common/Robot.h \
    ../src/hardware/sensors/camera/CameraInfo.h

FORMS    += \
    ../src/gui/mainwindow.ui \
    ../src/gui/adapters/joystickadapter.ui \
    ../src/gui/adapters/mapadapter.ui

RESOURCES += \
    ../src/gui/resources.qrc

OTHER_FILES += \
    config.xml

INCLUDEPATH += /usr/include
DEPENDPATH += /usr/include

# libUSB (for LIDAR)

LIBS += -L/usr/lib/x86_64-linux-gnu/ -lusb-1.0

INCLUDEPATH += /usr/lib/x86_64-linux-gnu
DEPENDPATH += /usr/lib/x86_64-linux-gnu

PRE_TARGETDEPS += /usr/lib/x86_64-linux-gnu/libusb-1.0.a

# BOOST

LIBS += -L/usr/lib/ -lboost_thread -lboost_system

PRE_TARGETDEPS += /usr/lib/libboost_thread.a
PRE_TARGETDEPS += /usr/lib/libboost_system.a

# PCL
INCLUDEPATH += /usr/include/pcl-1.7
DEPENDPATH += /usr/include/pcl-1.7

LIBS += -L/usr/lib -lpcl_common -lpcl_visualization -lpcl_kdtree

# VTK (PCL Dependency)
INCLUDEPATH += /usr/include/vtk-5.8
DEPENDPATH += /usr/include/vtk-5.8

LIBS += -L/usr/lib/ -lvtkCommon

# Eigen (header-only library)
INCLUDEPATH += /usr/include/eigen3
DEPENDPATH += /usr/include/eigen3

# OpenCV
INCLUDEPATH += /usr/include/
DEPENDPATH += /usr/include/

LIBS += -L/usr/lib/ -lopencv_core -lopencv_imgproc -lopencv_calib3d
