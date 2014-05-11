
QT       += testlib xml core gui declarative serialport

TARGET = IGVC_Test
CONFIG   += console
CONFIG   -= app_bundle

CONFIG += c++11

INCLUDEPATH += ../src/ \
    ../src/gui/

TEMPLATE = app

DEFINES += SRCDIR=\\\"$$PWD/\\\"

SOURCES += ../src/tests/testmain.cpp \
    ../src/common/utils/StringUtils.cpp \
    ../src/common/logger/logger.cpp \
    ../src/intelligence/linedetection/linedetector.cpp \
    ../src/intelligence/linedetection/transformer.cpp \
    ../src/intelligence/posetracking/positiontracker.cpp \
    ../src/common/config/configmanager.cpp \
    ../src/common/utils/gpsfilereader.cpp \
    ../src/hardware/sensors/gps/nmeacompatiblegps.cpp \
    ../src/hardware/serial/ASIOSerialPort.cpp \
    ../src/hardware/sensors/gps/nmea.cpp \
    ../src/intelligence/posetracking/basicpositiontracker.cpp \
    ../src/intelligence/controller/controller.cpp \
    ../src/common/utils/GPSWaypointSource.cpp \
    ../src/hardware/sensors/gps/simulatedgps.cpp \
    ../src/hardware/sensors/IMU/Ardupilot.cpp \
    ../src/intelligence/pathplanning/astarplanner.cpp \
    ../src/intelligence/pathplanning/igvcsearchproblem.cpp \
    ../src/intelligence/pathplanning/searchlocation.cpp \
    ../src/intelligence/pathplanning/searchmove.cpp \
    ../src/hardware/actuators/lights/lightcontroller.cpp

HEADERS += ../src/tests/teststringutils.hpp \
    ../src/tests/testpositiontracker.hpp \
    ../src/tests/testgpsutils.h \
    ../src/tests/testgpsreader.hpp \
    ../src/tests/testangleutils.h \
    ../src/tests/capturegpsandpostracker.hpp \
    ../src/common/config/configmanager.h \
    ../src/common/logger/logger.h \
    ../src/common/utils/GPSUtils.h \
    ../src/common/utils/AngleUtils.h \
    ../src/common/utils/gpsfilereader.h \
    ../src/common/utils/StringUtils.hpp \
    ../src/intelligence/posetracking/positiontracker.h \
    ../src/intelligence/posetracking/basicpositiontracker.h \
    ../src/hardware/sensors/gps/nmeacompatiblegps.h \
    ../src/hardware/serial/ASIOSerialPort.h \
    ../src/hardware/sensors/gps/nmea.hpp \
    ../src/hardware/sensors/gps/GPS.hpp \
    ../src/tests/testlinedetection.hpp \
    ../src/intelligence/linedetection/linedetector.h \
    ../src/intelligence/linedetection/transformer.h \
    ../src/tests/testcontroller.h \
    ../src/intelligence/controller/controller.h \
    ../src/common/utils/GPSWaypointSource.h \
    ../src/hardware/sensors/gps/simulatedgps.h \
    ../src/tests/CaptureIMU.hpp \
    ../src/hardware/sensors/IMU/Ardupilot.h \
    ../src/hardware/sensors/IMU/IMU.h \
    ../src/tests/testastarplanner.hpp \
    ../src/intelligence/pathplanning/astarplanner.h \
    ../src/intelligence/pathplanning/GraphSearch.hpp \
    ../src/intelligence/pathplanning/igvcsearchproblem.h \
    ../src/intelligence/pathplanning/pathplanner.hpp \
    ../src/intelligence/pathplanning/searchlocation.h \
    ../src/intelligence/pathplanning/searchmove.h \
    ../src/intelligence/pathplanning/SearchProblem.hpp \
    ../src/hardware/actuators/lights/lightcontroller.h

INCLUDEPATH += /usr/include
DEPENDPATH += /usr/include

# libUSB (for LIDAR)

LIBS += -L/usr/lib/x86_64-linux-gnu/ -lusb-1.0

INCLUDEPATH += /usr/lib/x86_64-linux-gnu
DEPENDPATH += /usr/lib/x86_64-linux-gnu

# BOOST

LIBS += -L/usr/lib/ -lboost_thread -lboost_system

# PCL
INCLUDEPATH += /usr/include/pcl-1.7
DEPENDPATH += /usr/include/pcl-1.7

LIBS += -L/usr/lib -lpcl_common -lpcl_visualization -lpcl_kdtree -lpcl_io

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

LIBS += -L/usr/lib/ -lopencv_core -lopencv_imgproc -lopencv_calib3d -lopencv_highgui

# FlyCapture2
INCLUDEPATH += /usr/include/
DEPENDPATH += /usr/include/

LIBS += -L/usr/lib/ -lflycapture

#SICK Toolbox
#NOTE : Toolbox depends on pthread library
INCLUDEPATH += /usr/local/include
DEPENDPATH += /usr/local/include

LIBS += -L/usr/local/lib -lsicklms-1.0 -pthread
