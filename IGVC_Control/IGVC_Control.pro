#-------------------------------------------------
#
# Project created by QtCreator 2013-07-31T18:57:25
#
#-------------------------------------------------

QT       += core gui declarative xml

TARGET = IGVC_Control
TEMPLATE = app

CONFIG += c++11

INCLUDEPATH += ../src/ \
    ../src/gui/

SOURCES += \
    ../src/common/config/configmanager.cpp \
    ../src/common/logger/logger.cpp \
    ../src/gui/systemstatusindicator.cpp \
    ../src/gui/mdiwindow.cpp \
    ../src/gui/mainwindow.cpp \
    ../src/gui/main.cpp \
    ../src/gui/configtreemodel.cpp \
    ../src/gui/batteryindicator.cpp \
    ../src/hardware/sensors/gps/nmea.cpp \
    ../src/hardware/sensors/joystick/Joystick.cpp \
    ../src/hardware/sensors/lidar/SimulatedLidar.cpp \
    ../src/hardware/sensors/lidar/NAV200.cpp \
    ../src/hardware/serial/ASIOSerialPort.cpp \
    ../src/gui/adapters/joystickadapter.cpp \
    ../src/gui/adapters/cameraadapter.cpp \
    ../src/hardware/actuators/motors/MotorEncoderDriver2013.cpp \
    ../src/common/utils/StringUtils.cpp \
    ../src/intelligence/pathplanning/searchlocation.cpp \
    ../src/intelligence/pathplanning/searchmove.cpp \
    ../src/intelligence/pathplanning/igvcsearchproblem.cpp \
    ../src/gui/adapters/lidaradapter.cpp \
    ../src/hardware/sensors/IMU/Ardupilot.cpp \
    ../src/gui/adapters/mapadapter.cpp \
    ../src/hardware/sensors/gps/simulatedgps.cpp \
    ../src/common/utils/ImageUtils.cpp \
    ../src/hardware/sensors/camera/CameraInfo.cpp \
    ../src/gui/adapters/gpsadapter.cpp \
    ../src/hardware/sensors/camera/StereoPlayback.cpp \
    ../src/hardware/sensors/camera/StereoImageRepeater.cpp \
    ../src/hardware/sensors/camera/Bumblebee2.cpp \
    ../src/gui/adapters/imuadapter.cpp \
    ../src/hardware/sensors/lidar/lms200.cpp \
    ../src/intelligence/linedetection/linedetector.cpp \
    ../src/intelligence/posetracking/positiontracker.cpp \
    ../src/common/utils/GPSWaypointSource.cpp \
    ../src/common/utils/gpsfilereader.cpp \
    ../src/hardware/sensors/gps/nmeacompatiblegps.cpp \
    ../src/intelligence/posetracking/basicpositiontracker.cpp \
    ../src/gui/adapters/positiontrackeradapter.cpp \
    ../src/intelligence/mapping/mapbuilder.cpp \
    ../src/intelligence/controller/controller.cpp \
    ../src/intelligence/pathplanning/astarplanner.cpp \
    ../src/gui/adapters/pathadapter.cpp \
    ../src/hardware/actuators/lights/lightcontroller.cpp \
    ../src/gui/adapters/lightshieldadapter.cpp

HEADERS  += \
    ../src/common/config/configmanager.h \
    ../src/common/logger/logger.h \
    ../src/common/events/Event.hpp \
    ../src/common/events/Delegate.hpp \
    ../src/gui/systemstatusindicator.h \
    ../src/gui/mdiwindow.h \
    ../src/gui/mainwindow.h \
    ../src/gui/configtreemodel.h \
    ../src/gui/batteryindicator.h \
    ../src/hardware/sensors/gps/nmea.hpp \
    ../src/hardware/sensors/gps/GPS.hpp \
    ../src/hardware/sensors/joystick/Joystick.h \
    ../src/hardware/sensors/lidar/SimulatedLidar.h \
    ../src/hardware/sensors/lidar/NAV200.h \
    ../src/hardware/sensors/lidar/Lidar.h \
    ../src/common/datastructures/VisOdomData.hpp \
    ../src/common/datastructures/StereoImageData.hpp \
    ../src/common/datastructures/SensorData.hpp \
    ../src/common/datastructures/IMUData.hpp \
    ../src/common/datastructures/ImageData.hpp \
    ../src/common/datastructures/GPSData.hpp \
    ../src/common/datastructures/DataPoint.hpp \
    ../src/common/datastructures/DataArray.hpp \
    ../src/hardware/serial/ASIOSerialPort.h \
    ../src/gui/adapters/joystickadapter.h \
    ../src/gui/adapters/cameraadapter.h \
    ../src/hardware/actuators/motors/MotorEncoderDriver2013.h \
    ../src/hardware/actuators/motors/MotorDriver.hpp \
    ../src/common/utils/StringUtils.hpp \
    ../src/intelligence/JoystickDriver.hpp \
    ../src/intelligence/pathplanning/SearchProblem.hpp \
    ../src/intelligence/pathplanning/GraphSearch.hpp \
    ../src/intelligence/pathplanning/searchlocation.h \
    ../src/intelligence/pathplanning/searchmove.h \
    ../src/intelligence/pathplanning/igvcsearchproblem.h \
    ../src/gui/adapters/lidaradapter.h \
    ../src/hardware/sensors/IMU/Ardupilot.h \
    ../src/gui/adapters/mapadapter.h \
    ../src/hardware/sensors/gps/simulatedgps.h \
    ../src/hardware/sensors/IMU/IMU.h \
    ../src/common/utils/ImageUtils.h \
    ../src/hardware/sensors/camera/CameraInfo.h \
    ../src/gui/adapters/gpsadapter.h \
    ../src/hardware/sensors/camera/StereoSource.hpp \
    ../src/hardware/sensors/camera/StereoPlayback.h \
    ../src/hardware/sensors/camera/StereoPair.hpp \
    ../src/hardware/sensors/camera/StereoImageRepeater.h \
    ../src/hardware/sensors/camera/Bumblebee2.h \
    ../src/gui/adapters/imuadapter.h \
    ../src/hardware/sensors/lidar/lms200.h \
    ../src/intelligence/linedetection/linedetector.h \
    ../src/intelligence/posetracking/positiontracker.h \
    ../src/common/utils/gaussianvariable.hpp \
    ../src/common/datastructures/GPSData.hpp \
    ../src/common/utils/GPSUtils.h \
    ../src/common/utils/AngleUtils.h \
    ../src/common/datastructures/GPSData.hpp \
    ../src/common/utils/GPSWaypointSource.h \
    ../src/common/utils/gpsfilereader.h \
    ../src/hardware/sensors/gps/nmeacompatiblegps.h \
    ../src/intelligence/posetracking/basicpositiontracker.h \
    ../src/common/datastructures/robotposition.hpp \
    ../src/gui/adapters/positiontrackeradapter.h \
    ../src/intelligence/mapping/mapbuilder.h \
    ../src/intelligence/controller/controller.h \
    ../src/intelligence/pathplanning/pathplanner.hpp \
    ../src/intelligence/pathplanning/astarplanner.h \
    ../src/gui/adapters/pathadapter.h \
    ../src/hardware/actuators/lights/lightcontroller.h \
    ../src/gui/adapters/lightshieldadapter.h

FORMS    += \
    ../src/gui/mainwindow.ui \
    ../src/gui/adapters/joystickadapter.ui \
    ../src/gui/adapters/lidaradapter.ui \
    ../src/gui/adapters/mapadapter.ui \
    ../src/gui/adapters/cameraadapter.ui \
    ../src/gui/adapters/gpsadapter.ui \
    ../src/gui/adapters/imuadapter.ui \
    ../src/gui/adapters/positiontrackeradapter.ui \
    ../src/gui/adapters/pathadapter.ui \
    ../src/gui/adapters/lightshieldadapter.ui

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

# BOOST

LIBS += -L/usr/lib/ -lboost_thread -lboost_system

# PCL
INCLUDEPATH += /usr/include/pcl-1.7
DEPENDPATH += /usr/include/pcl-1.7

LIBS += -L/usr/lib -lpcl_common -lpcl_visualization -lpcl_kdtree -lpcl_filters -lpcl_search

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
