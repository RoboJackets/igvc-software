#-------------------------------------------------
#
# Project created by QtCreator 2013-02-15T17:17:39
#
#-------------------------------------------------

QT       += core gui

TARGET = LidarTestGUI
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    ../../sensors/lidar/NAV200.cpp \
    lidardisplaywidget.cpp \
    ../../sensors/lidar/SimulatedLidar.cpp \
    ../../mapping/extractors/lidarobstacleextractor.cpp \
    ../../mapping/obstacles/linearobstacle.cpp

INCLUDEPATH += ../../

HEADERS  += mainwindow.h \
    ../../sensors/lidar/NAV200.h \
    ../../sensors/lidar/Lidar.h \
    ../../events/Event.hpp \
    ../../events/Delegate.hpp \
    lidardisplaywidget.h \
    ../../sensors/lidar/SimulatedLidar.h \
    ../../mapping/extractors/lidarobstacleextractor.h \
    ../../mapping/obstacles/linearobstacle.h \
    ../../mapping/obstacles/obstacle.hpp

FORMS    += mainwindow.ui

LIBS += -lusb-1.0

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../../../usr/lib/release/ -lboost_system
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../../../usr/lib/debug/ -lboost_system
else:symbian: LIBS += -lboost_system
else:unix: LIBS += -L$$PWD/../../../../../../../../usr/lib/ -lboost_system

INCLUDEPATH += $$PWD/../../../../../../../../usr/include
DEPENDPATH += $$PWD/../../../../../../../../usr/include

win32:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../../../../usr/lib/release/boost_system.lib
else:win32:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../../../../usr/lib/debug/boost_system.lib
else:unix:!symbian: PRE_TARGETDEPS += $$PWD/../../../../../../../../usr/lib/libboost_system.a

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../../../usr/lib/release/ -lboost_thread
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../../../usr/lib/debug/ -lboost_thread
else:symbian: LIBS += -lboost_thread
else:unix: LIBS += -L$$PWD/../../../../../../../../usr/lib/ -lboost_thread

INCLUDEPATH += $$PWD/../../../../../../../../usr/include
DEPENDPATH += $$PWD/../../../../../../../../usr/include

win32:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../../../../usr/lib/release/boost_thread.lib
else:win32:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../../../../usr/lib/debug/boost_thread.lib
else:unix:!symbian: PRE_TARGETDEPS += $$PWD/../../../../../../../../usr/lib/libboost_thread.a

CONFIG += link_pkgconfig
PKGCONFIG += x11
