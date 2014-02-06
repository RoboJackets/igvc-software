
QT       += testlib core declarative gui

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
    ../src/common/utils/gpsfilereader.cpp

HEADERS += ../src/tests/teststringutils.hpp \
    ../src/common/utils/StringUtils.hpp \
    ../src/tests/testgpsutils.h \
    ../src/common/utils/GPSUtils.h \
    ../src/common/utils/AngleUtils.h \
    ../src/tests/testangleutils.h \
    ../src/tests/testgpsreader.hpp \
    ../src/common/logger/logger.h \
    ../src/common/utils/gpsfilereader.h

