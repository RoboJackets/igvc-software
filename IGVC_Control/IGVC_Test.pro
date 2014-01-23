
QT       += testlib

QT       -= gui

TARGET = IGVC_Test
CONFIG   += console
CONFIG   -= app_bundle

CONFIG += c++11

INCLUDEPATH += ../src/ \
    ../src/gui/

TEMPLATE = app

DEFINES += SRCDIR=\\\"$$PWD/\\\"

SOURCES += ../src/tests/testmain.cpp \
    ../src/common/utils/StringUtils.cpp

HEADERS += ../src/tests/teststringutils.hpp \
    ../src/common/utils/StringUtils.hpp

