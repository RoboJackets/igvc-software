#-------------------------------------------------
#
# Project created by QtCreator
#
#-------------------------------------------------

QT       += testlib

QT       -= gui

TARGET = IGVC_Test
CONFIG   += console
CONFIG   -= app_bundle

CONFIG += c++11

INCLUDEPATH += ../src/ \
    ../src/gui/

TEMPLATE = app


#SOURCES += tst_qtestprojecttest.cpp
DEFINES += SRCDIR=\\\"$$PWD/\\\"

SOURCES += \
    teststringutils.cpp \
    ../src/common/utils/StringUtils.cpp

HEADERS += \
    ../src/common/utils/StringUtils.hpp

