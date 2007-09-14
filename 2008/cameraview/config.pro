# This is a QT project file.
# Run `qmake` to generate a Makefile from this.

CONFIG += warn_on release

TEMPLATE = app
# name of executable
TARGET = cameraview

# compile objects to here
OBJECTS_DIR = ./bin
# put moc stuff here
MOC_DIR = ./bin

# include path extensions
INCLUDEPATH += /usr/local/include/player-2.0
LIBS += -L/usr/local/lib -lplayerc++ -lplayerc -lm -lplayerxdr -lplayererror 

# Files
HEADERS += *.h
SOURCES += *.cpp
