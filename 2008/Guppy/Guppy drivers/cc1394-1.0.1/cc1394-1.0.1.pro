TEMPLATE	= app
LANGUAGE	= C++

CONFIG	+= thread warn_on nodebug

LIBS	+= -L /usr/lib -lqui /usr/lib/libraw1394.a /usr/lib/libdc1394_control.a /usr/lib/libavt1394.a -o cc1394

DEFINES	+= MESSAGELOGGER

INCLUDEPATH	+= /usr/include

HEADERS	+= F7BPPChangedEvent.h \
	videoPanel.h \
	constants.h \
	CamWindow.h \
	DC1394.h \
	CamWindowCloseEvent.h \
	AdjustmentsCloseEvent.h \
	F7NotifyData.h \
	CustomEvents.h \
	F7UpdateNotifyEvent.h \
	UpdateF7Event.h \
	VPanelCloseEvent.h \
	version.h \
	seqCtlElement.h

SOURCES	+= main.cpp \
	F7BPPChangedEvent.cpp \
	videoPanel.cpp \
	CamWindow.cpp \
	DC1394.cpp \
	CamWindowCloseEvent.cpp \
	AdjustmentsCloseEvent.cpp \
	F7UpdateNotifyEvent.cpp \
	UpdateF7Event.cpp \
	VPanelCloseEvent.cpp

FORMS	= mainform.ui \
	adjustmentsform.ui \
	caminfoform.ui \
	siodemoform.ui \
	seqctldemoform.ui \
	messagelogger.ui \
	SoftResetDelay.ui \
	drmform.ui

IMAGES	= images/filenew \
	images/fileopen \
	images/filesave \
	images/print \
	images/undo \
	images/redo \
	images/editcut \
	images/editcopy \
	images/editpaste \
	images/searchfind \
	images/filenew_1 \
	images/fileopen_1 \
	images/filesave_1 \
	images/print_1 \
	images/undo_1 \
	images/redo_1 \
	images/editcut_1 \
	images/editcopy_1 \
	images/editpaste_1 \
	images/searchfind_1

unix {
  UI_DIR = .ui
  MOC_DIR = .moc
  OBJECTS_DIR = .obj
}





