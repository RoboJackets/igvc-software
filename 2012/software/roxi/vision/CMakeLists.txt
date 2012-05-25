include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)
FIND_PACKAGE( Qt4 REQUIRED )
QT4_WRAP_CPP(MOC_SRCS 
DCam_Config.h
)

include_directories(
	${IGVC_SOURCE_DIR}/roxi
	/usr/include/qt4
	/usr/include/qt4/Qt
	/usr/include/qt4/QtGui
	/usr/include/qt4/QtCore
	/usr/include/qt4/QtXml
)

add_subdirectory(perspective_transform)

add_library(roxi_vision
	${MOC_SRCS}	
	CVcam.cpp
	DCam.cpp
	image_buffers.cpp
	vision.cc
	DCam_Config.cpp
	GuppyCam.cpp
	PjMat.cc
)

target_link_libraries(roxi_vision
QtCore
)

