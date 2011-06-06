include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

include_directories(
	${IGVC_SOURCE_DIR}/gps_jeanni
	/usr/include/qt4
	/usr/include/qt4/Qt
	/usr/include/qt4/QtGui
	/usr/include/qt4/QtCore
	/usr/include/qt4/QtXml
)

add_library(gps_vision
	CVcam.cpp
	DCam.cpp
	image_buffers.cpp
	vision.cc
	DCam_Config.cpp
	GuppyCam.cpp
	PjMat.cc
)

target_link_libraries(gps_vision
QtCore
)

