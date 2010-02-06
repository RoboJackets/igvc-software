include_directories(
	${IGVC_SOURCE_DIR}/candiii
	/usr/include/qt4
	/usr/include/qt4/Qt
	/usr/include/qt4/QtGui
	/usr/include/qt4/QtCore
	/usr/include/qt4/QtXml
)

SET(CMAKE_C_FLAGS  " -g3 -O0 -Wall -W -pthread -DQT_THREAD_SUPPORT -fopenmp")
SET(CMAKE_CXX_FLAGS " -g3 -O0 -Wall -W -pthread -DQT_THREAD_SUPPORT -fopenmp")

add_library(vision
	CVcam.cpp
	DCam.cpp
	image_buffers.cpp
	vision.cc
	DCam_Config.cpp
	GuppyCam.cpp
	PjMat.cc
)

target_link_libraries(vision
QtCore
)

