include_directories(
	${IGVC_SOURCE_DIR}/jeanni
	/usr/include/qt4
	/usr/include/qt4/Qt
	/usr/include/qt4/QtGui
	/usr/include/qt4/QtCore
	/usr/include/qt4/QtXml
)


IF((${ROBOT_REV} MATCHES "candiiidbg") OR (${ROBOT_REV} MATCHES "jeanidbg"))
	SET(CMAKE_C_FLAGS  " -march=native -pipe -g3 -O0 -Wall -Wextra -pthread -DQT_THREAD_SUPPORT")
	SET(CMAKE_CXX_FLAGS " -march=native -pipe -g3 -O0 -Wall -Wextra -pthread -DQT_THREAD_SUPPORT")
ELSEIF( (${ROBOT_REV} MATCHES "candiii") OR (${ROBOT_REV} MATCHES "jeani"))
	SET(CMAKE_C_FLAGS  " -march=native -pipe -g3 -O3 -Wall -Wextra -Wuninitialized -Werror=uninitialized -pthread -DQT_THREAD_SUPPORT -fopenmp")
	SET(CMAKE_CXX_FLAGS " -march=native -pipe -g3 -O3 -Wall -Wextra -Wuninitialized -Werror=uninitialized -pthread -DQT_THREAD_SUPPORT -fopenmp")
ENDIF((${ROBOT_REV} MATCHES "candiiidbg") OR (${ROBOT_REV} MATCHES "jeanidbg"))

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

