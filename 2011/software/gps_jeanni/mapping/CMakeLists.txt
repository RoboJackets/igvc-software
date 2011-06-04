
IF((${ROBOT_REV} MATCHES "candiiidbg") OR (${ROBOT_REV} MATCHES "jeannidbg"))
	SET(CMAKE_C_FLAGS "-march=native -pipe -g3 -O0 -Wall -Wextra -pthread -DQT_THREAD_SUPPORT")
	SET(CMAKE_CXX_FLAGS "-march=native -pipe -g3 -O0 -Wall -Wextra -pthread -DQT_THREAD_SUPPORT")
ELSEIF( (${ROBOT_REV} MATCHES "candiii") OR (${ROBOT_REV} MATCHES "jeanni"))
	SET(CMAKE_C_FLAGS "-march=native -pipe -g3 -O3 -Wall -Wextra -Wuninitialized -Werror=uninitialized -pthread -DQT_THREAD_SUPPORT -fopenmp")
	SET(CMAKE_CXX_FLAGS "-march=native -pipe -g3 -O3 -Wall -Wextra -Wuninitialized -Werror=uninitialized -pthread -DQT_THREAD_SUPPORT -fopenmp")
ENDIF((${ROBOT_REV} MATCHES "candiiidbg") OR (${ROBOT_REV} MATCHES "jeannidbg"))

include_directories(
	${IGVC_SOURCE_DIR}/jeanni
	${IGVC_SOURCE_DIR}/jeanni/vision
	/usr/include/opencv
	/usr/include/qt4
	/usr/include/qt4/Qt
	/usr/include/qt4/QtGui
	/usr/include/qt4/QtCore
	/usr/include/qt4/QtXml
)

add_library(gps_mapping
	cvcorrImages.cpp
	mapgen.cpp
	PointParamEstimator.cpp
)
