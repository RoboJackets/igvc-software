#SET(CMAKE_C_FLAGS "-g3 -O0 -Wall -Wextra -pthread -DQT_THREAD_SUPPORT -fopenmp")
#SET(CMAKE_CXX_FLAGS "-g3 -O0 -Wall -Wextra -pthread -DQT_THREAD_SUPPORT -fopenmp")

SET(CMAKE_C_FLAGS "-g3 -O0 -Wall -Wextra -pthread -DQT_THREAD_SUPPORT")
SET(CMAKE_CXX_FLAGS "-g3 -O0 -Wall -Wextra -pthread -DQT_THREAD_SUPPORT")

include_directories(
	${IGVC_SOURCE_DIR}/candiii
	${IGVC_SOURCE_DIR}/candiii/vision
	/usr/include/opencv
	/usr/include/qt4
	/usr/include/qt4/Qt
	/usr/include/qt4/QtGui
	/usr/include/qt4/QtCore
	/usr/include/qt4/QtXml
)

add_library(mapping
	cvcorrImages.cpp
	mapgen.cpp
	PointParamEstimator.cpp
)
