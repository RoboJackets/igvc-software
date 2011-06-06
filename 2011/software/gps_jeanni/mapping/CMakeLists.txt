include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

include_directories(
	${IGVC_SOURCE_DIR}/gps_jeanni
	${IGVC_SOURCE_DIR}/gps_jeanni/vision
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
