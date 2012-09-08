include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

include_directories(
	${IGVC_SOURCE_DIR}/roxi
	${IGVC_SOURCE_DIR}/roxi/vision
	/usr/include/qt4
	/usr/include/qt4/Qt
	/usr/include/qt4/QtGui
	/usr/include/qt4/QtCore
	/usr/include/qt4/QtXml
)

add_library(potentialfields
	potentialfields.cc
	commonVecOps.cc
	pfdebug.cc
)

target_link_libraries(potentialfields
	gps
	xml
	roxi_vision
	opencv_core
	opencv_highgui
	opencv_imgproc
)

add_executable(potentialfieldstest
	test.cc
)

target_link_libraries(potentialfieldstest
	potentialfields
	opencv_core
	QtCore
)
