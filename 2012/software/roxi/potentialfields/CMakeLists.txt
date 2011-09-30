add_library(potentialfields
	potentialfields.cc
	commonVecOps.cc
)

target_link_libraries(potentialfields
	gps
)

include_directories(
	${IGVC_SOURCE_DIR}/roxi
	/usr/include/qt4
	/usr/include/qt4/Qt
	/usr/include/qt4/QtGui
	/usr/include/qt4/QtCore
	/usr/include/qt4/QtXml
)
