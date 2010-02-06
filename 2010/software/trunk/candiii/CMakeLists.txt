add_subdirectory(vision)
add_subdirectory(mapping)

include_directories(
	/home/nebousuru/cv2/include
	${IGVC_SOURCE_DIR}/candiii/vision
	${IGVC_SOURCE_DIR}/candiii/mapping
	/usr/include/qt4
	/usr/include/qt4/Qt
	/usr/include/qt4/QtGui
	/usr/include/qt4/QtCore
	/usr/include/qt4/QtXml
	${IGVC_SOURCE_DIR}/motors/osmc/PC_test
	${IGVC_SOURCE_DIR}/arduino
)

link_directories(
	/home/nebousuru/cv2/lib
)

#SET(CMAKE_C_FLAGS  " -fopenmp -g3 -O0 -Wall -W -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED")
#SET(CMAKE_CXX_FLAGS "-fopenmp -g3 -O0 -Wall -W -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED")

SET(CMAKE_C_FLAGS  " -fopenmp -g -O2 -Wall -W -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED")
SET(CMAKE_CXX_FLAGS "-fopenmp -g -O2 -Wall -W -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED")

add_executable(candiii
	main.cpp
	moc_DCam_Config.cpp
	Robot.cpp
	XmlConfiguration.cpp
)

target_link_libraries(candiii
	mapping
	vision
	motors_old
	ArduinoInterface
	OSMC_driver

	ltdl
	m
	GL
	dc1394
	glut
	cv
	cvaux
	highgui
	QtXml
	QtGui
	QtCore
	pthread
)
