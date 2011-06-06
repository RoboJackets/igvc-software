include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

FIND_PACKAGE( Qt4 REQUIRED )
QT4_WRAP_CPP(MOC_SRCS 
vision/DCam_Config.h
)

include_directories(

	#/home/robojackets/OpenCV_2.1.0_install/include

	${IGVC_SOURCE_DIR}/gps_jeanni/vision
	${IGVC_SOURCE_DIR}/gps_jeanni/mapping
	/usr/include/qt4
	/usr/include/qt4/Qt
	/usr/include/qt4/QtGui
	/usr/include/qt4/QtCore
	/usr/include/qt4/QtXml
	${IGVC_SOURCE_DIR}/motors/osmc/PC_test
	${IGVC_SOURCE_DIR}/motors/osmc/PC_4wd
	${IGVC_SOURCE_DIR}/arduino
	${IGVC_SOURCE_DIR}/encoders/pcdriver
	${IGVC_SOURCE_DIR}/encoders/pcdriver_signed
	${IGVC_SOURCE_DIR}/lidar
	${IGVC_SOURCE_DIR}/joystick
	${IGVC_SOURCE_DIR}/gps
)

link_directories(
#/home/robojackets/OpenCV_2.1.0_install/lib
)

add_subdirectory(vision)
add_subdirectory(mapping)

IF((${ROBOT_REV} MATCHES "candiiidbg") OR (${ROBOT_REV} MATCHES "candiii"))
	set(robot_bin gps_candiii)
ELSEIF((${ROBOT_REV} MATCHES "jeannidbg") OR (${ROBOT_REV} MATCHES "jeanni"))
	set(robot_bin gps_jeanni)
ENDIF((${ROBOT_REV} MATCHES "candiiidbg") OR (${ROBOT_REV} MATCHES "candiii"))

add_executable(${robot_bin}
	main.cpp
	#moc_DCam_Config.cpp
	${MOC_SRCS}	
	Robot.cpp
	XmlConfiguration.cpp
)

IF((${ROBOT_REV} MATCHES "candiiidbg") OR (${ROBOT_REV} MATCHES "candiii"))
target_link_libraries(${robot_bin}
	gps_mapping
	gps_vision
	gps

	ArduinoInterface
	OSMC_driver

	m
	GL
	GLU
	glut

	dc1394

	cv
	cvaux
	highgui

	QtXml
	QtGui
	QtCore

	pthread
	boost_thread-mt

	joystick

	#NAV200
)
ELSEIF((${ROBOT_REV} MATCHES "jeannidbg") OR (${ROBOT_REV} MATCHES "jeanni"))
target_link_libraries(${robot_bin}
	gps_mapping
	gps_vision
	gps

	ArduinoInterface
	OSMC_driver_4wd

	m
	GL
	GLU
	glut

	dc1394

	cv
	cvaux
	highgui

	QtXml
	QtGui
	QtCore

	pthread
	boost_thread-mt

	joystick
	#NAV200
)
ENDIF((${ROBOT_REV} MATCHES "candiiidbg") OR (${ROBOT_REV} MATCHES "candiii"))
