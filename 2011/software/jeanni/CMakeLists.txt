

include_directories(

	#/home/robojackets/OpenCV_2.1.0_install/include

	${IGVC_SOURCE_DIR}/jeanni/vision
	${IGVC_SOURCE_DIR}/jeanni/mapping
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
)

link_directories(
#/home/robojackets/OpenCV_2.1.0_install/lib
)

add_subdirectory(vision)
add_subdirectory(mapping)

IF(${ROBOT_REV} MATCHES "candiiidbg")
	SET(CMAKE_C_FLAGS  " -march=native -pipe -fopenmp -g3 -O0 -Wall -Wextra -Wuninitialized -Werror=uninitialized -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED -DOSMC_2WD")
	SET(CMAKE_CXX_FLAGS "-march=native -pipe -fopenmp -g3 -O0 -Wall -Wextra -Wuninitialized -Werror=uninitialized -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED -DOSMC_2WD")
ELSEIF(${ROBOT_REV} MATCHES "candiii")
	SET(CMAKE_C_FLAGS  " -march=native -pipe -fopenmp -g3 -O3 -Wall -Wextra -Wuninitialized -Werror=uninitialized -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED -DOSMC_2WD")
	SET(CMAKE_CXX_FLAGS "-march=native -pipe -fopenmp -g3 -O3 -Wall -Wextra -Wuninitialized -Werror=uninitialized -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED -DOSMC_2WD")
ELSEIF(${ROBOT_REV} MATCHES "jeannidbg")
	SET(CMAKE_C_FLAGS  "  -march=native -pipe -fopenmp -g3 -O0 -Wall -Wextra -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED -DOSMC_4WD")
	SET(CMAKE_CXX_FLAGS " -march=native -pipe -fopenmp -g3 -O0 -Wall -Wextra -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED -DOSMC_4WD")
ELSEIF(${ROBOT_REV} MATCHES "jeanni")
	SET(CMAKE_C_FLAGS  "  -march=native -pipe -fopenmp -g3 -O3 -Wall -Wextra -Wuninitialized -Werror=uninitialized -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED -DOSMC_4WD")
	SET(CMAKE_CXX_FLAGS " -march=native -pipe -fopenmp -g3 -O3 -Wall -Wextra -Wuninitialized -Werror=uninitialized -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED -DOSMC_4WD")
ENDIF(${ROBOT_REV} MATCHES "candiiidbg")



IF((${ROBOT_REV} MATCHES "candiiidbg") OR (${ROBOT_REV} MATCHES "candiii"))
	set(robot_bin candiii)
ELSEIF((${ROBOT_REV} MATCHES "jeannidbg") OR (${ROBOT_REV} MATCHES "jeanni"))
	set(robot_bin jeanni)
ENDIF((${ROBOT_REV} MATCHES "candiiidbg") OR (${ROBOT_REV} MATCHES "candiii"))

add_executable(${robot_bin}
	main.cpp
	moc_DCam_Config.cpp
	Robot.cpp
	XmlConfiguration.cpp
)

IF((${ROBOT_REV} MATCHES "candiiidbg") OR (${ROBOT_REV} MATCHES "candiii"))
target_link_libraries(${robot_bin}
	mapping
	vision

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
	mapping
	vision

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
