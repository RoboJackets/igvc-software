add_subdirectory(vision)
add_subdirectory(mapping)

include_directories(
	${IGVC_SOURCE_DIR}/candiii/vision
	${IGVC_SOURCE_DIR}/candiii/mapping
	/usr/include/qt4
	/usr/include/qt4/Qt
	/usr/include/qt4/QtGui
	/usr/include/qt4/QtCore
	/usr/include/qt4/QtXml
	${IGVC_SOURCE_DIR}/motors/osmc/PC_test
	${IGVC_SOURCE_DIR}/motors/osmc/PC_4wd
	${IGVC_SOURCE_DIR}/arduino
	${IGVC_SOURCE_DIR}/newencoders/pcdriver
)

IF(${ROBOT_REV} MATCHES "candiiidbg")
	SET(CMAKE_C_FLAGS  " -march=native -pipe -fopenmp -g3 -O0 -Wall -Wextra -Wuninitialized -Werror=uninitialized -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED -DOSMC_2WD")
	SET(CMAKE_CXX_FLAGS "-march=native -pipe -fopenmp -g3 -O0 -Wall -Wextra -Wuninitialized -Werror=uninitialized -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED -DOSMC_2WD")
ELSEIF(${ROBOT_REV} MATCHES "candiii")
	SET(CMAKE_C_FLAGS  " -march=native -pipe -fopenmp -g3 -O2 -Wall -Wextra -Wuninitialized -Werror=uninitialized -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED -DOSMC_2WD")
	SET(CMAKE_CXX_FLAGS "-march=native -pipe -fopenmp -g3 -O2 -Wall -Wextra -Wuninitialized -Werror=uninitialized -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED -DOSMC_2WD")
ELSEIF(${ROBOT_REV} MATCHES "jeanidbg")
	SET(CMAKE_C_FLAGS  "  -march=native -pipe -g3 -O0 -Wall -Wextra -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED -DOSMC_4WD")
	SET(CMAKE_CXX_FLAGS " -march=native -pipe -g3 -O0 -Wall -Wextra -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED -DOSMC_4WD")
ELSEIF(${ROBOT_REV} MATCHES "jeani")
	SET(CMAKE_C_FLAGS  "  -march=native -pipe -fopenmp -g3 -O2 -Wall -Wextra -Wuninitialized -Werror=uninitialized -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED -DOSMC_4WD")
	SET(CMAKE_CXX_FLAGS " -march=native -pipe -fopenmp -g3 -O2 -Wall -Wextra -Wuninitialized -Werror=uninitialized -pthread -D_REENTRANT -DQT_XML_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED -DOSMC_4WD")
ENDIF(${ROBOT_REV} MATCHES "candiiidbg")



IF((${ROBOT_REV} MATCHES "candiiidbg") OR (${ROBOT_REV} MATCHES "candiii"))
	set(robot_bin candiii)
ELSEIF((${ROBOT_REV} MATCHES "jeanidbg") OR (${ROBOT_REV} MATCHES "jeani"))
	set(robot_bin jeani)
ENDIF((${ROBOT_REV} MATCHES "candiiidbg") OR (${ROBOT_REV} MATCHES "candiii"))

add_executable(${robot_bin}
	main.cpp
	moc_DCam_Config.cpp
	Robot.cpp
	XmlConfiguration.cpp
)

target_link_libraries(${robot_bin}
	mapping
	vision
	motors_old
	ArduinoInterface
	OSMC_driver
	OSMC_driver_4wd

	#ltdl
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
)

