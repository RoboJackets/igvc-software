find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories( ${OGRE_INCLUDE_DIRS} )
link_directories( ${OGRE_LIBRARY_DIRS} )

add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME}
	eStop_panel.cpp
	speed_panel.cpp
	speedometer.cpp
	bat_panel.cpp
	time_panel.cpp
	launch_panel.cpp
	my_thread.cpp
	node_panel.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES} ${OGRE_LIBRARY_DIRS})