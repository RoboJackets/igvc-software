find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
find_package(catkin REQUIRED COMPONENTS rviz)
include(${QT_USE_FILE})

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(${catkin_INCLUDE_DIRS})
link_directories( ${catkin_LIBRARY_DIRS} )
include_directories( ${OGRE_INCLUDE_DIRS} )
link_directories( ${OGRE_LIBRARY_DIRS} )

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
	eStop_panel.h
	speed_panel.h
	speedometer.h
	bat_panel.h
	time_panel.h
	launch_panel.h
	my_thread.h
	node_panel.h
	num_button.h
)

set(SOURCE_FILES
	eStop_panel.cpp
	speed_panel.cpp
	speedometer.cpp
	bat_panel.cpp
	time_panel.cpp
	launch_panel.cpp
	my_thread.cpp
	node_panel.cpp
	num_button.cpp
	${MOC_FILES})

add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES} ${OGRE_LIBRARY_DIRS})