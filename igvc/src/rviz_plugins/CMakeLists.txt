# Find the QtWidgets library
find_package(Qt5Core 5.2 REQUIRED)
find_package(Qt5Gui 5.2 REQUIRED)
find_package(Qt5Widgets 5.2 REQUIRED)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(${PROJECT_NAME}
	eStop_panel.cpp
	speed_panel.cpp
	speedometer.cpp
	bat_panel.cpp
	time_panel.cpp
	launch_panel.cpp
	my_thread.cpp
	node_panel.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OGRE_LIBRARY_DIRS} ${Qt5Widgets_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES})