#find_package(Qt5Core REQUIRED)
#find_package(Qt5Gui REQUIRED)
#find_package(Qt5Widgets REQUIRED)
#find_package(catkin REQUIRED COMPONENTS rviz)
#
#
## Find includes in corresponding build directories
#set(CMAKE_INCLUDE_CURRENT_DIR ON)
#
#include_directories(${catkin_INCLUDE_DIRS})
#link_directories( ${catkin_LIBRARY_DIRS} )
#include_directories( ${OGRE_INCLUDE_DIRS} )
#link_directories( ${OGRE_LIBRARY_DIRS} )
#
#add_definitions(-DQT_NO_KEYWORDS)
#
#qt5_wrap_cpp(MOC_FILES
#  bat_panel.h
#  eStop_panel.h
#  example_panel.h
#  launch_panel.h
#  my_thread.h
#  node_panel.h
#  num_button.h
#  sensor_panel.h
#  speedometer.h
#  speed_panel.h
#  time_panel.h
#  waypoint_panel.h
#)
#
#set(SOURCE_FILES
#  bat_panel.cpp
#  eStop_panel.cpp
#  example_panel.cpp
#  launch_panel.cpp
#  my_thread.cpp
#  node_panel.cpp
#  num_button.cpp
#  sensor_panel.cpp
#  speedometer.cpp
#  speed_panel.cpp
#  time_panel.cpp
#  waypoint_panel.cpp
#  ${MOC_FILES})
#
#add_library(${PROJECT_NAME} ${SOURCE_FILES})
#add_dependencies(${PROJECT_NAME} igvc_msgs_gencpp)
#
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${OGRE_LIBRARY_DIRS})
