cmake_minimum_required(VERSION 2.8.3)
project(igvc_rviz_plugins)

SET(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  igvc_msgs
)

find_package(Qt5Widgets REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rviz igvc_msgs
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set (rviz_plugins_SRCS
    src/rviz_plugins/bat_panel.cpp
    src/rviz_plugins/eStop_panel.cpp
    src/rviz_plugins/example_panel.cpp
    src/rviz_plugins/launch_panel.cpp
    src/rviz_plugins/my_thread.cpp
    src/rviz_plugins/node_panel.cpp
    src/rviz_plugins/num_button.cpp
    src/rviz_plugins/sensor_panel.cpp
    src/rviz_plugins/speedometer.cpp
    src/rviz_plugins/speed_panel.cpp
    src/rviz_plugins/time_panel.cpp
    src/rviz_plugins/waypoint_panel.cpp
)

set (rviz_plugins_HDRS
    src/rviz_plugins/bat_panel.h
    src/rviz_plugins/eStop_panel.h
    src/rviz_plugins/example_panel.h
    src/rviz_plugins/launch_panel.h
    src/rviz_plugins/my_thread.h
    src/rviz_plugins/node_panel.h
    src/rviz_plugins/num_button.h
    src/rviz_plugins/sensor_panel.h
    src/rviz_plugins/speedometer.h
    src/rviz_plugins/speed_panel.h
    src/rviz_plugins/time_panel.h
    src/rviz_plugins/waypoint_panel.h
)

qt5_wrap_cpp(rviz_plugins_MOCS ${rviz_plugins_HDRS})

add_library(${PROJECT_NAME} ${rviz_plugins_SRCS} ${rviz_plugins_MOCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Widgets)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
