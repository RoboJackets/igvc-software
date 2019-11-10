cmake_minimum_required(VERSION 2.8.3)
project(igvc_rviz_plugins)

include(../igvc.cmake)

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
#  LIBRARIES ${PROJECT_NAME}
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
        src/igvc_rviz_plugins/bat_panel.cpp
        src/igvc_rviz_plugins/eStop_panel.cpp
        src/igvc_rviz_plugins/example_panel.cpp
        src/igvc_rviz_plugins/launch_panel.cpp
        src/igvc_rviz_plugins/my_thread.cpp
        src/igvc_rviz_plugins/node_panel.cpp
        src/igvc_rviz_plugins/num_button.cpp
        src/igvc_rviz_plugins/sensor_panel.cpp
        src/igvc_rviz_plugins/speedometer.cpp
        src/igvc_rviz_plugins/speed_panel.cpp
        src/igvc_rviz_plugins/time_panel.cpp
        src/igvc_rviz_plugins/waypoint_panel.cpp
)

set (rviz_plugins_HDRS
        include/igvc_rviz_plugins/bat_panel.h
        include/igvc_rviz_plugins/eStop_panel.h
        include/igvc_rviz_plugins/example_panel.h
        include/igvc_rviz_plugins/launch_panel.h
        include/igvc_rviz_plugins/my_thread.h
        include/igvc_rviz_plugins/node_panel.h
        include/igvc_rviz_plugins/num_button.h
        include/igvc_rviz_plugins/sensor_panel.h
        include/igvc_rviz_plugins/speedometer.h
        include/igvc_rviz_plugins/speed_panel.h
        include/igvc_rviz_plugins/time_panel.h
        include/igvc_rviz_plugins/waypoint_panel.h
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
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
