cmake_minimum_required(VERSION 3.9.0)
project(igvc_rviz_plugins)

include(../igvc.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  igvc_msgs
)

set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
    ## pull in all required include dirs, define QT_LIBRARIES, etc.
    include(${QT_USE_FILE})
else()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
    set(QT_LIBRARIES Qt5::Widgets)
endif()
add_definitions(-DQT_NO_KEYWORDS)

find_package(Boost REQUIRED COMPONENTS container)


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

link_directories(${catkin_LIBRARY_DIRS})

#add_subdirectory(src/imu_viz)

set(SRC_FILES
    src/imu_viz/imu_display.cpp
    src/imu_viz/imu_visual.cpp
    include/imu_viz/imu_display.h
    include/imu_viz/imu_visual.h
    )

add_library(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#
#add_library(${PROJECT_NAME} ${rviz_plugins_SRCS} ${rviz_plugins_MOCS})
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Widgets)
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
