cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()

set(CMAKE_CXX_FLAGS "-g -std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(OGRE OGRE)
include_directories( ${OGRE_INCLUDE_DIRS} )
link_directories( ${OGRE_LIBRARY_DIRS} )

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
  src/bare_panel.h
  src/speedometer.h
)

set(SOURCE_FILES
  src/bare_panel.cpp
  src/speedometer.cpp
  ${MOC_FILES}
)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${OGRE_LIBRARIES}${QWT_LIBRARIES})
