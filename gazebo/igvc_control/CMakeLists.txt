cmake_minimum_required(VERSION 2.8.3)
project(igvc_control)
find_package(catkin REQUIRED)


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  igvc_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml" PATTERN "*.urdf"
)

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
   FILES_MATCHING PATTERN "*.yaml"
)

add_executable(jaymii_control src/jaymii_control.cpp)
add_dependencies(jaymii_control igvc_msgs_gencpp)
target_link_libraries(jaymii_control ${catkin_LIBRARIES})