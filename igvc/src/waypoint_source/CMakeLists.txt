add_executable(waypoint_source main.cpp)
add_dependencies(waypoint_source igvc_msgs_gencpp)
target_link_libraries(waypoint_source ${catkin_LIBRARIES})
