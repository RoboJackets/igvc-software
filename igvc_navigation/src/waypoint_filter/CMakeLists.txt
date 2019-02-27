add_executable(waypoint_filter main.cpp)
add_dependencies(waypoint_filter igvc_msgs_gencpp)
target_link_libraries(waypoint_filter ${catkin_LIBRARIES})
