add_executable(waypoint_source waypoint_source.cpp)
add_dependencies(waypoint_source igvc_msgs_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_source ${catkin_LIBRARIES})
