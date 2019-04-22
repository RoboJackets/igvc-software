add_executable(control gazebo_control.cpp)
add_dependencies(control igvc_msgs_gencpp)
target_link_libraries(control ${catkin_LIBRARIES})
