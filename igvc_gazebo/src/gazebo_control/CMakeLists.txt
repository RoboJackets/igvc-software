add_executable(gazebo_control gazebo_control.cpp)
add_dependencies(gazebo_control igvc_msgs_gencpp)
target_link_libraries(gazebo_control ${catkin_LIBRARIES})
