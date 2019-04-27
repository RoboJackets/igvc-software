add_executable(quaternion_to_rpy main.cpp)
add_dependencies(quaternion_to_rpy igvc_msgs_gencpp)
target_link_libraries(quaternion_to_rpy ${catkin_LIBRARIES})
