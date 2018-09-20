add_executable(speed_compare main.cpp)
add_dependencies(speed_compare igvc_msgs_gencpp)
target_link_libraries(speed_compare ${catkin_LIBRARIES})
