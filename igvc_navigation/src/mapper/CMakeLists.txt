add_executable(mapper main.cpp)
add_dependencies(mapper igvc_msgs_gencpp)
target_link_libraries(mapper ${catkin_LIBRARIES} ${PCL_LIBRARIES} igvc_state)
