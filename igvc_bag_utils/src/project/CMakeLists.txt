add_executable(project main.cpp)
add_dependencies(project igvc_msgs_gencpp)
target_link_libraries(project ${catkin_LIBRARIES})
