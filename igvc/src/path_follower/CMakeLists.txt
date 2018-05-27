add_executable(path_follower main.cpp SmoothControl.cpp)
add_dependencies(path_follower igvc_msgs_gencpp)
target_link_libraries(path_follower ${catkin_LIBRARIES})
