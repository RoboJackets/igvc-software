add_executable(path_follower path_follower.cpp smooth_control.cpp path_follower.h)
add_dependencies(path_follower igvc_msgs_gencpp)
target_link_libraries(path_follower ${catkin_LIBRARIES})

