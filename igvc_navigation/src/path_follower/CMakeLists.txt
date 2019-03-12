add_executable(path_follower Path_follower.cpp Smooth_control.cpp Path_follower.h)
add_dependencies(path_follower igvc_msgs_gencpp)
target_link_libraries(path_follower ${catkin_LIBRARIES})
