add_executable(pose_tracker main.cpp)
add_dependencies(pose_tracker igvc_msgs_gencpp)
target_link_libraries(pose_tracker ${catkin_LIBRARIES})
