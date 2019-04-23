add_executable(encoders encoders.cpp)
add_dependencies(encoders igvc_msgs_gencpp)
target_link_libraries(encoders ${catkin_LIBRARIES})
