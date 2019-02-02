add_executable(system_stats main.cpp cpu_usage.h)
add_dependencies(speed_compare igvc_msgs_gencpp)
target_link_libraries(system_stats ${catkin_LIBRARIES})
