add_executable(system_stats main.cpp cpu_usage.hpp)
add_dependencies(system_stats igvc_msgs_gencpp)
target_link_libraries(system_stats ${catkin_LIBRARIES})
