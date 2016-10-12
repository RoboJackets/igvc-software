add_executable(odometer main.cpp Odometer.cpp)
add_dependencies(odometer igvc_msgs_gencpp)
target_link_libraries(odometer ${catkin_LIBRARIES})
