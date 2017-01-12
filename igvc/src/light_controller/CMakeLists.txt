add_executable(light_controller main.cpp)
add_dependencies(light_controller igvc_msgs_gencpp)
target_link_libraries(light_controller ${catkin_LIBRARIES} SerialPort)
