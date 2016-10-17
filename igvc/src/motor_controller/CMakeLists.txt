add_executable(motor_controller main.cpp)
add_dependencies(motor_controller igvc_msgs_gencpp)
target_link_libraries(motor_controller ${catkin_LIBRARIES} SerialPort)
