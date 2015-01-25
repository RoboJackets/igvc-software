add_executable(motor_controller motor_controller.cpp)
target_link_libraries(motor_controller ${catkin_LIBRARIES} SerialPort)
