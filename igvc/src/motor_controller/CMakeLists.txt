add_executable(motor_controller main.cpp)
target_link_libraries(motor_controller ${catkin_LIBRARIES} SerialPort)
