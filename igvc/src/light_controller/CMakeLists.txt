add_executable(light_controller main.cpp)
target_link_libraries(light_controller ${catkin_LIBRARIES} SerialPort)
