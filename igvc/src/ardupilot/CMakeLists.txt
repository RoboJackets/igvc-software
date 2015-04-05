add_executable(ardupilot ardupilot.cpp)
target_link_libraries(ardupilot ${catkin_LIBRARIES} SerialPort)
