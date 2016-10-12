add_executable(ardupilot ardupilot.cpp)
add_dependencies(ardupilot igvc_msgs_gencpp)
target_link_libraries(ardupilot ${catkin_LIBRARIES} SerialPort)
