find_package(LibFreespace)

if(EXISTS "/usr/local/lib/libfreespace.so")
    add_executable(imu main.cpp)
    add_dependencies(imu igvc_msgs_gencpp)
    target_link_libraries(imu ${catkin_LIBRARIES} /usr/local/lib/libfreespace.so)
endif()
