find_package(LibFreespace)

if(EXISTS "/usr/local/lib/libfreespace.so")
    add_executable(imu_legacy main.cpp)
    add_dependencies(imu_legacy igvc_msgs_gencpp)
    target_link_libraries(imu_legacy ${catkin_LIBRARIES} /usr/local/lib/libfreespace.so)
endif()
