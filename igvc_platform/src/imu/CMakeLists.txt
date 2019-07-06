add_executable(imu YostLabDriver.cpp yostlab_driver_node.cpp)
add_dependencies(imu ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu ${catkin_LIBRARIES})

install(
    TARGETS imu
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
