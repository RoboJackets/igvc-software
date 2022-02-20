add_executable(swerve_odometer swerve_odometer.cpp)
add_dependencies(swerve_odometer ${catkin_EXPORTED_TARGETS})
target_link_libraries(swerve_odometer ${catkin_LIBRARIES})

install(
    TARGETS swerve_odometer
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)