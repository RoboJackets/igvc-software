add_executable(swerve_drive swerve_drive.cpp interval.cpp utils.cpp)
add_dependencies(swerve_drive ${catkin_EXPORTED_TARGETS})
target_link_libraries(swerve_drive ${catkin_LIBRARIES})

install(
    TARGETS swerve_drive
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
