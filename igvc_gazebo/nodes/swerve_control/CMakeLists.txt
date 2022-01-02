add_executable(swerve_control swerve_control.cpp)
add_dependencies(swerve_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(swerve_control ${catkin_LIBRARIES})

install(
    TARGETS swerve_control
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
