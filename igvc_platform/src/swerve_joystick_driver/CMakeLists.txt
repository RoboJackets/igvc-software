add_executable(swerve_joystick_driver swerve_joystick.cpp)
add_dependencies(swerve_joystick_driver ${catkin_EXPORTED_TARGETS})
target_link_libraries(swerve_joystick_driver ${catkin_LIBRARIES})

install(
    TARGETS swerve_joystick_driver
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
