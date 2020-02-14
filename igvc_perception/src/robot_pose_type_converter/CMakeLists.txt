add_executable(robot_pose_type_converter robot_pose_type_converter.cpp)
add_dependencies(robot_pose_type_converter ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_pose_type_converter ${catkin_LIBRARIES})

install(
        TARGETS robot_pose_type_converter
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
