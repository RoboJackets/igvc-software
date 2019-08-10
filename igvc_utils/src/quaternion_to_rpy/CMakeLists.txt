add_executable(quaternion_to_rpy main.cpp)
add_dependencies(quaternion_to_rpy ${catkin_EXPORTED_TARGETS})
target_link_libraries(quaternion_to_rpy ${catkin_LIBRARIES})

install(
    TARGETS quaternion_to_rpy
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
