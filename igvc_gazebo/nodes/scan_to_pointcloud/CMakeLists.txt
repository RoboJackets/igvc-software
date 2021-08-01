add_executable(scan_to_pointcloud main.cpp)
add_dependencies(scan_to_pointcloud ${catkin_EXPORTED_TARGETS})
target_link_libraries(scan_to_pointcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(
    TARGETS scan_to_pointcloud
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
