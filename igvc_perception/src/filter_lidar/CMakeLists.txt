add_executable(filter_lidar main.cpp)
add_dependencies(filter_lidar ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_lidar ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(
    TARGETS filter_lidar
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
