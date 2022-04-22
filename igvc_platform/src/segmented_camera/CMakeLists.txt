find_package(PkgConfig REQUIRED)


add_executable(segmented_camera_info_publisher segmented_camera_info_publisher.cpp)
add_dependencies(segmented_camera_info_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(segmented_camera_info_publisher ${catkin_LIBRARIES})


install(
    TARGETS segmented_camera_info_publisher
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
