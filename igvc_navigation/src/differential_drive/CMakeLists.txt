add_executable(differential_drive differential_drive.cpp)
add_dependencies(differential_drive ${catkin_EXPORTED_TARGETS})
target_link_libraries(differential_drive ${catkin_LIBRARIES})

install(
    TARGETS differential_drive
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
