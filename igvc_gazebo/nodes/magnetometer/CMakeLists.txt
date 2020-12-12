add_executable(mag_republisher Magnetometer.cpp)
add_dependencies(mag_republisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(mag_republisher ${catkin_LIBRARIES})

install(
        TARGETS mag_republisher
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)