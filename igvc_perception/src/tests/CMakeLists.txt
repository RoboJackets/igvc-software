add_executable(slam_tests main.cpp SlamTests.cpp)
add_dependencies(slam_tests ${catkin_EXPORTED_TARGETS})
target_link_libraries(slam_tests ${catkin_LIBRARIES})

install(
        TARGETS slam_tests
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)