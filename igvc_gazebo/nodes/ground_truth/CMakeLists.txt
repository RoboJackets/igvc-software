add_executable(ground_truth_republisher ground_truth.cpp ground_truth.h)
add_dependencies(ground_truth_republisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(ground_truth_republisher ${catkin_LIBRARIES})

install(
    TARGETS ground_truth_republisher
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
