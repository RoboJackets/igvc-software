add_executable(ground_segment GroundSegmenter.cpp main.cpp)
target_link_libraries(ground_segment ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(
    TARGETS ground_segment
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
