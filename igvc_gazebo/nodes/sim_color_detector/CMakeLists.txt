add_executable(sim_color_detector sim_color_detector.cpp)
add_dependencies(sim_color_detector ${catkin_EXPORTED_TARGETS})
target_link_libraries(sim_color_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(
    TARGETS sim_color_detector
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
