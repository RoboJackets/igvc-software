add_executable(back_circle main.cpp)
add_dependencies(back_circle ${catkin_EXPORTED_TARGETS})
target_link_libraries(back_circle ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})

install(
    TARGETS back_circle
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
