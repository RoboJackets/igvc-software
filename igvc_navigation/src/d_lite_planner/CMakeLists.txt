add_executable(d_lite_planner DLitePlanner.cpp main.cpp)
add_dependencies(d_lite_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(d_lite_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES} GraphSearch)

install(
    TARGETS d_lite_planner
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
