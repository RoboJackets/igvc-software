add_executable(field_d_planner FieldDPlanner.cpp main.cpp)
add_dependencies(field_d_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(field_d_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES} GraphSearch)

install(
    TARGETS field_d_planner
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
