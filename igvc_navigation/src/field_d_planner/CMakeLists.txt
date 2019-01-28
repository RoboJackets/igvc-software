add_executable(field_d_planner
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/Graph.cpp
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/Node.cpp
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/PriorityQueue.cpp
        FieldDPlanner.cpp
        main.cpp)
add_dependencies(field_d_planner igvc_msgs_gencpp)
target_link_libraries(field_d_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(grid_test
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/Graph.cpp
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/Node.cpp
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/PriorityQueue.cpp
        FieldDPlanner.cpp
        unit_tests/main.cpp)
add_dependencies(grid_test igvc_msgs_gencpp)
target_link_libraries(grid_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})
