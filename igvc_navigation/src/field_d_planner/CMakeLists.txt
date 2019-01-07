add_executable(field_d_planner
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/Graph.cpp
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/Node.cpp
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/PriorityQueue.cpp
        FieldDPlanner.cpp
        main.cpp)
add_dependencies(field_d_planner igvc_msgs_gencpp)
target_link_libraries(field_d_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES})
