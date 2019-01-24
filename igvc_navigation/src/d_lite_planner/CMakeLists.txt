add_executable(d_lite_planner
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/Graph.cpp
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/Node.cpp
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/PriorityQueue.cpp
        DLitePlanner.cpp
        main.cpp)
add_dependencies(d_lite_planner igvc_msgs_gencpp)
target_link_libraries(d_lite_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES})
