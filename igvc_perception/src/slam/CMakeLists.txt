# qt components used (also by qglviewer): Core Gui Xml OpenGL Widgets

find_package(SUITESPARSE)
find_package(G2O)

if (SUITESPARSE_FOUND AND G2O_FOUND)
    message(STATUS "Found suitesparse and g2o, compiling Slam")
    find_package(Eigen3 3.3 REQUIRED)
    add_executable(Slam
            slam.cpp
            g2o/vertex/vertex_circle.cpp
            g2o/vertex/vertex_robot_state.cpp
            g2o/edge/edge_point_on_circle.cpp
            g2o/edge/edge_imu.cpp
            g2o/edge/edge_landmark.cpp)
    target_include_directories(Slam PRIVATE ${catkin_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIRS})
    target_link_libraries(Slam ${catkin_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES} Eigen3::Eigen)
    add_dependencies(Slam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
else()
    message(STATUS "Couldn't find suitesparse and g2o. Not compiling Slam.")
endif()
