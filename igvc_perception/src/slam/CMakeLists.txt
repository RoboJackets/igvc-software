# qt components used (also by qglviewer): Core Gui Xml OpenGL Widgets

if (SUITESPARSE_FOUND AND G2O_FOUND)
    message(STATUS "Found suitesparse and g2o, compiling Slam")
    add_executable(Slam slam.cpp)
    target_include_directories(Slam PRIVATE ${catkin_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIRS})
    #set_target_properties(Slam PROPERTIES OUTPUT_NAME slam${EXE_POSTFIX})
    target_link_libraries(Slam ${catkin_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES} Eigen3::Eigen)
endif()