add_executable(
    mapper ros_mapper.cpp octomapper.cpp octomapper.h ros_mapper.h map_utils.cpp map_utils.h mapper.cpp mapper.h)
add_dependencies(mapper ${catkin_EXPORTED_TARGETS})
target_link_libraries(mapper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

find_package(OpenMP)
if (OpenMP_FOUND)
    target_link_libraries(mapper ${OpenMP_CXX_LIBRARIES})
endif ()

install(
    TARGETS mapper
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
