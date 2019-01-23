find_package(OpenMP)
if (OpenMP_FOUND)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_executable(particle_filter
  particle_filter_node.cpp
  octomapper.cpp octomapper.h
  particle_filter.h particle_filter.cpp
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/hsluv.h
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/hsluv.c)
add_dependencies(particle_filter igvc_msgs_gencpp)
target_link_libraries(particle_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})
