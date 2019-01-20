add_executable(particle_filter particle_filter_node.cpp octomapper.cpp octomapper.h
  particle_filter.h particle_filter.cpp)
add_dependencies(particle_filter igvc_msgs_gencpp)
target_link_libraries(particle_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})
