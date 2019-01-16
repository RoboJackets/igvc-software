add_executable(particle_filter particle_filter.cpp basic_particle_filter.cpp basic_particle_filter.h particle_filter_base.h particle_filter_base.cpp octomapper.cpp octomapper.h)
add_dependencies(particle_filter igvc_msgs_gencpp)
target_link_libraries(particle_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
