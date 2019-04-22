add_executable(dc_motor_model dc_motor_model.cpp)
add_dependencies(dc_motor_model igvc_msgs_gencpp)
target_link_libraries(dc_motor_model ${catkin_LIBRARIES})
