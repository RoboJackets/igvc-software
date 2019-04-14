add_library(TestLib ${LIB_TYPE} helloWorld.cu)

add_executable(trajectory_controller
               main.cpp
               trajectory_controller.cpp
               ros_trajectory_controller.cpp
               differential_drive_model.cpp
               differential_drive_distance_field_cost.cpp)
add_dependencies(trajectory_controller igvc_msgs_gencpp TestLib)
target_link_libraries(trajectory_controller ${catkin_LIBRARIES} TestLib FastSweep SignedDistanceField)
