find_package(pigpio REQUIRED)

add_library(pi_motor pi_motor.cpp pi_motor.h)
target_link_libraries(pi_motor ${pigpio_LIBRARY})
target_include_directories(pi_motor PRIVATE ${pigpio_INCLUDE_DIRS})

add_executable(pi_motor_controller pi_motor_controller.cpp pi_motor_controller.h)
target_link_libraries(pi_motor_controller ${catkin_LIBRARIES} pi_motor)
