project (IGVC)

cmake_minimum_required(VERSION 2.6)

set(BOARD_REV diecimila)
#set(BOARD_REV duemilanove)
#set(BOARD_REV sparkfun_pro_8mhz)

#set(ROBOT_REV candiiidbg)
#set(ROBOT_REV candiii)
set(ROBOT_REV jeannidbg)
#set(ROBOT_REV jeanni)

add_subdirectory (arduino)
add_subdirectory (lidar)
add_subdirectory (gps)
add_subdirectory (joystick)
add_subdirectory (extern)
add_subdirectory (motors)
add_subdirectory (encoders)
add_subdirectory (jeanni)
add_subdirectory (gps_jeanni)
add_subdirectory (Micro-conTest)
add_subdirectory (simple_gps_lidar_drive)
