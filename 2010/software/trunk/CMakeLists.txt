project (IGVC)

cmake_minimum_required(VERSION 2.6)

set(BOARD_REV diecimila)
#set(BOARD_REV duemilanove)
#set(BOARD_REV sparkfun_pro_8mhz)

set(ROBOT_REV candiiidbg)
#set(ROBOT_REV candiii)
#set(ROBOT_REV jeanidbg)
#set(ROBOT_REV jeani)

add_subdirectory (arduino)
add_subdirectory (lidar)
#add_subdirectory (gps)
add_subdirectory (joystick)
add_subdirectory (extern)
add_subdirectory (motors)
add_subdirectory (encoders)
add_subdirectory (newencoders)
add_subdirectory (candiii)
