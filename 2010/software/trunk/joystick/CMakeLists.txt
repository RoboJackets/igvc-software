
include_directories(${Candi_SOURCE_DIR}/motors/old)

link_libraries (
	SDL
	motors_old
	)

add_executable(joystickDrive joystickDrive.cc)
