
include_directories(${Candi_SOURCE_DIR}/old_motors)

link_libraries (
	SDL
	motors_old
	)

add_executable(joystickDrive joystickDrive.cc)
