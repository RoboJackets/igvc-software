include_directories(
	${IGVC_SOURCE_DIR}/motors/old
	${IGVC_SOURCE_DIR}/motors/osmc/PC_test
	${IGVC_SOURCE_DIR}/arduino
	${IGVC_SOURCE_DIR}/encoders/pcdriver
	${IGVC_SOURCE_DIR}/encoders/pcdriver_signed
	${IGVC_SOURCE_DIR}/motors/osmc/PC_4wd
)

add_executable(joystickDrive 
	joystickmain.cc
)

target_link_libraries (joystickDrive
	joystick
)

add_library(joystick
	joystickDrive.cc
)

target_link_libraries (joystick
	SDL
	OSMC_driver
	OSMC_driver_4wd
)
