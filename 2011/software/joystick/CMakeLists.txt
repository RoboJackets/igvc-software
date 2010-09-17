
include_directories(
	${IGVC_SOURCE_DIR}/motors/old
	${IGVC_SOURCE_DIR}/motors/osmc/PC_test
	${IGVC_SOURCE_DIR}/arduino
	${IGVC_SOURCE_DIR}/encoders/pcdriver
	${IGVC_SOURCE_DIR}/encoders/pcdriver_signed
)

add_executable(joystickDrive joystickDrive.cc)

target_link_libraries (joystickDrive
	SDL
	OSMC_driver
	quadCoderDriver
	)
