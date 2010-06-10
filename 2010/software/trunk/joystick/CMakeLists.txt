
include_directories(${IGVC_SOURCE_DIR}/motors/old)
include_directories(${IGVC_SOURCE_DIR}/motors/osmc/PC_test)
include_directories(${IGVC_SOURCE_DIR}/arduino)
include_directories(${IGVC_SOURCE_DIR}/newencoders/pcdriver)
include_directories(${IGVC_SOURCE_DIR}/newencoders/pcdriver_signed)

add_executable(joystickDrive joystickDrive.cc)

target_link_libraries (joystickDrive
	SDL
	OSMC_driver
	quadCoderDriver
	)
