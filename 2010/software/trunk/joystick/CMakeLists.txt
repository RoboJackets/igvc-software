
include_directories(${IGVC_SOURCE_DIR}/motors/old)
include_directories(${IGVC_SOURCE_DIR}/motors/osmc/PC_test)
include_directories(${IGVC_SOURCE_DIR}/arduino)


link_libraries (
	SDL
	boost_system-mt
	OSMC_driver
	ArduinoInterface
	DataPacket
	)

add_executable(joystickDrive joystickDrive.cc)
