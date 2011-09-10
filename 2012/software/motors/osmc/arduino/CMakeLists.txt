include(${IGVC_SOURCE_DIR}/atmel_crosscompile.cmake)


include_directories(${IGVC_SOURCE_DIR}/arduino)
include_directories(${IGVC_SOURCE_DIR}/extern/libarduinocore)
include_directories(${IGVC_SOURCE_DIR}/extern/libsoftwareserial)

link_libraries(arduinocore)
#link_libraries(arduinocore softwareserial)

add_executable(OSMC_interface

main.cc
serial_comm_helper.cc
packet_handle.cc
SPI.cc
wheel_encoders.cc
current_sensors.cc
motorPWM.cc
#profile.cc
analogjoystick.cc
)


add_executable(MagTest
	MagnotometerTest.cpp
)


