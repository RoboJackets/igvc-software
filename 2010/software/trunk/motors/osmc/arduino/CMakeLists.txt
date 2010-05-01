
IF(${BOARD_REV} MATCHES "diecimila")
	include(${IGVC_SOURCE_DIR}/arduino_diecimila_crosscompile.cmake)
ELSEIF(${BOARD_REV} MATCHES "duemilanove")
	include(${IGVC_SOURCE_DIR}/arduino_duemilanove_crosscompile.cmake)
ENDIF(${BOARD_REV} MATCHES "diecimila")


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

