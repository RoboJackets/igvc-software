include(${IGVC_SOURCE_DIR}/arduino_diecimila_crosscompile.cmake)

include_directories(${IGVC_SOURCE_DIR}/arduino)
include_directories(${IGVC_SOURCE_DIR}/extern/libarduinocore)

link_libraries(arduinocore)

add_executable(OSMC_interface

main.cc
serial_comm_helper.cc
packet_handle.cc
SPI.cc
wheel_encoders.cc
current_sensors.cc
)
