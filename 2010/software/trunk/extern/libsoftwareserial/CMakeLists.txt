include(${IGVC_SOURCE_DIR}/arduino_diecimila_crosscompile.cmake)

include_directories(${IGVC_SOURCE_DIR}/extern/libarduinocore)

add_library (softwareserial
	SoftwareSerial.cpp
)
