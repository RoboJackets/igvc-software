include(${IGVC_SOURCE_DIR}/atmel_crosscompile.cmake)

include_directories(${IGVC_SOURCE_DIR}/extern/libarduinocore)

add_library (softwareserial
	SoftwareSerial.cpp
)
