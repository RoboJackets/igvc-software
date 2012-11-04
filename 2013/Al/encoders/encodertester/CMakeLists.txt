include(${IGVC_SOURCE_DIR}/arduino_diecimila_crosscompile.cmake)

include_directories(${IGVC_SOURCE_DIR}/arduino)
include_directories(${IGVC_SOURCE_DIR}/extern/libarduinocore)

link_libraries(arduinocore)

add_executable(encodertester
	encodertester.cc
)

#add_executable(encoderTesting2
#	encoderTesting2.cc
#)
