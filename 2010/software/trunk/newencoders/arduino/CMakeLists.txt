include(${IGVC_SOURCE_DIR}/arduino_diecimila_crosscompile.cmake)

include_directories(${IGVC_SOURCE_DIR}/arduino)
include_directories(${IGVC_SOURCE_DIR}/extern/libarduinocore)

link_libraries(arduinocore)

add_executable(newencoders
	main.cc
	#diffenc.cc
	diffenc_dual.cc
	serial_comm_helper.cc
)
