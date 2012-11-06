include(${IGVC_SOURCE_DIR}/atmel_crosscompile.cmake)

include_directories(${IGVC_SOURCE_DIR}/arduino)
include_directories(${IGVC_SOURCE_DIR}/extern/libarduinocore)

add_executable(newencoders_signed
	main.cc
	diffenc.cc
	serial_comm_helper.cc
)

target_link_libraries(newencoders_signed
	arduinocore
)


