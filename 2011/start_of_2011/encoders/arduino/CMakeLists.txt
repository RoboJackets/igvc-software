include(${IGVC_SOURCE_DIR}/atmel_crosscompile.cmake)

include_directories(${IGVC_SOURCE_DIR}/arduino)
include_directories(${IGVC_SOURCE_DIR}/extern/libarduinocore)

add_executable(newencoders_single
	main.cc
	#diffenc.cc
	diffenc_dual.cc
	serial_comm_helper.cc
)

target_link_libraries(newencoders_single
	arduinocore
)

SET(CMAKE_C_FLAGS_front "${CMAKE_C_FLAGS} -DFRONT_ENCODER")
SET(CMAKE_CXX_FLAGS_front "${CMAKE_CXX_FLAGS} -DFRONT_ENCODER")
SET(CMAKE_C_FLAGS_back "${CMAKE_C_FLAGS} -DBACK_ENCODER")
SET(CMAKE_CXX_FLAGS_back "${CMAKE_CXX_FLAGS} -DBACK_ENCODER")

#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS_front}")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_front}")
add_executable(newencoders_front
	main.cc
	#diffenc.cc
	diffenc_dual.cc
	serial_comm_helper.cc
)

target_link_libraries(newencoders_front
	arduinocore
)

#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS_back}")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_back}")
add_executable(newencoders_back
	main.cc
	#diffenc.cc
	diffenc_dual.cc
	serial_comm_helper.cc
)

target_link_libraries(newencoders_back
	arduinocore
)
