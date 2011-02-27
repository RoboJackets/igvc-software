include(${IGVC_SOURCE_DIR}/atmel_crosscompile.cmake)

include_directories(${IGVC_SOURCE_DIR}/arduino)
include_directories(${IGVC_SOURCE_DIR}/extern/libarduinocore)

add_executable(Micro-controllerTest
	Micro-controllerTest.cc
)

target_link_libraries(Micro-controllerTest
	arduinocore
)
