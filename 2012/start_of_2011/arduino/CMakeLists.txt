include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

add_library (ArduinoInterface ArduinoInterface.cc)
add_library (DataPacket DataPacket.cc)

target_link_libraries(ArduinoInterface
	DataPacket
	boost_system-mt
	boost_thread-mt
)
