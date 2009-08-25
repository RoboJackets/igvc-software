include_directories(${IGVC_SOURCE_DIR}/arduino)

link_libraries(ArduinoInterface DataPacket boost_system-mt)

add_executable(OSMC_test

main.cc
OSMC_driver.cc

)
