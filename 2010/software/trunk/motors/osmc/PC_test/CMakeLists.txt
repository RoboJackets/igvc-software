include(${IGVC_SOURCE_DIR}/standardPCOpts.cmake)

include_directories(${IGVC_SOURCE_DIR}/arduino)

add_library (OSMC_driver OSMC_driver.cc)

link_libraries(ArduinoInterface DataPacket boost_system-mt OSMC_driver)

add_executable(OSMC_test

main.cc


)
