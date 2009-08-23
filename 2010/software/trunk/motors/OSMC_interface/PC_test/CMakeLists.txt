include_directories(${Candi_SOURCE_DIR}/arduino)

link_libraries(ArduinoInterface DataPacket)

add_executable(OSMC_test

main.cc
OSMC_driver.cc

)
