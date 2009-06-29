add_subdirectory(ArduinoMotorEncoders)

add_library (MotorEncoders MotorEncoders.cc)

include_directories(${Candi_SOURCE_DIR}/arduino)

add_executable (MotorEncodersTest MotorEncodersTest.cc)

target_link_libraries (MotorEncodersTest MotorEncoders ArduinoInterface DataPacket)
