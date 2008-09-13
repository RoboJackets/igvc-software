add_library (MotorEncoders MotorEncoders.cc)

add_definitions(-I${Candi_SOURCE_DIR}/arduino)

add_executable (MotorEncodersTest MotorEncodersTest.cc)

target_link_libraries (MotorEncodersTest MotorEncoders ArduinoInterface)
