add_library (MotorEncoders MotorEncoders.cc)

add_executable (MotorEncodersTest MotorEncodersTest.cc)

target_link_libraries (MotorEncodersTest MotorEncoders ${Candi_SOURCE_DIR}/arduino/ArduinoInterface)
