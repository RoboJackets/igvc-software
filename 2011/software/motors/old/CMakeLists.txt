include_directories(
${IGVC_SOURCE_DIR}/motors/osmc/PC_test
${IGVC_SOURCE_DIR}/arduino
${IGVC_SOURCE_DIR}/newencoders/pcdriver
)
add_Library(motors_old motors_old.cpp)
