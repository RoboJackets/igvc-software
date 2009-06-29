SET(CMAKE_SYSTEM_NAME Generic)

SET(CMAKE_C_COMPILER avr-gcc)
SET(CMAKE_CXX_COMPILER avr-g++)

SET(CSTANDARD "-std=gnu99")
SET(CDEBUG "-gstabs")
SET(CWARN "-Wall -Wstrict-prototypes")
SET(CTUNING "-funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums")
SET(COPT "-Os")
SET(CINCS "-I${Candi_SOURCE_DIR}/libarduino")
SET(CMCU "-mmcu=atmega168")
SET(CDEFS "-DF_CPU=16000000")


SET(CFLAGS "${CMCU} ${CDEBUG} ${CDEFS} ${CINCS} ${COPT} ${CWARN} ${CSTANDARD} ${CEXTRA}")
#SET(CXXFLAGS "${CMCU} ${CDEFS} ${CINCS} ${COPT} -nodefaultlibs")
SET(CXXFLAGS "${CMCU} ${CDEFS} ${CINCS} ${COPT}")

SET(CMAKE_C_FLAGS  ${CFLAGS})
SET(CMAKE_CXX_FLAGS ${CXXFLAGS})

include_directories(${Candi_SOURCE_DIR}/libarduino)
LINK_DIRECTORIES(${Candi_SOURCE_DIR}/libarduino)

link_libraries(arduinocore)
#link_libraries(libstdc++)

add_executable(cmaketest libinctest.cc)
