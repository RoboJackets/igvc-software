SET(CMAKE_SYSTEM_NAME Generic)

SET(CMAKE_C_COMPILER avr-gcc)
SET(CMAKE_CXX_COMPILER avr-g++)

SET(CSTANDARD "-std=gnu99")
SET(CXXSTANDARD "-std=gnu++0x")
SET(CDEBUG "-gstabs")
SET(CWARN "-Wall -Wstrict-prototypes -Wextra")
SET(CXXWARN "-Wall -Wextra")
SET(CTUNING "-funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums")
SET(COPT "-Os")
SET(CMCU "-mmcu=atmega168")
SET(CDEFS "-DF_CPU=16000000")
#SET(CDEFS "-DF_CPU=8000000")


SET(CFLAGS "${CMCU} ${CDEBUG} ${CDEFS} ${CINCS} ${COPT} ${CWARN} ${CSTANDARD} ${CEXTRA}")
SET(CXXFLAGS "${CMCU} ${CDEFS} ${CINCS} ${COPT} ${CXXWARN} ${CXXSTANDARD}")

SET(CMAKE_C_FLAGS  ${CFLAGS})
SET(CMAKE_CXX_FLAGS ${CXXFLAGS})
