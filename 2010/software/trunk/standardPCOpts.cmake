SET(CDEBUG "-g3")
SET(CWARN "-Wall -Wstrict-prototypes -Wextra -Wuninitialized -Werror=uninitialized")
SET(CXXWARN "-Wall -Wextra -Wuninitialized -Werror=uninitialized")
SET(CXXSTANDARD "-std=gnu++0x")
#SET(CTUNING "-funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums")
SET(COPT "-O2")

SET(CFLAGS "${CDEBUG} ${COPT} ${CWARN}")
SET(CXXFLAGS "${CDEBUG} ${COPT} ${CXXWARN} ${CXXSTANDARD}")

SET(CMAKE_C_FLAGS  ${CFLAGS})
SET(CMAKE_CXX_FLAGS ${CXXFLAGS})
