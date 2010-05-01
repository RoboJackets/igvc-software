
#SET(CFLAGS "-Wall -Wstrict-prototypes -Wextra -Wuninitialized -Werror=uninitialized -g3 -O2 -pthread")
#SET(CXXFLAGS "-std=gnu++0x -Wall -Wextra -Wuninitialized -Werror=uninitialized -g3 -O2 -pthread")

SET(CFLAGS "-Wall -Wstrict-prototypes -Wextra -g3 -O0 -pthread")
SET(CXXFLAGS "-std=gnu++0x -Wall -Wextra -g3 -O0 -pthread")

SET(CMAKE_C_FLAGS  ${CFLAGS})
SET(CMAKE_CXX_FLAGS ${CXXFLAGS})
