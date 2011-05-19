
IF((${ROBOT_REV} MATCHES "candiiidbg") OR (${ROBOT_REV} MATCHES "jeannidbg"))
	SET(CMAKE_C_FLAGS  "-march=native -pipe -Wall -Wstrict-prototypes -Wextra -ggdb3 -O0 -pthread")
	SET(CMAKE_CXX_FLAGS "-march=native -pipe -Wall -Wextra -ggdb3 -O0 -pthread")
ELSEIF( (${ROBOT_REV} MATCHES "candiii") OR (${ROBOT_REV} MATCHES "jeanni"))
	SET(CMAKE_C_FLAGS  "-march=native -pipe -Wall -Wstrict-prototypes -Wextra -Wuninitialized -Werror=uninitialized -ggdb3 -O2 -pthread")
#	SET(CMAKE_CXX_FLAGS "-march=native -pipe -std=gnu++0x -Wall -Wextra -Wuninitialized -Werror=uninitialized -g3 -O2 -pthread")
	SET(CMAKE_CXX_FLAGS "-march=native -pipe -Wall -Wextra -Wuninitialized -Werror=uninitialized -ggdb3 -O2 -pthread")
ENDIF((${ROBOT_REV} MATCHES "candiiidbg") OR (${ROBOT_REV} MATCHES "jeannidbg"))
