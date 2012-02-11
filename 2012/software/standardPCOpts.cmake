
IF((${ROBOT_REV} MATCHES "candiiidbg") OR (${ROBOT_REV} MATCHES "jeannidbg"))
	SET(CMAKE_C_FLAGS  "-march=native -pipe -Wall -Wstrict-prototypes -Wextra -ggdb3 -O0 -pthread -DOSMC_4WD -fopenmp")
	SET(CMAKE_CXX_FLAGS "-march=native -pipe -Wall -Wextra -ggdb3 -O0 -pthread -DOSMC_4WD  -fopenmp -rdynamic -g")
ELSEIF( (${ROBOT_REV} MATCHES "candiii") OR (${ROBOT_REV} MATCHES "jeanni"))
	SET(CMAKE_C_FLAGS  "-march=native -pipe -Wall -Wstrict-prototypes -Wextra -Wuninitialized -Werror=uninitialized -ggdb3 -O2 -pthread -DOSMC_4WD  -fopenmp")
#	SET(CMAKE_CXX_FLAGS "-march=native -pipe -std=gnu++0x -Wall -Wextra -Wuninitialized -Werror=uninitialized -g3 -O2 -pthread")
	SET(CMAKE_CXX_FLAGS "-march=native -pipe -Wall -Wextra -Wuninitialized -Werror=uninitialized -ggdb3 -O2 -pthread -DOSMC_4WD -fopenmp")
ENDIF((${ROBOT_REV} MATCHES "candiiidbg") OR (${ROBOT_REV} MATCHES "jeannidbg"))
