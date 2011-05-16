

IF(${BOARD_REV} MATCHES "diecimila")
	SET(CMAKE_SYSTEM_NAME Generic)

	SET(CMAKE_C_COMPILER avr-gcc)
	SET(CMAKE_CXX_COMPILER avr-g++)

	SET(CMAKE_C_FLAGS "-std=gnu99 -Wall -Wstrict-prototypes -Wextra -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Os -mmcu=atmega168 -DF_CPU=8000000")
	SET(CMAKE_CXX_FLAGS "-std=gnu++0x -Wall -Wextra -fpack-struct -Os -mmcu=atmega168 -DF_CPU=16000000")
ELSEIF(${BOARD_REV} MATCHES "duemilanove")
	SET(CMAKE_SYSTEM_NAME Generic)

	SET(CMAKE_C_COMPILER avr-gcc)
	SET(CMAKE_CXX_COMPILER avr-g++)

	SET(CMAKE_C_FLAGS "-std=gnu99 -Wall -Wstrict-prototypes -Wextra -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Os -mmcu=atmega168 -DF_CPU=8000000")
	SET(CMAKE_CXX_FLAGS "-std=gnu++0x -Wall -Wextra -fpack-struct -Os -mmcu=atmega328p -DF_CPU=16000000")
ELSEIF(${BOARD_REV} MATCHES "sparkfun_pro_8mhz")
	SET(CMAKE_SYSTEM_NAME Generic)

	SET(CMAKE_C_COMPILER avr-gcc)
	SET(CMAKE_CXX_COMPILER avr-g++)

	SET(CMAKE_C_FLAGS "-std=gnu99 -Wall -Wstrict-prototypes -Wextra -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Os -mmcu=atmega168 -DF_CPU=8000000")
	SET(CMAKE_CXX_FLAGS "-std=gnu++0x -Wall -Wextra -Os -fpack-struct -mmcu=atmega168 -DF_CPU=8000000")
ENDIF(${BOARD_REV} MATCHES "diecimila")
