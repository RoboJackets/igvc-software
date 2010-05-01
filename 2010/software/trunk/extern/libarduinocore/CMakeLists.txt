IF(${BOARD_REV} MATCHES "diecimila")
	include(${IGVC_SOURCE_DIR}/arduino_diecimila_crosscompile.cmake)
ELSEIF(${BOARD_REV} MATCHES "duemilanove")
	include(${IGVC_SOURCE_DIR}/arduino_duemilanove_crosscompile.cmake)
ENDIF(${BOARD_REV} MATCHES "diecimila")

include_directories(${IGVC_SOURCE_DIR}/extern/libarduinocore)

add_library (arduinocore
	HardwareSerial.cpp
	Print.cpp
	wiring.c
	wiring_shift.c
	WInterrupts.c
	wiring_digital.c
	WMath.cpp
	pins_arduino.c
	wiring_analog.c
	wiring_pulse.c
	Tone.cpp
)
