
include(${IGVC_SOURCE_DIR}/atmel_crosscompile.cmake)

include_directories(${IGVC_SOURCE_DIR}/extern/libarduinocore)

add_library (arduinocore19
	HardwareSerial.cpp
	pins_arduino.c
	Print.cpp
	Tone.cpp
	WInterrupts.c
	wiring_analog.c
	wiring.c
	wiring_digital.c
	wiring_pulse.c
	wiring_shift.c
	WMath.cpp
	WString.cpp
)


