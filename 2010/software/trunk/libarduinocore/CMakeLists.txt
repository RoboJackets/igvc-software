include(${Candi_SOURCE_DIR}/arduino_diecimila_crosscompile.cmake)

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
)
