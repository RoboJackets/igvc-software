SET(CMAKE_SYSTEM_NAME Generic)

SET(CMAKE_C_COMPILER avr-gcc)
SET(CMAKE_CXX_COMPILER avr-g++)

SET(CMAKE_C_FLAGS  "-Os -mmcu=atmega168 -DF_CPU=16000000 -std=gnu99 -gstabs -Wall -Wstrict-prototypes -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums")
SET(CMAKE_CXX_FLAGS "-Os -mmcu=atmega168 -DF_CPU=16000000 -std=gnu99 -gstabs -Wall -Wstrict-prototypes -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums")
add_library (arduino

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
