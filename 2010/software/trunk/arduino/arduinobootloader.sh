input=$1
ext=.hex
hex=$input$ext

avr-objcopy -O ihex -R .eeprom $input $hex
avrdude -V -c stk500v1 -p m168 -F -b 19200 -P /dev/ttyUSB0 -U flash:w:$hex
rm $hex
