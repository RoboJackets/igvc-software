partnum=$1
input=$2
ext=.hex
hex=$input$ext

avr-objcopy -O ihex -R .eeprom $input $hex
avrdude -V -c dragon_jtag -p $partnum -b 19200 -P usb -U flash:w:$hex -U flash:v:$hex
rm $hex
