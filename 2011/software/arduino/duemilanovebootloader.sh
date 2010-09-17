input=$1
port=$2
ext=.hex
hex=$input$ext

avr-objcopy -O ihex -R .eeprom $input $hex
avrdude -V -c arduino -p m328p -F -b 57600 -P $port -U flash:w:$hex
rm $hex
