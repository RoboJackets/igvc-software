input=$1
port=$2
ext=.hex
hex=$input$ext

avr-objcopy -O ihex -R .eeprom $input $hex
avrdude -V -c stk500v1 -p m328 -F -b 19200 -P $port -U flash:w:$hex
rm $hex
