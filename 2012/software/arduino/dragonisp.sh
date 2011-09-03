input=$1
ext=.hex
hex=$input$ext

avr-objcopy -O ihex -R .eeprom $input $hex
sudo avrdude -V -c dragon_isp -p m168 -b 19200 -P usb -U flash:w:$hex
rm $hex
