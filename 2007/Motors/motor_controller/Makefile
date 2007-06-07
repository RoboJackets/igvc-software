TARGET=controller
OBJS=controller.o rjte.o comm.o joystick.o
DEVICE=atmega8

CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-O3 -Wall -mmcu=$(DEVICE)
LDFLAGS=-mmcu=$(DEVICE)

all:		$(TARGET).hex $(TARGET).bin

clean:
		rm -f $(TARGET).hex $(TARGET).bin $(TARGET).elf $(OBJS)

prog:		$(TARGET).bin
		rjte_load $(TARGET).bin "/dev/ttyUSB0"

$(TARGET).elf:	$(OBJS)
		$(CC) $(CFLAGS) -I /usr/include/ $(LDFLAGS) $(OBJS) -o $(TARGET).elf

$(TARGET).hex:	$(TARGET).elf
		$(OBJCOPY) -O ihex $< $@

$(TARGET).bin:	$(TARGET).elf
		$(OBJCOPY) -O binary $< $@
