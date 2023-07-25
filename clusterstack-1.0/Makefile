# Name: Makefile
# Project: USB I2C
# Author: Christian Starkjohann, modified for I2C USB by Till Harbaum
# Creation Date: 2005-03-20
# Tabsize: 4
# Copyright: (c) 2005 by OBJECTIVE DEVELOPMENT Software GmbH
# License: Proprietary, free under certain conditions. See Documentation.
# This Revision: $Id: Makefile-avrusb.tiny45,v 1.3 2007/06/07 13:53:47 harbaum Exp $

#DEFINES += -DDEBUG
#DEFINES += -DDEBUG_LEVEL=1
DEFINES += -DF_CPU=12000000
PROGRAMMER = pi_1
#COMPILE = avr-gcc -Wl,--gc-sections -Wall -O2 -I../src/usbdrv -I../src -mmcu=atmega328p $(DEFINES)
COMPILE = avr-gcc -Wl,--gc-sections,-u,vfprintf -lprintf_flt -lm -Wall -O2 -I. -I../src/usbdrv -I../src -mmcu=atmega328p $(DEFINES)

OBJECTS = usbdrv/usbdrv.o usbdrv/usbdrvasm.o usbdrv/oddebug.o main.o config.o

# symbolic targets:
all:	firmware.hex

usbdrv/%.o: ../src/usbdrv/%.c | usbdrv
	$(COMPILE) -c $< -o $@

%.o: ../src/%.c
	$(COMPILE) -c $< -o $@

config.o: config.c config.h
	$(COMPILE) -c config.c -o config.o

usbdrv/%.o: ../src/usbdrv/%.S | usbdrv
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

usbdrv: 
	mkdir $@

.c.s:
	$(COMPILE) -S $< -o $@

clean:
	rm -f firmware.hex firmware.lst firmware.obj firmware.cof firmware.list firmware.map *.bin *.o */*.o *~ */*~ firmware.s usbdrv/oddebug.s usbdrv/usbdrv.s 

# file targets:
firmware.bin:	$(OBJECTS) config.h
	$(COMPILE) -o firmware.bin $(OBJECTS)

firmware.hex:	firmware.bin
	rm -f firmware.hex firmware.eep.hex
	avr-objcopy -j .text -j .data -O ihex firmware.bin firmware.hex
	../src/checksize firmware.bin 32768 2048
# do the checksize script as our last action to allow successful compilation
# on Windows with WinAVR where the Unix commands will fail.

# Get Device signature then call make again with PID set
program: firmware.hex
	avrdude -c $(PROGRAMMER) -p atmega328pb -U lfuse:w:0xff:m -U flash:w:firmware.hex -U hfuse:w:0xd0:m -U efuse:w:0xfd:m

program-atmelice:
	avrdude -c atmelice_isp  -p atmega328pb -U lfuse:w:0xff:m -U flash:w:firmware.hex -U hfuse:w:0xd0:m -U efuse:w:0xfd:m

disasm:	firmware.bin
	avr-objdump -d firmware.bin

cpp:
	$(COMPILE) -E main.c
