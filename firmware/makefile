# Name: makefile
# Project: rocket_ejection
# Author: Jonny Dyer
# Credits: Pascal Stang for avrlib

MCU = attiny84
F_CPU = 12000000   # Hz
BOARD_REV ?= REV_B
BOOTLOADER_ADDRESS=1940
TINYVECTOR_OSCCAL_OFFSET=6

TRG = main

SRC = $(TRG).c

# AVRLIB defines
AVRLIB_SRC = swTimer.c timerx8.c
AVRLIB = /Users/$(USER)/Documents/avrlib/

# V-USB defines
VUSBLIB = usbdrv/
VUSBOBJ = # usbdrv.o usbdrvasm.o oddebug.o

#put additional assembler source file here
ASRC = osccalASM.o

#additional libraries and object files to link
LIB	=

#additional includes to compile
INC	=  

#assembler flags
ASFLAGS = -I. -I$(AVRLIB) -I$(VUSBLIB) \
		  -DF_CPU=$(F_CPU) -D$(BOARD_REV)=1 \

#compiler flags
CPFLAGS	= -g -Os -Wall -Wstrict-prototypes \
		  -I. -I$(AVRLIB) -I$(VUSBLIB) \
		  -DF_CPU=$(F_CPU) -D$(BOARD_REV)=1 \
		  -DBOOTLOADER_ADDRESS=$(BOOTLOADER_ADDRESS) \
		  -DTINYVECTOR_OSCCAL_OFFSET=$(TINYVECTOR_OSCCAL_OFFSET) \
		  -Wa,-ahlms=$(<:.c=.lst)

LDFLAGS = -Wl,-Map=$(TRG).map,--cref# -lm

CC	= avr-gcc
AS	= avr-gcc -x assembler-with-cpp	
AVRDUDE = avrdude -c avrispv2 -P /dev/tty.usbserial

SRC += $(addprefix $(AVRLIB), $(AVRLIB_SRC)) 
OBJ = $(ASRC:.s=.o) $(SRC:.c=.o) $(addprefix $(VUSBLIB),$(VUSBOBJ))
CPFLAGS += -mmcu=$(MCU)
ASFLAGS += -mmcu=$(MCU)
LDFLAGS += -mmcu=$(MCU)

###### RULES ########

all: $(TRG).elf $(TRG).hex $(TRG).ok

%.o : %.c 
	@echo "Building $<"
	$(CC) -c $(CPFLAGS) $< -o $@

%.S : %.c
	@echo "Building $<"
	$(CC) -S $(CPFLAGS) $< -o $@

%.o : %.S
	@echo "Assembling $<"
	$(AS) -c $(ASFLAGS) $< -o $@

%.elf : $(OBJ)
	@echo "Linking $@"
	$(CC) $(OBJ) $(LIB) $(LDFLAGS) -o $@

%.hex: %.elf
	avr-objcopy -j .text -j .data -O ihex -R .eeprom $< $@

%ok:
	avr-size $(TRG).elf
	@echo "Errors: none" 

help:
	@echo "make ........... to build $(TRG).hex"
	@echo "make fuse ...... to flash the fuses"
	@echo "make flash ..... to flash the firmware (use this on metaboard)"
	@echo "make clean ..... to delete objects and hex file"

fuse:
	@[ "$(FUSE_H)" != "" -a "$(FUSE_L)" != "" ] || \
		{ echo "*** Edit Makefile and choose values for FUSE_L and FUSE_H!"; exit 1; }
	$(AVRDUDE) -U hfuse:w:$(FUSE_H):m -U lfuse:w:$(FUSE_L):m

flash: $(TRG).hex
	micronucleus --run $(TRG).hex
	#$(AVRDUDE) -pt84 -U flash:w:$(TRG).hex


clean::
	$(RM) $(OBJ)
	$(RM) $(SRC:.c=.s)
	$(RM) $(SRC:.c=.lst)
	$(RM) $(TRG).map
	$(RM) $(TRG).elf
	$(RM) $(TRG).cof
	$(RM) $(TRG).obj
	$(RM) $(TRG).a90
	$(RM) $(TRG).sym
	$(RM) $(TRG).eep
	$(RM) $(TRG).hex
	$(RM) $(TRG).bin
	@echo "Errors: none"
	
size:
	avr-size $(TRG).elf
		  
