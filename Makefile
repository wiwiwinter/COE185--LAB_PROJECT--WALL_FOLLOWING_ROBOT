DEVICE     = atmega328p
CLOCK      = 16000000
PROGRAMMER = -c arduino -b 115200 -P COM3
FUSES      = -U hfuse:w:0xde:m -U lfuse:w:0xff:m -U efuse:w:0x05:m

SRC_DIR    = src
INC_DIR    = include
OBJECTS    = $(SRC_DIR)/main.o $(SRC_DIR)/time.o $(SRC_DIR)/ultrasonic.o $(SRC_DIR)/motors.o $(SRC_DIR)/servo.o $(SRC_DIR)/uart.o

AVRDUDE    = avrdude $(PROGRAMMER) -p $(DEVICE)
COMPILE    = avr-gcc -Wall -Os -std=gnu99 -I$(INC_DIR) -DDEBUG_SAMPLES -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)
LDFLAGS   += -Wl,-u,vfprintf -lprintf_flt -lm

all: main.hex

$(SRC_DIR)/%.o: $(SRC_DIR)/%.c
	$(COMPILE) -c $< -o $@

main.elf: $(OBJECTS)
	$(COMPILE) -o main.elf $(OBJECTS) $(LDFLAGS)

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avr-size --format=avr --mcu=$(DEVICE) main.elf

flash: all
	$(AVRDUDE) -U flash:w:main.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

install: flash fuse

load: all
	bootloadHID main.hex

clean:
	rm -f main.hex main.elf $(OBJECTS)

disasm: main.elf
	avr-objdump -d main.elf

cpp:
	$(COMPILE) -E $(SRC_DIR)/main.c
