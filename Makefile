# autosar-ecu-freertos - CAN signal packing + UDS (ISO 14229) demo
CC      = gcc
CFLAGS  = -std=c11 -Wall -Wextra -O2 -Isrc
TARGET  = autosar_demo
SRCS    = src/main.c src/can_uds.c

all: $(TARGET)

$(TARGET): $(SRCS) src/can_uds.h
	$(CC) $(CFLAGS) -o $@ $(SRCS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: all run clean
