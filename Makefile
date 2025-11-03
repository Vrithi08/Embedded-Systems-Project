CC := gcc
CFLAGS := -Wall -Wextra -std=c11 -O2

TARGET := pill_disp
SRC := pill_disp.c

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -f $(TARGET) *.o *.elf *.hex *.bin

.PHONY: all clean
