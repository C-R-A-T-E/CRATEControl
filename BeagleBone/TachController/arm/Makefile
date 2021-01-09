PROJ_NAME = pru_echo

CHIP = am57xx
CHIP_REV = am572x_2_0

CFLAGS=-g3 -Wall
LDFLAGS=
OUT_DIR = out

TARGET = $(OUT_DIR)/$(PROJ_NAME)
SOURCES = $(wildcard *.c)
OBJS     =$(patsubst %,$(OUT_DIR)/%,$(SOURCES:.c=.o))

all: $(TARGET)

$(TARGET): $(OBJS)
	gcc $(CFLAGS) -o $(TARGET) $(OBJS)

$(OUT_DIR)/%.o: %.c
	mkdir -p $(OUT_DIR)
	gcc $(CFLAGS) -c -o $@ $<

run: $(TARGET)
	@echo "Running pru_echo"
	@$(TARGET)

clean:
	rm -rf $(OUT_DIR)
