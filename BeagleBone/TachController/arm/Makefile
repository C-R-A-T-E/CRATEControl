PROJ_NAME = tach_control

CHIP = am57xx
CHIP_REV = am572x_2_0

CFLAGS=-g3 -Wall
LDFLAGS=
OUT_DIR = out
BIN_DIR = ../bin

TARGET = $(OUT_DIR)/$(PROJ_NAME)
SOURCES = $(wildcard *.c)
OBJS     =$(patsubst %,$(OUT_DIR)/%,$(SOURCES:.c=.o))

all: $(TARGET)

$(TARGET): $(OBJS)
	gcc $(CFLAGS) -o $(TARGET) $(OBJS)
	@mkdir -p $(BIN_DIR)
	@cp $(TARGET) $(BIN_DIR)/$(PROJ_NAME)

$(OUT_DIR)/%.o: %.c
	mkdir -p $(OUT_DIR)
	gcc $(CFLAGS) -c -o $@ $<

run: $(TARGET)
	@echo "Running $(TARGET)"
	@$(TARGET)

clean:
	rm -rf $(OUT_DIR)
