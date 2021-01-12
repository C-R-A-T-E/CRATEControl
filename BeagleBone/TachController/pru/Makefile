# Makefile hacked up from TI, Cloud9 & Exploring BeagleBone
#

PROJ_NAME = TachControl

CHIP = am57xx
CHIP_REV = am572x_2_0

PRU_CGT =/usr/lib/ti/pru-software-support-package
PRU_CGT_CUSTOM = ../../pru-software-support-package

CORE = ../../Core
CORE_OUT_DIR = $(CORE)/pru/out
CORE_OBJS = $(CORE_OUT_DIR)/pru_time.o $(CORE_OUT_DIR)/pru_util.o

LIBS =--library=$(PRU_CGT)/lib/rpmsg_lib.lib
INCLUDE = --include_path=$(PRU_CGT)/include --include_path=$(PRU_CGT_CUSTOM)/include/$(CHIP_REV) --include_path=$(PRU_CGT)/include/$(CHIP_REV) --include_path=$(CORE)/pru

STACK_SIZE = 0x100
HEAP_SIZE = 0x100
OUT_DIR = out

LINKER_COMMAND_FILE=./$(CHIP)_pru.cmd

PRU1_0 = /sys/class/remoteproc/remoteproc4
PRU1_1 = /sys/class/remoteproc/remoteproc5
PRU2_0 = /sys/class/remoteproc/remoteproc6
PRU2_1 = /sys/class/remoteproc/remoteproc7

CFLAGS = -v3 -O2 --display_error_number --endian=little --hardware_mac=on --obj_directory=$(OUT_DIR) --pp_directory=$(OUT_DIR) -ppd -ppa
LFLAGS = --reread_libs --warn_sections --stack_size=$(STACK_SIZE) --heap_size=$(HEAP_SIZE)

TARGET = $(OUT_DIR)/$(PROJ_NAME)
FIRMWARE = am57xx-pru1_1-tach_control

MAP = $(OUT_DIR)/$(PROJ_NAME).map
SOURCES = $(wildcard *.c)
OBJS    = $(patsubst %,$(OUT_DIR)/%,$(SOURCES:.c=.o))

all: $(TARGET)

$(TARGET): $(OBJS) $(CORE_OBJS) $(LINKER_COMMAND_FILE)
	/usr/bin/clpru $(CFLAGS) -z -i$(PRU_CGT)/lib -i$(PRU_CGT)/include $(LFLAGS) -o $(TARGET) $(OBJS) $(CORE_OBJS) -m$(MAP) $(LINKER_COMMAND_FILE) --library=libc.a $(LIBS)

$(OUT_DIR)/%.o: %.c
	@mkdir -p $(OUT_DIR)
	/usr/bin/clpru --include_path=$(PRU_CGT)/include $(INCLUDE) $(CFLAGS) -fe $@ $<

$(CORE_OUT_DIR)/pru_time.o: $(CORE)/pru/pru_time.c
	@mkdir -p $(CORE_OUT_DIR)
	/usr/bin/clpru --include_path=$(PRU_CGT)/include $(INCLUDE) $(CFLAGS) -fe $@ $<

$(CORE_OUT_DIR)/pru_util.o: $(CORE)/pru/pru_util.c
	@mkdir -p $(CORE_OUT_DIR)
	/usr/bin/clpru --include_path=$(PRU_CGT)/include $(INCLUDE) $(CFLAGS) -fe $@ $<

run: $(TARGET) stop
	@echo "Installing $(TARGET) as frimware"
	@echo "Must run as privileged user, using sudo"
	@sudo cp $(TARGET) /lib/firmware/$(FIRMWARE)
	@echo $(FIRMWARE) > $(PRU1_1)/firmware
	@echo "Starting $(PRU1_1)"
	@echo start > $(PRU1_1)/state


stop: 
	@PRU1_1_STATE=`cat $(PRU1_1)/state`; \
	if [ $$PRU1_1_STATE = "running" ]; then \
		echo "Stopping $(PRU1_1)"; \
		echo stop > $(PRU1_1)/state; \
	fi


.PHONY: all clean

# Remove the $(OUT_DIR) directory
clean:
	rm -rf $(OUT_DIR)
	rm -rf $(CORE_OUT_DIR)

# Includes the dependencies that the compiler creates (-ppd and -ppa flags)
-include $(OBJS:%.o=%.pp)
