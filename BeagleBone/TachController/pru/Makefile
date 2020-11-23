# Makefile hacked up from TI, Cloud9 & Exploring BeagleBone
#

PROJ_NAME = TachController

PRU_CGT =/usr/lib/ti/pru-software-support-package
PRU_CGT_CUSTOM = ../../pru-software-support-package

MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
CURRENT_DIR := $(notdir $(patsubst %/,%,$(dir $(MKFILE_PATH))))

CHIP = am57xx
CHIP_REV = am572x_2_0

LIBS =--library=$(PRU_CGT)/lib/rpmsg_lib.lib
INCLUDE = --include_path=$(PRU_CGT)/include --include_path=$(PRU_CGT_CUSTOM)/include/$(CHIP_REV) --include_path=$(PRU_CGT)/include/$(CHIP_REV)
STACK_SIZE = 0x100
HEAP_SIZE = 0x100
OUT_DIR = out

LINKER_COMMAND_FILE=./$(CHIP)_pru.cmd

PRU1_0 :=/sys/class/remoteproc/remoteproc4
PRU1_1 :=/sys/class/remoteproc/remoteproc5
PRU2_0 :=/sys/class/remoteproc/remoteproc6
PRU2_1 :=/sys/class/remoteproc/remoteproc7

#Common compiler and linker flags (Defined in 'PRU Optimizing C/C++ Compiler User's Guide)
CFLAGS = -v3 -O2 --display_error_number --endian=little --hardware_mac=on --obj_directory=$(OUT_DIR) --pp_directory=$(OUT_DIR) -ppd -ppa
#Linker flags (Defined in 'PRU Optimizing C/C++ Compiler User's Guide)
LFLAGS = --reread_libs --warn_sections --stack_size=$(STACK_SIZE) --heap_size=$(HEAP_SIZE)

TARGET = $(OUT_DIR)/$(PROJ_NAME).out
MAP = $(OUT_DIR)/$(PROJ_NAME).map
SOURCES = $(wildcard *.c)
OBJS     =$(patsubst %,$(OUT_DIR)/%,$(SOURCES:.c=.obj))

all: $(TARGET)

install_PRU:
	@echo ''
	@echo 'Stopping current PRU1_0 application ($(PRU1_0))'
	@echo stop | sudo tee $(PRU1_0)/state
	@echo 'Installing firmware'
	@sudo cp $(TARGET) /lib/firmware/am335x-pru0-fw
	@echo 'Deploying firmware'
	@echo am335x-pru0-fw | sudo tee $(PRU1_0)/firmware
	@echo 'Starting new PRU1_0 application'
	@echo start | sudo tee $(PRU1_0)/state

# Invokes the linker (-z flag) to make the .out file
$(TARGET): $(OBJS) $(LINKER_COMMAND_FILE)
	@echo ''
	@echo 'Building target: $@'
	@echo 'Invoking: PRU Linker'
	/usr/bin/clpru $(CFLAGS) -z -i$(PRU_CGT)/lib -i$(PRU_CGT)/include $(LFLAGS) -o $(TARGET) $(OBJS) -m$(MAP) $(LINKER_COMMAND_FILE) --library=libc.a $(LIBS)
	@echo 'Finished building target: $@'
	@echo ''
	@echo 'Output files can be found in the "$(OUT_DIR)" directory'

# Invokes the compiler on all c files in the directory to create the.obj files
$(OUT_DIR)/%.obj: %.c
	@mkdir -p $(OUT_DIR)
	@echo ''
	@echo 'Building file: $<'
	@echo 'Invoking: PRU Compiler'
	/usr/bin/clpru --include_path=$(PRU_CGT)/include $(INCLUDE) $(CFLAGS) -fe $@ $<

.PHONY: all clean

# Remove the $(OUT_DIR) directory
clean:
	@echo 'Removing files in the "$(OUT_DIR)" directory'
	@rm -rf $(OUT_DIR)

# Includes the dependencies that the compiler creates (-ppd and -ppa flags)
-include $(OBJS:%.obj=%.pp)
