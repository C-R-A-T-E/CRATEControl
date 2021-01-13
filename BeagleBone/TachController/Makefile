# This Makefile hacked together from TI, Cloud9 & Exploring BeagleBone Makefiles
#

TARGET = bin/tach_control

SUBDIRS = pru/ arm/
BIN_DIR = bin/

all: $(SUBDIRS)

$(SUBDIRS):
	@$(MAKE) -C $@

run: $(SUBDIRS) $(TARGET)
	@echo "Running $(TARGET)"
	@$(TARGET)

clean:
	@for d in $(SUBDIRS); do (cd $$d; $(MAKE) clean ); done
	rm -rf $(BIN_DIR)

.PHONY: all clean $(SUBDIRS)



