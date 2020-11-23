# This Makefile hacked together from TI, Cloud9 & Exploring BeagleBone Makefiles
#

SUBDIRS = arm/ pru/
all: $(SUBDIRS)

$(SUBDIRS):
	@$(MAKE) -C $@

clean:
	@for d in $(SUBDIRS); do (cd $$d; $(MAKE) clean ); done

.PHONY: all clean $(SUBDIRS)



