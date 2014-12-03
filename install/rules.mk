#
# ARCH defines
#

ifeq ($(ARCH),AM1808)
CROSS_COMPILE = arm-none-linux-gnueabi-
else
$(error unknown ARCH)
endif

#
# Libraries and programs.
#

ifneq ($(filter Linux Linuxlib,$(CONF)),)

BASE = ../..

OBJS = $(SOURCES:%.c=%.o)
DEPS = $(OBJS:%.o=%.d)

INCLUDES += -I$(BASE)/lms2012/source

DEVKIT = $(BASE)/extra/linux-devkit/arm-none-linux-gnueabi
INCLUDES += -I$(DEVKIT)/usr/include/dbus-1.0
INCLUDES += -I$(DEVKIT)/usr/lib/dbus-1.0/include
INCLUDES += -I$(DEVKIT)/usr/include
CFLAGS += -DPCASM $(INCLUDES) -O0 -Wall -fPIC
LDFLAGS += -L$(BASE)/ev314/Linux_$(ARCH)/sys/lib -L$(DEVKIT)/usr/lib

ifeq ($(CONF),Linuxlib)
LDFLAGS += -shared
INSTALL_DIR = sys/lib
else
INSTALL_DIR = sys
VPATH = sys/lib
endif

INSTALL_TARGET = $(BASE)/ev314/Linux_$(ARCH)/$(INSTALL_DIR)/$(TARGET)

all: install

%.o: ../source/%.c
	$(CROSS_COMPILE)gcc $(CFLAGS) -c -MMD -MP -o $@ $<

$(TARGET): $(OBJS) $(filter -lc_%,$(LIBS))
	$(CROSS_COMPILE)gcc $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

install: $(INSTALL_TARGET)
$(INSTALL_TARGET): $(TARGET)
	mkdir -p $(dir $@) && cp $< $@

$(SUBDIRS:%=-l%): FORCE
	$(MAKE) -C ../../$(@:-l%=%)/Linuxlib_$(ARCH) install

clean:
	rm -f $(OBJS) $(DEPS) $(TARGET)

uninstall:
	rm -f $(INSTALL_TARGET)

.PHONY: all install clean uninstall FORCE
FORCE:

-include $(DEPS)

endif

#
# Kernel modules.
#

ifeq ($(CONF),Linuxmod)

BASE = ../..

KDIR ?= $(BASE)/extra/linux-03.20.00.13
KERNEL_MAKEFLAGS = ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE)
PREPARE = kernel.prepare

INSTALL_DIR = sys/mod
INSTALL_TARGET = $(BASE)/ev314/Linux_$(ARCH)/$(INSTALL_DIR)/$(TARGET)

all: install

$(TARGET): $(PREPARE)
	$(MAKE) $(KERNEL_MAKEFLAGS) -C $(KDIR) M=$$PWD

kernel.prepare:
	$(MAKE) -C $(BASE)/install kernel.prepare

install: $(INSTALL_TARGET)
$(INSTALL_TARGET): $(TARGET)
	mkdir -p $(dir $@) && cp $< $@

clean:
	$(MAKE) $(KERNEL_MAKEFLAGS) -C $(KDIR) M=$$PWD clean

uninstall:
	rm -f $(INSTALL_TARGET)

.PHONY: all install clean uninstall

endif

#
# Automagic PATH configuration.
#

PATH_CHECK = $(BASE)/install/.path-check
PATH_CHECK_TRY = $(BASE)/CodeSourcery/Sourcery_G++_Lite/bin \
		 $(HOME)/CodeSourcery/Sourcery_G++_Lite/bin

$(PATH_CHECK):
	@if ! which $(CROSS_COMPILE)gcc > /dev/null; then \
		for d in $(PATH_CHECK_TRY); do \
			test -x $$d/$(CROSS_COMPILE)gcc && found=$$d; \
		done; \
		if test -z "$$found"; then \
			echo "##################" >&2; \
			echo "# Can not find $(CROSS_COMPILE)gcc, please install it." >&2; \
			echo "##################" >&2; \
			echo >&2; \
		else \
			cur_dir=`pwd`;cd "$$found";found_dir=`pwd`;cd "$$cur_dir"; \
			echo 'export PATH := '$$found_dir':$$(PATH)' > $@; \
		fi; \
	else \
		touch $@; \
	fi

-include $(PATH_CHECK)

MKIMAGE_CHECK = $(BASE)/install/.mkimage-check

$(MKIMAGE_CHECK):
	@if ! which mkimage > /dev/null; then \
		echo "##################" >&2; \
		echo "# Can not find mkimage, please install u-boot-tools package." >&2; \
		echo "##################" >&2; \
		echo >&2; \
		false; \
	else \
		touch $@; \
	fi
