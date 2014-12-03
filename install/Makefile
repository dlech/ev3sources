ARCH = AM1808

BASE = ..

KERNEL = $(BASE)/extra/linux-03.20.00.13

KERNEL_CONF = LEGOBoard.config

default: help

include rules.mk

#
# ev314 program.
#

ev314:
	$(MAKE) -C $(BASE)/ev314/Linux_$(ARCH)

ev314.clean: 
	$(MAKE) -C $(BASE)/ev314/Linux_$(ARCH) clean uninstall

.PHONY: ev314 ev314.clean 

#
# ev314 modules.
#

MODULES = d_analog d_bt d_iic d_power d_pwm d_sound d_uart d_ui d_usbdev \
	  d_usbhost

modules: $(MODULES)

$(MODULES):
	$(MAKE) -C $(BASE)/$@/Linuxmod_$(ARCH)

modules.clean: $(MODULES:%=%.clean)

$(MODULES:%=%.clean): %.clean:
	$(MAKE) -C $(BASE)/$*/Linuxmod_$(ARCH) clean uninstall

.PHONY: modules $(MODULES) modules.clean $(MODULES:%=%.clean)

#
# Kernel.
#

kernel: uImage

$(KERNEL)/.config: $(KERNEL_CONF)
	cp $< $@

kernel.prepare: $(KERNEL)/include/generated/bounds.h
$(KERNEL)/include/generated/bounds.h: $(KERNEL)/.config
	$(MAKE) -C $(KERNEL) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) oldconfig modules_prepare

$(KERNEL)/arch/arm/boot/uImage: kernel.prepare | $(MKIMAGE_CHECK)
	$(MAKE) -C $(KERNEL) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) uImage

uImage: $(KERNEL)/arch/arm/boot/uImage
	cp $< $@

kernel.clean:
	$(MAKE) -C $(KERNEL) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) clean

.PHONY: kernel kernel.prepare kernel.clean

#
# Misc.
#

help:
	@echo 'Execute "make all" to build everything'
	@echo 'Execute "make install" to flash a SD card'
	@echo 'Execute "make TARGET" where TARGET is:'
	@echo ' ev314: to build the ev314 program
	@echo ' modules: to build lms2012 kernel modules'
	@echo ' kernel: to build Linux kernel'
	@echo 
	@echo 'Execute "make TARGET.clean" to clean an individual target'
	@echo 'or "make clean" to clean everything'
	@echo

clean: ev314.clean modules.clean kernel.clean
	rm ./.path-check &>/dev/null

all: kernel modules ev314
	./flash_sdcard.sh

.PHONY: help clean all
