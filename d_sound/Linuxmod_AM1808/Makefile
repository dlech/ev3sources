ifneq ($(KERNELRELEASE),)

# kbuild part of makefile
obj-m += d_sound.o

else

# normal makefile

TARGET = d_sound.ko

CONF = Linuxmod
ARCH = AM1808

include ../../install/rules.mk

endif
