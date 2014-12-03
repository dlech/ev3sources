ifneq ($(KERNELRELEASE),)

# kbuild part of makefile
obj-m += d_pwm.o

else

# normal makefile

TARGET = d_pwm.ko

CONF = Linuxmod
ARCH = AM1808

include ../../install/rules.mk

endif
