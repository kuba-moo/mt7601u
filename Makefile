ifeq ($(KERNELRELEASE),)
default:
	$(MAKE) -C /lib/modules/`uname -r`/build M=$$PWD
else
EXTRA_CFLAGS += -Werror -Wenum-compare

obj-m := mt7601u.o

mt7601u-y := \
	usb.o init.o main.o mcu.o trace.o dma.o core.o eeprom.o phy.o \
	mac.o util.o debugfs.o tx.o
endif
