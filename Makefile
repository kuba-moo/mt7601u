ifeq ($(KERNELRELEASE),)
KDIR ?= /lib/modules/`uname -r`/build
default:
	$(MAKE) -C $(KDIR) M=$$PWD
clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
else
EXTRA_CFLAGS += -Werror -Wenum-compare

obj-m := mt7601u.o

mt7601u-y := \
	usb.o init.o main.o mcu.o trace.o dma.o core.o eeprom.o phy.o \
	mac.o util.o debugfs.o tx.o
endif
