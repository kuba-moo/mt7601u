EXTRA_CFLAGS += -Werror -Wenum-compare
ccflags-y += -D__CHECK_ENDIAN__

obj-m := mt7601u.o

mt7601u-y := \
	usb.o init.o main.o mcu.o trace.o dma.o core.o eeprom.o phy.o \
	mac.o util.o debugfs.o tx.o

CFLAGS_trace.o := -I$(src)
