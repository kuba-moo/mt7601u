ifneq ($(KERNELRELEASE),)
KDIR ?= /lib/modules/$(KERNELRELEASE)/build
endif

KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD
clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
install:
	$(MAKE) -C $(KDIR) M=$$PWD modules_install
