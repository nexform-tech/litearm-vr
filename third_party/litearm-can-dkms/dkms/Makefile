# Out-of-tree build of the patched gs_usb (pre-build.sh puts the right
# baseline in place as gs_usb.c first).
obj-m := gs_usb.o

KDIR ?= /lib/modules/$(KERNELRELEASE)/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
