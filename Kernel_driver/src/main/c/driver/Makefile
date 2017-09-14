
ifneq ($(KERNELRELEASE),)

  obj-m           := sis8300drv.o
  ccflags-y       := -I$(src)/../include
  sis8300drv-objs := sis8300.o sis8300_ioctl.o sis8300_read.o sis8300_write.o sis8300_irq.o sis8300_dma.o sis8300_llseek.o sis8300_mmap.o

else

  KERNELDIR ?= /lib/modules/$(shell uname -r)/build
  PWD := $(shell pwd)


all: modules

modules modules_install clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) $@

.PHONY: all modules modules_install clean

endif
