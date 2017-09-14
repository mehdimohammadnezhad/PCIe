# Will Build sis8300drv.ko
sis8300drv-objs := sis8300.o sis8300_ioctl.o sis8300_read.o sis8300_write.o sis8300_irq.o sis8300_dma.o sis8300_llseek.o sis8300_mmap.o

# Kernel Module:
obj-m := sis8300drv.o

# Headers
ccflags-y := -I ../include 

current_dir := $(shell pwd)
kernel_this := $(shell uname -r)
kernel_dir := /lib/modules/$(kernel_this)/build
target_dir := /lib/modules/$(kernel_this)/extra

default: modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.ko.unsigned *.mod.c .tmp_versions *.symvers *.order *.markers

modules modules_install:
	make -C "$(kernel_dir)" M=$(current_dir) ${ccflags-y} $@


