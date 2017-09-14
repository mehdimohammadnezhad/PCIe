PROJECT = sis8300drv
include ${EPICS_ENV_PATH}/module.Makefile

USR_DEPENDENCIES += udev

EXCLUDE_ARCHS += eldk

# userspace library
SOURCES += $(wildcard src/main/c/lib/*.c)
HEADERS += $(wildcard src/main/c/include/*.h)
HEADERS += src/main/c/lib/sis8300drv_utils.h
HEADERS += src/main/c/lib/sis8300drv_list.h

# utility programs/tools
EXECUTABLES = 			\
	sis8300drv_acq		\
	sis8300drv_flashfw	\
	sis8300drv_i2c_rtm	\
	sis8300drv_irq		\
	sis8300drv_mem		\
	sis8300drv_mmap		\
	sis8300drv_out		\
	sis8300drv_reg		\
	sis8300drv_rem      \
	sis8300drv_i2c_temp

vpath %.c 								\
	../../src/main/c/tools/acquisition	\
	../../src/main/c/tools/flash		\
	../../src/main/c/tools/i2c_rtm		\
	../../src/main/c/tools/irq			\
	../../src/main/c/tools/memory		\
	../../src/main/c/tools/mmap			\
	../../src/main/c/tools/output		\
	../../src/main/c/tools/register		\
	../../src/main/c/tools/remove       \
	../../src/main/c/tools/i2c_temp

SIS8300DRV_TOOLS_LIB = -L . -lrt -lm -ludev -lsis8300drv

sis8300drv_acq: sis8300drv_acq.o libsis8300drv.so
	$(CCC) -o $@ $^ ${SIS8300DRV_TOOLS_LIB}

sis8300drv_flashfw: sis8300drv_flashfw.o libsis8300drv.so
	$(CCC) -o $@ $^ ${SIS8300DRV_TOOLS_LIB}

sis8300drv_i2c_rtm: sis8300drv_i2c_rtm.o libsis8300drv.so
	$(CCC) -o $@ $^ ${SIS8300DRV_TOOLS_LIB}
	
sis8300drv_irq: sis8300drv_irq.o libsis8300drv.so
	$(CCC) -o $@ $^ ${SIS8300DRV_TOOLS_LIB}
	
sis8300drv_mem: sis8300drv_mem.o libsis8300drv.so
	$(CCC) -o $@ $^ ${SIS8300DRV_TOOLS_LIB}
	
sis8300drv_mmap: sis8300drv_mmap.o libsis8300drv.so
	$(CCC) -o $@ $^ ${SIS8300DRV_TOOLS_LIB}
	
sis8300drv_out: sis8300drv_out.o libsis8300drv.so
	$(CCC) -o $@ $^ ${SIS8300DRV_TOOLS_LIB}
	
sis8300drv_reg: sis8300drv_reg.o libsis8300drv.so
	$(CCC) -o $@ $^ ${SIS8300DRV_TOOLS_LIB}
	
sis8300drv_rem: sis8300drv_mem.o libsis8300drv.so
	$(CCC) -o $@ $^ ${SIS8300DRV_TOOLS_LIB}

sis8300drv_i2c_temp: sis8300drv_i2c_temp.o libsis8300drv.so
	$(CCC) -o $@ $^ ${SIS8300DRV_TOOLS_LIB}
