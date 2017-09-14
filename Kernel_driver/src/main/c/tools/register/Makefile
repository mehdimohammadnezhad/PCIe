FILES := $(wildcard *)
SOURCES := $(filter %.c,$(FILES))
OBJECTS := $(SOURCES:.c=)

CC := gcc
CCFLAGS := -O3 -std=c99 -D_GNU_SOURCE -Wall -I../../include -I../../lib
LDFLAGS := -lpthread -lsis8300drv -lrt -lm -ludev -L../../lib

all:
	@$(MAKE) $(OBJECTS)

%: %.c
	$(CC) $(CCFLAGS) $(LDFLAGS) -o $@ $<
	
clean:
	rm $(OBJECTS)
