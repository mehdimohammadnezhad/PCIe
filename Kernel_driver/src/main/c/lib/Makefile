
# shared liblary name
NAME := sis8300drv

# output destinations
TARGETSO := lib$(NAME).so
TARGETA := lib$(NAME).a
OBJDIR := obj

CCFLAGS := -fPIC -Wall -g

LDFLAGS := -Wall -lpthread

# files
OBJS := sis8300drv      \
	sis8300drv_ad9510   \
	sis8300drv_flash    \
	sis8300drv_rtm      \
	sis8300drv_utils
	
	
INCLUDES := . ../include

LIBS :=

###############################################################################

# rules

all: $(TARGETSO) $(TARGETA)

clean:
	rm -rf *~ $(TARGETA) $(TARGETSO) $(OBJDIR)

$(TARGETSO): $(OBJS:%=$(OBJDIR)/%.o) 
	gcc -shared $(LDFLAGS) $(LIBS:%=-l%) -o  $@ $^

$(OBJDIR)/%.o: %.c
	mkdir -p $(OBJDIR)
	gcc -c $(CCFLAGS) $(INCLUDES:%=-I%) -o $@ $<

$(TARGETA): $(OBJS:%=$(OBJDIR)/%.o) 
	/usr/bin/ar -rcs		$@		$^ 

