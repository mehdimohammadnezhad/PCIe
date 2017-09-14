SUBDIRS := \
	driver \
	lib    \
	tools

%:
	@for ii in $(SUBDIRS); do \
		$(MAKE) $@ -C $$ii; \
	done

all:

