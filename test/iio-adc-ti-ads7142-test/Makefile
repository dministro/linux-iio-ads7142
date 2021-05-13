TOP := $(shell pwd)
BUILDDIR ?= $(TOP)/build
OUTNAME=iio-adc-ti-ads7142-test
DESTDIR=/usr

ifeq ($(V),1)
Q :=
ECHO := @true
else
Q := @
ECHO := @echo
endif

# Compilation tools
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
SIZE = $(CROSS_COMPILE)size
STRIP = $(CROSS_COMPILE)strip
OBJCOPY = $(CROSS_COMPILE)objcopy
GDB = $(CROSS_COMPILE)gdb
NM = $(CROSS_COMPILE)nm
AR = $(CROSS_COMPILE)ar
RL = $(CROSS_COMPILE)ranlib
RM := rm -rf

CFLAGS_LOCAL += $(ADDITIONALFLAGS) $(CPPFLAGS)
CFLAGS_LOCAL += -Wchar-subscripts -Wcomment
CFLAGS_LOCAL += -Werror-implicit-function-declaration -Wmain -Wparentheses
CFLAGS_LOCAL += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs
CFLAGS_LOCAL += -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef
CFLAGS_LOCAL += -Wshadow -Wpointer-arith -Wbad-function-cast -Wwrite-strings
CFLAGS_LOCAL += -Waggregate-return -Wstrict-prototypes
CFLAGS_LOCAL += -Wmissing-prototypes -Wmissing-declarations
CFLAGS_LOCAL += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CFLAGS_LOCAL += -Wredundant-decls -Wnested-externs -Winline
CFLAGS_LOCAL += -Wunreachable-code -Wno-aggregate-return
CFLAGS_LOCAL += -std=gnu99
CFLAGS_LOCAL += -Wmissing-prototypes -Wstrict-prototypes -Wunused-variable
CFLAGS_LOCAL += -Wmaybe-uninitialized -Wdiscarded-qualifiers

CFLAGS_DEFS=-D'__weak=__attribute__((weak))'  -D'__packed=__attribute__((__packed__))'
CFLAGS_LIBS=-lm -lc -lpthread

include $(TOP)/Makefile.vars

vpath %.c $(TOP)
vpath %.S $(TOP)

OBJS := $(addprefix $(BUILDDIR)/,$(obj-y))
LIBS := $(addprefix $(BUILDDIR)/,$(lib-y))

-include $(OBJS:.o=.d)

all: release

$(BUILDDIR):
	@mkdir -p $(BUILDDIR)

$(BUILDDIR)/%.d: %.c
	@mkdir -p $(dir $@)
	$(ECHO) DEP $<
	$(Q)$(CC) $(CFLAGS) $(CFLAGS_LOCAL) $(CFLAGS_CPU) $(CFLAGS_INC) $(CFLAGS_DEFS) $(CFLAGS_LIBS) -MM $< -MT $(basename $@).o -o $(basename $@).d

$(BUILDDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(ECHO) CC $<
	$(Q)$(CC) $(CFLAGS) $(CFLAGS_LOCAL) $(CFLAGS_CPU) $(CFLAGS_INC) $(CFLAGS_DEFS) $(CFLAGS_LIBS) -c $< -o $@

$(BUILDDIR)/%.d: %.S
	@mkdir -p $(dir $@)
	$(ECHO) DEP $<
	$(Q)$(CC) $(CFLAGS_ASM) $(CFLAGS_CPU) $(CFLAGS_INC) $(CFLAGS_DEFS) $(CFLAGS_LIBS) -MM $< -MT $(basename $@).o -o $(basename $@).d

$(BUILDDIR)/%.o: %.S
	@mkdir -p $(dir $@)
	$(ECHO) CC $<
	$(Q)$(CC) $(CFLAGS_ASM) $(CFLAGS_CPU) $(CFLAGS_INC) $(CFLAGS_DEFS) $(CFLAGS_LIBS) -c $< -o $@

$(BUILDDIR)/$(OUTNAME): $(OBJS) $(LIBS)
	$(Q)$(CC) -o $@ $^ $(CFLAGS) $(CFLAGS_LOCAL) $(LIBS) $(CFLAGS_CPU) $(CFLAGS_INC) $(CFLAGS_DEFS) $(CFLAGS_LIBS)

debug: ADDITIONALFLAGS=-DDEBUG -g -O0

debug: $(BUILDDIR)/$(OUTNAME)

release: $(BUILDDIR)/$(OUTNAME)

clean:
	@rm -rf $(BUILDDIR)

