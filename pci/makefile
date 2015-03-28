# eCos makefile

# This is a generated file - do not edit

export PREFIX :=..
export COMMAND_PREFIX := arm-linux-
export CC := $(COMMAND_PREFIX)gcc
export OBJCOPY := $(COMMAND_PREFIX)objcopy
export OBJDUMP := $(COMMAND_PREFIX)objdump
export HOST := UNIX
export AR := $(COMMAND_PREFIX)ar

#export REPOSITORY := /home/snowel/work/redboot/packages
#PACKAGE := hal/arm/arch/current
OBJECT_PREFIX := 
CFLAGS := -mbig-endian -Wall -Wpointer-arith -Wstrict-prototypes -Winline -Wundef  -g -O2 -ffunction-sections -fdata-sections  -fno-exceptions  -finit-priority -mapcs-frame -D_KERNEL -D__ECOS -DCPU=33 -DXSCALE=33  -DSK_USE_CSUM  -DYUKON -DDEBUG #-DGENESIS -DSK_DIAG_SUPPORT
#-Woverloaded-virtual -fvtable-gc -fno-rtti

LDFLAGS := -mbig-endian -Wl,--gc-sections -Wl,-static -g -O2 -nostdlib
#VPATH := $(REPOSITORY)/$(PACKAGE)
INCLUDE_PATH := $(INCLUDE_PATH) -I$(PREFIX)/include -I.
#$(foreach dir,$(VPATH),-I$(dir) -I$(dir)/src -I$(dir)/tests)
MLT := $(wildcard $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.ldi)
#$(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.h
TESTS := 

build: pci.a

LIBRARY := pci.a
COMPILE := pci.c pci_hw.c ixp425_pci.c ixdp425_pci.c pci_test.c sk98/skge.c sk98/skgeinit.c sk98/skxmac2.c sk98/skcsum.c sk98/skgepnmi.c sk98/ski2c.c sk98/skrlmt.c sk98/skvpd.c sk98/skaddr.c  sk98/skgehwt.c  sk98/skgesirq.c sk98/sklm80.c sk98/skqueue.c sk98/sktimer.c
 #sk98/skdim.c #sk98/skgemib.c

OBJECTS := $(COMPILE:.cxx=.o.d)
OBJECTS := $(OBJECTS:.c=.o.d)
OBJECTS := $(OBJECTS:.S=.o.d)

$(LIBRARY): $(OBJECTS)
	$(AR) rcs $@ $(foreach obj,$?,$(dir $(obj))$(notdir $(obj:.o.d=.o)))
	@cat $^ > $(@:=.deps)
	@touch $@


clean: 
	rm -f  $(LIBRARY) $(LIBRARY).deps $(OBJECTS) $(OBJECTS:.o.d=.o) 

include ../rules.mak

