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
CFLAGS := -mbig-endian -Wall -Wpointer-arith -Wstrict-prototypes -Winline -Wundef  -g -O0 -ffunction-sections -fdata-sections  -fno-exceptions  -finit-priority -mapcs-frame -D_KERNEL -D__ECOS -DCPU=33 -DXSCALE=33
#-Woverloaded-virtual -fvtable-gc -fno-rtti

LDFLAGS := -mbig-endian -Wl,--gc-sections -Wl,-static -g -O2 -nostdlib
#VPATH := $(REPOSITORY)/$(PACKAGE)
INCLUDE_PATH := $(INCLUDE_PATH) -I$(PREFIX)/include -I.
#$(foreach dir,$(VPATH),-I$(dir) -I$(dir)/src -I$(dir)/tests)
MLT := $(wildcard $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.ldi)
#$(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.h
TESTS := 

build: net.a

LIBRARY := net.a
#COMPILE := src/hal_misc.c src/context.S src/arm_stub.c src/hal_syscall.c
COMPILE := enet.c arp.c ip.c udp.c icmp.c ping.c tftp_client.c cksum.c inet_addr.c ticks.c load.c
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

