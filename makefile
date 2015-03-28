# eCos makefile

# This is a generated file - do not edit

export PREFIX :=.
export COMMAND_PREFIX := arm-linux-
export CC := $(COMMAND_PREFIX)gcc
export OBJCOPY := $(COMMAND_PREFIX)objcopy
export OBJDUMP := $(COMMAND_PREFIX)objdump
export HOST := UNIX
export AR := $(COMMAND_PREFIX)ar

#export REPOSITORY := /home/snowel/work/redboot/packages
#PACKAGE := hal/arm/arch/current
OBJECT_PREFIX := 

#CFLAGS := -mbig-endian -Wall -Wpointer-arith -Wstrict-prototypes -Winline -Wundef  -g -O2 -ffunction-sections -fdata-sections  -fno-exceptions  -finit-priority -mapcs-frame -D_KERNEL -D__ECOS -DCPU=33 -DXSCALE=33
#-Woverloaded-virtual -fvtable-gc -fno-rtti
CFLAGS := -mbig-endian -Wall -Wpointer-arith -Wstrict-prototypes -Winline -Wundef -Woverloaded-virtual -g -Os -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fvtable-gc -finit-priority -mapcs-frame

LDFLAGS := -mbig-endian -Wl,--gc-sections -Wl,-static -g -O2 -nostdlib
#VPATH := $(REPOSITORY)/$(PACKAGE)
INCLUDE_PATH := $(INCLUDE_PATH) -I$(PREFIX)/include -I.
#$(foreach dir,$(VPATH),-I$(dir) -I$(dir)/src -I$(dir)/tests)
MLT := $(wildcard $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.ldi)
#$(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.h
TESTS := 

MODEL_NAME := redboot
TARGET := $(MODEL_NAME).bin


$(TARGET): $(MODEL_NAME).elf
	$(OBJCOPY) --strip-debug $< $(@:.bin=.img) 
	$(OBJCOPY) -O srec $< $(@:.bin=.srec)
	$(OBJCOPY) -O binary $< $@
	$(OBJDUMP) -x $< >$(@:.bin=.map)
#build:  libtarget.a  vectors.o

LIBRARY := libtarget.a
#COMPILE := src/hal_misc.c src/context.S src/arm_stub.c src/hal_syscall.c
COMPILE := context.S ixp425_misc.c arch_hal_misc.c common_hal_misc.c xscale_misc.c hal_if.c ixp425_diag.c grg_misc.c drv_api.c diag.cxx io.c  parse.c eth_drv.c  net_io.c enet.c upgrade.c flash.c iob.c main.c version.c timer.c redboot_linux_exec.c  cmd.c
#memcmp.cxx strcmp.cxx strsuppt.cxx memcpy.c pktbuf.c
OBJECTS := $(COMPILE:.cxx=.o.d)
OBJECTS := $(OBJECTS:.c=.o.d)
OBJECTS := $(OBJECTS:.S=.o.d)

$(LIBRARY): $(OBJECTS)
	$(AR) rcs $(PREFIX)/$@ $(foreach obj,$?,$(dir $(obj))$(notdir $(obj:.o.d=.o)))
	@cat $^ > $(@:=.deps)
	@touch $@


vectors.o: vectors.S
	$(CC) -Wp,-MD,vectors.tmp $(INCLUDE_PATH) $(CFLAGS) -c -o $@ $<
	#@echo $@ ": \\" > $(notdir $@).deps
	@cat vectors.tmp >> $(notdir $@).deps
	@echo >> $(notdir $@).deps
	@rm vectors.tmp


LIBSS := $(PREFIX)/lib/lib.a $(PREFIX)/pci/pci.a $(PREFIX)/net/net.a #$(PREFIX)/eth/npe.a

#$(PREFIX)/lib/target.ld: $(wildcard $(REPOSITORY)/$(PACKAGE)/src/arm.ld)
#	$(CC) -E -P -Wp,-MD,target.tmp -xc $(INCLUDE_PATH) $(CFLAGS) -o $@ $<
#	@echo $@ ": \\" > $(notdir $@).deps
#	@tail +2 target.tmp >> $(notdir $@).deps
#	@echo >> $(notdir $@).deps
#	@rm target.tmp
#	

#redboot.elf: $(wildcard target.ld $(PREFIX)/lib/libtarget.a $(PREFIX)/lib/libextras.a)
redboot.elf: vectors.o $(LIBRARY) $(LIBSS)
	@sh -c "mkdir -p $(dir $@)"
	$(CC)  $(LIBRARY)  $(LIBSS)  $(LDFLAGS) -Ttarget.ld -o $@

#$(PREFIX)/lib/version.o

FORCE:

clean: CMD = clean

clean: $(LIBSS)
	rm -f vectors.o vectors.o.deps $(LIBRARY) $(LIBRARY).deps $(OBJECTS) $(OBJECTS:.o.d=.o) $(MODEL_NAME).* $(LIBSS)

$(PREFIX)/eth/npe.a: FORCE
	cd $(@D);$(MAKE) $(CMD)

$(PREFIX)/lib/lib.a: FORCE
	cd $(@D);$(MAKE) $(CMD)

$(PREFIX)/pci/pci.a: FORCE
	cd $(@D);$(MAKE) $(CMD)

$(PREFIX)/net/net.a: FORCE
	cd $(@D);$(MAKE) $(CMD)

include ./rules.mak

