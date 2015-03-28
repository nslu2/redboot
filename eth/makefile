# eCos makefile

# This is a generated file - do not edit

export PREFIX :=../
export COMMAND_PREFIX := ../../i686-pc-linux-gnulibc2.2-x-xscale-elf/bin/xscale-elf-
export CC := $(COMMAND_PREFIX)gcc
export OBJCOPY := $(COMMAND_PREFIX)objcopy
export OBJDUMP := $(COMMAND_PREFIX)objdump
export HOST := UNIX
export AR := $(COMMAND_PREFIX)ar

#export REPOSITORY := /home/snowel/work/redboot/packages
#PACKAGE := hal/arm/arch/current
OBJECT_PREFIX := 
CFLAGS := -mbig-endian -Wall -Wpointer-arith -Wstrict-prototypes -Winline -Wundef  -g -O2 -ffunction-sections -fdata-sections  -fno-exceptions  -finit-priority -mapcs-frame -D_KERNEL -D__ECOS -DCPU=33 -DXSCALE=33
#-Woverloaded-virtual -fvtable-gc -fno-rtti

LDFLAGS := -mbig-endian -Wl,--gc-sections -Wl,-static -g -O2 -nostdlib
#VPATH := $(REPOSITORY)/$(PACKAGE)
INCLUDE_PATH := $(INCLUDE_PATH) -I$(PREFIX)/include -I. -I./include
#$(foreach dir,$(VPATH),-I$(dir) -I$(dir)/src -I$(dir)/tests)
MLT := $(wildcard $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.ldi)
#$(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.h
TESTS := 

build: npe.a

LIBRARY := npe.a
#COMPILE := src/hal_misc.c src/context.S src/arm_stub.c src/hal_syscall.c
COMPILE := if_npe.c \
	npeDl/IxNpeDl.c npeDl/IxNpeDlImageMgr.c npeDl/IxNpeDlNpeMgr.c npeDl/IxNpeDlNpeMgrUtils.c \
	osServices/IxOsBuffPoolMgt.c osServices/IxOsServices.c \
        qmgr/IxQMgrQCfg.c qmgr/IxQMgrDispatcher.c qmgr/IxQMgrInit.c \
	qmgr/IxQMgrAqmIf.c qmgr/IxQMgrQAccess.c \
	ethAcc/IxEthAcc.c ethAcc/IxEthAccCommon.c ethAcc/IxEthAccDataPlane.c \
	ethAcc/IxEthAccMac.c ethAcc/IxEthAccMii.c \
	npeMh/IxNpeMh.c npeMh/IxNpeMhConfig.c npeMh/IxNpeMhReceive.c npeMh/IxNpeMhSend.c \
	npeMh/IxNpeMhSolicitedCbMgr.c npeMh/IxNpeMhUnsolicitedCbMgr.c \
	ethDB/IxEthDBAPI.c ethDB/IxEthDBAPISupport.c ethDB/IxEthDBCore.c ethDB/IxEthDBEvents.c \
	ethDB/IxEthDBHashtable.c ethDB/IxEthDBLearning.c ethDB/IxEthDBMem.c \
        ethDB/IxEthDBNPEAdaptor.c ethDB/IxEthDBPortUpdate.c ethDB/IxEthDBUtil.c
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

