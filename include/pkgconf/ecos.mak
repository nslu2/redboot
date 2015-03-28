ECOS_GLOBAL_CFLAGS = -mbig-endian -Wall -Wpointer-arith -Wstrict-prototypes -Winline -Wundef -Woverloaded-virtual -g -O2 -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fvtable-gc -finit-priority -mapcs-frame
ECOS_GLOBAL_LDFLAGS = -mbig-endian -Wl,--gc-sections -Wl,-static -g -O2 -nostdlib
ECOS_COMMAND_PREFIX = xscale-elf-
