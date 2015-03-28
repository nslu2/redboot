
#define  DEFINE_VARS
#include <redboot.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/hal_if.h>
#include <cyg/hal/hal_cache.h>
#include CYGHWR_MEMORY_LAYOUT_H

#ifdef CYGPKG_IO_ETH_DRIVERS
#include <cyg/io/eth/eth_drv.h>            // Logical driver interfaces
#endif

#include <cyg/hal/hal_tables.h>

#ifdef CYGDBG_HAL_DEBUG_GDB_INCLUDE_STUBS
#ifdef CYGBLD_HAL_PLATFORM_STUB_H
#include CYGBLD_HAL_PLATFORM_STUB_H
#else
#include <cyg/hal/plf_stub.h>
#endif
// GDB interfaces
extern void breakpoint(void);
#endif
#include <iob.h>

// Define table boundaries
CYG_HAL_TABLE_BEGIN( __RedBoot_INIT_TAB__, RedBoot_inits );
CYG_HAL_TABLE_END( __RedBoot_INIT_TAB_END__, RedBoot_inits );
extern struct init_tab_entry __RedBoot_INIT_TAB__[], __RedBoot_INIT_TAB_END__;

CYG_HAL_TABLE_BEGIN( __RedBoot_CMD_TAB__, RedBoot_commands );
CYG_HAL_TABLE_END( __RedBoot_CMD_TAB_END__, RedBoot_commands );
extern struct cmd __RedBoot_CMD_TAB__[], __RedBoot_CMD_TAB_END__;

CYG_HAL_TABLE_BEGIN( __RedBoot_IDLE_TAB__, RedBoot_idle );
CYG_HAL_TABLE_END( __RedBoot_IDLE_TAB_END__, RedBoot_idle );
extern struct idle_tab_entry __RedBoot_IDLE_TAB__[], __RedBoot_IDLE_TAB_END__;

void
do_idle(bool is_idle)
{
    struct idle_tab_entry *idle_entry;

    for (idle_entry = __RedBoot_IDLE_TAB__; 
         idle_entry != &__RedBoot_IDLE_TAB_END__;  idle_entry++) {
        (*idle_entry->fun)(is_idle);
    }
}

void
do_version(int argc, char *argv[])
{
#ifdef CYGPKG_REDBOOT_FLASH
    externC void _flash_info(void);
#endif
    char *version = CYGACC_CALL_IF_MONITOR_VERSION();

    diag_printf(version);
#ifdef HAL_PLATFORM_CPU
    diag_printf("Platform: %s (%s) %s\n", HAL_PLATFORM_BOARD, HAL_PLATFORM_CPU, HAL_PLATFORM_EXTRA);
#endif
    diag_printf("Copyright (C) 2000, 2001, 2002, Red Hat, Inc.\n\n");
    diag_printf("RAM: %p-%p, %p-%p available\n", 
                (void*)ram_start, (void*)ram_end,
                (void*)user_ram_start, (void *)user_ram_end);
#ifdef CYGPKG_REDBOOT_FLASH
    _flash_info();
#endif
}

// Wrapper used by diag_printf()
static void
_mon_write_char(char c, void **param)
{
    if (c == '\n') {
        mon_write_char('\r');
    }
    mon_write_char(c);
}

static void * go_saved_context;
static int go_return_status;

static void
return_to_redboot(int status)
{
    CYGARC_HAL_SAVE_GP();

    go_return_status = status;
    HAL_THREAD_LOAD_CONTEXT(&go_saved_context);
    // never returns

    // need this to balance above CYGARC_HAL_SAVE_GP on
    // some platforms. It will never run, though.
    CYGARC_HAL_RESTORE_GP();
}

void reset(void)
{
		 diag_printf("... Resetting.");
		 CYGACC_CALL_IF_DELAY_US(2*100000);
		 diag_printf("\n");
		 CYGACC_CALL_IF_RESET();
		 diag_printf("!! oops, RESET not working on this platform\n");  
}
void
cyg_start(void)
{
    int cur;
    struct init_tab_entry *init_entry;
    extern char RedBoot_version[];
    unsigned long timeNow,timeOld;
    unsigned char myaddr[8]={ 0x00, 0xC0, 0x02, 0x01, 0x01, 0x01};
    int use_cmd;

    // Export version information
    CYGACC_CALL_IF_MONITOR_VERSION_SET(RedBoot_version);

    CYGACC_CALL_IF_MONITOR_RETURN_SET(return_to_redboot);

    // Make sure the channels are properly initialized.
    diag_init_putc(_mon_write_char);
    hal_if_diag_init();

    // Force console to output raw text - but remember the old setting
    // so it can be restored if interaction with a debugger is
    // required.
    cur = CYGACC_CALL_IF_SET_CONSOLE_COMM(CYGNUM_CALL_IF_SET_COMM_ID_QUERY_CURRENT);
    CYGACC_CALL_IF_SET_CONSOLE_COMM(CYGNUM_HAL_VIRTUAL_VECTOR_DEBUG_CHANNEL);
#ifdef CYGPKG_REDBOOT_ANY_CONSOLE
    console_selected = false;
#endif
    console_echo = true;
    CYGACC_CALL_IF_DELAY_US((cyg_int32)2*100000);

    ram_start = (unsigned char *)CYGMEM_REGION_ram;
    ram_end = (unsigned char *)(CYGMEM_REGION_ram+CYGMEM_REGION_ram_SIZE);
#ifdef HAL_MEM_REAL_REGION_TOP
    {
        unsigned char *ram_end_tmp = ram_end;
        ram_end = HAL_MEM_REAL_REGION_TOP( ram_end_tmp );
    }
#endif
#ifdef CYGMEM_SECTION_heap1
    workspace_start = (unsigned char *)CYGMEM_SECTION_heap1;
    workspace_end = (unsigned char *)(CYGMEM_SECTION_heap1+CYGMEM_SECTION_heap1_SIZE);
    workspace_size = CYGMEM_SECTION_heap1_SIZE;
#else
    workspace_start = (unsigned char *)CYGMEM_REGION_ram;
    workspace_end = (unsigned char *)(CYGMEM_REGION_ram+CYGMEM_REGION_ram_SIZE);
    workspace_size = CYGMEM_REGION_ram_SIZE;
#endif

    if ( ram_end < workspace_end ) {
        // when *less* SDRAM is installed than the possible maximum,
        // but the heap1 region remains greater...
        workspace_end = ram_end;
        workspace_size = workspace_end - workspace_start;
    }

    //    bist();
    
    //    for (init_entry = __RedBoot_INIT_TAB__; init_entry != &__RedBoot_INIT_TAB_END__;  init_entry++) {
    //    (*init_entry->fun)();
    //}
    
    //    net_init();
    //snowel
    HAL_ENABLE_INTERRUPTS();
    sys_timer_init();
    Led_Power(1);
    Led_Post(1);

    initIOB();
    // Lan_Initialize1();
    //    Lan_Initialize2();

    user_ram_start = workspace_start;
    user_ram_end = workspace_end;

    do_version(0,0);

    cyg_pci_init();

    diag_printf("welcome, snowel 888\r\n");


    //    pci_scan();
    
    //    Lan_Initialize3(myaddr);
    //    diag_printf("*IXP425_INTR_EN=%08x\n",*IXP425_INTR_EN);
    //    diag_printf("*IXP425_INTR_SEL=%08x\n",*IXP425_INTR_SEL);
    timeOld = sys_times();
    
#if 1
    if(PushButton())
      {
	diag_printf("have push button\r\n");
	Lan_Initialize3(myaddr);
	Download();
      }
#endif
    {
    // Give the guy a chance to abort any boot script
        int script_timeout_ms = CYGNUM_REDBOOT_BOOT_SCRIPT_DEFAULT_TIMEOUT * CYGNUM_REDBOOT_BOOT_SCRIPT_TIMEOUT_RESOLUTION;
        int res;
        static char line[CYGPKG_REDBOOT_MAX_CMD_LINE];

        diag_printf("== Executing boot script in %d.%03d seconds - enter ^C to abort\n", 
                    script_timeout_ms/1000, script_timeout_ms%1000);
        res = _GETS_CTRLC;  // Treat 0 timeout as ^C
        while (script_timeout_ms >= CYGNUM_REDBOOT_CLI_IDLE_TIMEOUT) {
            res = _rb_gets(line, sizeof(line), CYGNUM_REDBOOT_CLI_IDLE_TIMEOUT);
            if (res >= _GETS_OK) {
                diag_printf("== Executing boot script in %d.%03d seconds - enter ^C to abort\n", 
                            script_timeout_ms/1000, script_timeout_ms%1000);
                continue;  // Ignore anything but ^C
            }
            if (res != _GETS_TIMEOUT) break;
            script_timeout_ms -= CYGNUM_REDBOOT_CLI_IDLE_TIMEOUT;
        }
        if (res == _GETS_CTRLC) {
            use_cmd = 1;
        } else {
            use_cmd = 0;
        }
    }
    
    if(!use_cmd){
        do_boot();
    }

    Lan_Initialize3(myaddr);
    while(1)
      {
	IOB *psIOB;
	//diag_printf("step 1\n");
	if(kbd_proc())
	  cmd_proc();
#if 0
	if((psIOB=Lan_Receive1())!=NULL)
	  {
	    unsigned short dwID = *(unsigned short *)(psIOB->dbData +12);
	    if(dwID == 0x8888)
	      Download();

	    //diag_printf("receive a LAN packet \n");
	     Lan_Transmit3(psIOB);
	  }
	if((psIOB=Lan_Receive2())!=NULL)
	  {
	    diag_printf("receive a WAN packet \n");
	    freeIOB(psIOB);
	  }

#endif
	if((psIOB=Lan_Receive3())!=NULL)
	  {
#if 0
	    int i;
	    unsigned char *ptr=psIOB->dbData + psIOB->dwOffset;

	    diag_printf("psIOB->dwLength=%d,psIOB->dwOffset=%d\n",psIOB->dwLength,psIOB->dwOffset);
	    for(i=0;i<14;i++)
	      diag_printf("%02x.",*ptr++);

	    diag_printf("\nreceive a packet from GIGA \n");
#endif
	    freeIOB(psIOB);
	  }
#if 1
	timeNow = sys_times();
	if(timeNow-timeOld == 5)
	  {
	    Led_Power(0);
	    Led_Post(0);
	  }
	else
	  if(timeNow-timeOld >= 10)
	  {
	    Led_Power(1);
	    Led_Post(1);
	    timeOld = timeNow;
	  }
#endif

	if(PushButton())
	  Download();
	//diag_printf("step 4\n");
      }
}
