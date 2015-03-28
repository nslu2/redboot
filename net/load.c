//==========================================================================
//
//      load.c
//
//      RedBoot file/image loader
//
//==========================================================================
//####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 1998, 1999, 2000, 2001, 2002 Red Hat, Inc.
// Copyright (C) 2002, 2003 Gary Thomas
//
// eCos is free software; you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free
// Software Foundation; either version 2 or (at your option) any later version.
//
// eCos is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along
// with eCos; if not, write to the Free Software Foundation, Inc.,
// 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
//
// As a special exception, if other files instantiate templates or use macros
// or inline functions from this file, or you compile this file and link it
// with other works to produce a work based on this file, this file does not
// by itself cause the resulting work to be covered by the GNU General Public
// License. However the source code for this file must still be made available
// in accordance with section (3) of the GNU General Public License.
//
// This exception does not invalidate any other reasons why a work based on
// this file might be covered by the GNU General Public License.
//
// Alternative licenses for eCos may be arranged by contacting Red Hat, Inc.
// at http://sources.redhat.com/ecos/ecos-license/
// -------------------------------------------
//####ECOSGPLCOPYRIGHTEND####
//==========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):    gthomas
// Contributors: gthomas, tsmith
// Date:         2000-07-14
// Purpose:      
// Description:  
//              
// This code is part of RedBoot (tm).
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <redboot.h>
#include <elf.h>
#include <net/tftp_support.h>

static char usage[] = "[-r] [-v] "
                      "[-h <host>] [-m <varies>] "
                      "\n        [-b <base_address>] <file_name>";

// Exported CLI function
RedBoot_cmd("load", 
            "Load a file", 
            usage,
            do_load 
    );


extern getc_io_funcs_t tftp_io;

// Buffers, data used by redboot_getc
#define BUF_SIZE 256
struct {
    getc_io_funcs_t *io;
    int (*fun)(char *, int len, int *err);
    unsigned char  buf[BUF_SIZE];
    unsigned char *bufp;
    int   avail, len, err;
    int   verbose, decompress, tick;
} getc_info;

typedef int (*getc_t)(void);

//
// Read the next data byte from the stream.
// Returns:
//    >= 0 - actual data
//      -1 - error or EOF, status in getc_info.err
//
static int 
redboot_getc(void)
{
    static char spin[] = "|/-\\|-";
    if (getc_info.avail < 0) {
      return -1;
    }

    if (getc_info.avail == 0) {
        if (getc_info.verbose) {

    
           diag_printf("%c\b", spin[getc_info.tick++]);
            if (getc_info.tick >= sizeof(spin)) {
                getc_info.tick = 0;
            }
        }
        if (getc_info.len < BUF_SIZE) {
            // No more data available
            if (getc_info.verbose) diag_printf("\n");
            return -1;
        }
        getc_info.bufp = getc_info.buf;
        getc_info.len = (*getc_info.fun)(getc_info.bufp, BUF_SIZE, &getc_info.err);
        if ((getc_info.avail = getc_info.len) <= 0) {
            if (getc_info.len < 0) {
                diag_printf("I/O error: %s\n", (getc_info.io->error)(getc_info.err));
            }
            if (getc_info.verbose) diag_printf("\n");
            return -1;
        }
    }
    getc_info.avail--;
    return *getc_info.bufp++;
}


static int
redboot_getc_init(connection_info_t *info, getc_io_funcs_t *funcs, 
                  int verbose, int decompress)
{
    int res;

    res = (funcs->open)(info, &getc_info.err);    
    if (res < 0) {
        diag_printf("Can't load '%s': %s\n", info->filename, (funcs->error)(getc_info.err));
            return res;
    }
    getc_info.io = funcs;
    getc_info.fun = funcs->read;
    getc_info.avail = 0;
    getc_info.len = BUF_SIZE;
    getc_info.verbose = verbose;
    getc_info.decompress = decompress;
    getc_info.tick = 0;
    return 0;
}

static void
redboot_getc_rewind(void)
{
    getc_info.bufp = getc_info.buf;
    getc_info.avail = getc_info.len;
}

static void
redboot_getc_terminate(bool abort)
{
    if (getc_info.io->terminate) {
        (getc_info.io->terminate)(abort, redboot_getc);
    }
}

static void
redboot_getc_close(void)
{
    (getc_info.io->close)(&getc_info.err);
}




//
// 'load' CLI command processing
//   -b - specify a load [base] address
//   -m - specify an I/O stream/method
//   -c - Alternate serial I/O channel
//
void 
do_load(int argc, char *argv[])
{
    int res, num_options;
    int i, err;
    bool verbose = true,raw = true;
    bool base_addr_set;
    //char *mode_str;
    struct sockaddr_in host;
    //bool decompress = false;
    int chan = -1;
    unsigned long base = 0;
    unsigned long end = 0;
    //char type[4];
    char *filename = 0;
    struct option_info opts[6];
    connection_info_t info;
    getc_io_funcs_t *io = NULL;
    struct load_io_entry *io_tab;
#ifdef CYGSEM_REDBOOT_VALIDATE_USER_RAM_LOADS
    bool spillover_ok = false;
#endif

    CYGACC_CALL_IF_DELAY_US(100000);   // Wait for 100ms

    memset((char *)&host, 0, sizeof(host));
    host.sin_len = sizeof(host);
    host.sin_family = AF_INET;
    memcpy(&host.sin_addr, &__local_tftp_server_ip_addr, sizeof(host.sin_addr));
    host.sin_port = 0;

    //init_opts(&opts[0], 'v', false, OPTION_ARG_TYPE_FLG, 
    //          (void **)&verbose, 0, "verbose");
    //init_opts(&opts[1], 'r', false, OPTION_ARG_TYPE_FLG, 
    //          (void **)&raw, 0, "load raw data");
    init_opts(&opts[0], 'b', true, OPTION_ARG_TYPE_NUM, 
              (void **)&base, (bool *)&base_addr_set, "load address");
    num_options = 1;
    //num_options++;

    if (!scan_opts(argc, argv, 1, opts, num_options, 
                   (void *)&filename, OPTION_ARG_TYPE_STR, "file name")) {
        return;
    }


    if (chan >= CYGNUM_HAL_VIRTUAL_VECTOR_NUM_CHANNELS) {
        diag_printf("Invalid I/O channel: %d\n", chan);
        return;
    }

    {
        char *which;
        io_tab = (struct load_io_entry *)NULL;  // Default
        which = "TFTP";
        io = &tftp_io;
        diag_printf("Using default protocol (%s)\n", which);
	CYGACC_CALL_IF_DELAY_US(1000);   // Wait for 1ms
    }
#ifdef CYGSEM_REDBOOT_VALIDATE_USER_RAM_LOADS
    if (base_addr_set &&
        ((base < (unsigned long)user_ram_start) ||
         (base > (unsigned long)user_ram_end))) {
        if (!verify_action("Specified address (%p) is not believed to be in RAM", (void*)base))
            return;
        spillover_ok = true;
    }
#endif
    if (raw && !base_addr_set) {
        diag_printf("File load requires a memory address\n");
        return;
    }
    info.filename = filename;
    info.chan = chan;
    info.mode = io_tab ? io_tab->mode : 0;
    info.server = &host;
    res = redboot_getc_init(&info, io, verbose, false);
    if (res < 0) {
        return;
    }

    // Stream open, process the data
    if(raw)
    {
        unsigned char *mp = (unsigned char *)base;
        err = 0;
        while ((res = redboot_getc()) >= 0) {
#ifdef CYGSEM_REDBOOT_VALIDATE_USER_RAM_LOADS
            if (mp >= user_ram_end && !spillover_ok) {
                // Only if there is no need to stop the download
                // before printing output can we ask confirmation
                // questions.
                redboot_getc_terminate(true);
                diag_printf("*** Abort! RAW data spills over limit of user RAM at %p\n",(void*)mp);
                err = -1;
                break;
            }
#endif
            *mp++ = res;
        }
        end = (unsigned long) mp;

        // Save load base/top
        load_address = base;
        load_address_end = end;
        entry_address = base;           // best guess

        redboot_getc_terminate(false);
        if (0 == err)
            diag_printf("Raw file loaded %p-%p, assumed entry at %p\n", 
                        (void *)base, (void *)(end - 1), (void*)base);
	CYGACC_CALL_IF_DELAY_US(1000);   // Wait for 1ms
    }

    redboot_getc_close();  // Clean up
    CYGACC_CALL_IF_DELAY_US(100000);   // Wait for 100ms

    return;
}
