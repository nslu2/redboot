//==========================================================================
//
//      net/net_io.c
//
//      Stand-alone network logical I/O support for RedBoot
//
//==========================================================================
//####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 1998, 1999, 2000, 2001, 2002, 2003 Red Hat, Inc.
// Copyright (C) 2002 Gary Thomas
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
// Contributors: gthomas
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
#include <cyg/io/eth/eth_drv.h>            // Logical driver interfaces
#include <net/net.h>
#include <cyg/hal/hal_misc.h>   // Helper functions
#include <cyg/hal/hal_if.h>     // HAL I/O interfaces
#include <cyg/hal/drv_api.h>
#include <cyg/hal/hal_intr.h>


//
// Network initialization
//
#include <cyg/io/eth/eth_drv.h>
#include <cyg/io/eth/netdev.h>
#include <cyg/hal/hal_tables.h>

// Define table boundaries
CYG_HAL_TABLE_BEGIN( __NETDEVTAB__, netdev );
CYG_HAL_TABLE_END( __NETDEVTAB_END__, netdev );

//RedBoot_init(net_init, RedBoot_INIT_FIRST);



static cyg_netdevtab_entry_t *
net_devtab_entry(unsigned index)
{
    cyg_netdevtab_entry_t *t = &__NETDEVTAB__[index];

    if (t < &__NETDEVTAB__[0] || t >= &__NETDEVTAB_END__)
	return NULL;

    return t;
}

const char *
net_devname(unsigned index)
{
    cyg_netdevtab_entry_t *t = net_devtab_entry(index);
    if (t)
	return t->name;
    return NULL;
}

int
net_devindex(char *name)
{
    const char *devname;
    int index;

    for (index = 0; (devname = net_devname(index)) != NULL; index++)
	if (!strcmp(name, devname))
	    return index;
    return -1;
}

// TEMP

int 
start_console(void)
{
    int cur_console =
        CYGACC_CALL_IF_SET_CONSOLE_COMM(CYGNUM_CALL_IF_SET_COMM_ID_QUERY_CURRENT);

        CYGACC_CALL_IF_SET_CONSOLE_COMM(0);

    return cur_console;
}

void
end_console(int old_console)
{
    // Restore original console
    CYGACC_CALL_IF_SET_CONSOLE_COMM(old_console);
}
// TEMP


// EOF net_io.c
