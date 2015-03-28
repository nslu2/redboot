//==========================================================================
//
//      net/enet.c
//
//      Stand-alone ethernet [link-layer] support for RedBoot
//
//==========================================================================
//####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 1998, 1999, 2000, 2001, 2002 Red Hat, Inc.
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
#include <net/net.h>
#include <cyg/io/eth/eth_drv.h>       // Logical driver interfaces
#include <cyg/io/eth/netdev.h>

#include <iob.h>
extern CBQ Lan_CBQ[];

//#define ENET_STATS 1

#ifdef ENET_STATS
static int num_received = 0;
static int num_transmitted = 0;
#endif

//snowel
extern cyg_netdevtab_entry_t npe_netdev0;
extern cyg_netdevtab_entry_t npe_netdev1;
char __local_enet_addr1[ETHER_ADDR_LEN+2];
char __local_enet_addr2[ETHER_ADDR_LEN+2];

void
Lan_Initialize1(void)
{
    cyg_netdevtab_entry_t *t;
    unsigned index;
#if defined(CYGHWR_NET_DRIVERS) && (CYGHWR_NET_DRIVERS > 1)
    char *default_devname;
    int default_index;
#endif


    //snowel
    diag_printf("net_init\r\n");
    // Set defaults as appropriate
#ifdef CYGDBG_REDBOOT_NET_DEBUG
    net_debug = true;
#else
    net_debug = false;
#endif

    // Make sure the recv buffers are set up
    //    eth_drv_buffers_init();
    //    __pktbuf_init();
    Lan_CBQ[0].psQHead=NULL;
    Lan_CBQ[0].psQTail=NULL;
    Lan_CBQ[0].dwQCount=0;
    
    // Initialize network device.
    // Try default device first, then others

    //snowel
    t = &npe_netdev0;
    t->init(t);
    t->status = CYG_NETDEVTAB_STATUS_AVAIL;

    memcpy(__local_enet_addr1,&((struct eth_drv_sc *)t->device_instance)->sc_arpcom.esa,ETHER_ADDR_LEN);
    diag_printf("Ethernet %s: MAC address %02x:%02x:%02x:%02x:%02x:%02x\n",
		t->name,
		__local_enet_addr1[0],
		__local_enet_addr1[1],
		__local_enet_addr1[2],
		__local_enet_addr1[3],
		__local_enet_addr1[4],
		__local_enet_addr1[5]);

}

void Lan_GetAddr1(char *addr)
{
  memcpy(addr,__local_enet_addr1,6);
}

void
Lan_Initialize2(void)
{
    cyg_netdevtab_entry_t *t = &npe_netdev1;

    Lan_CBQ[1].psQHead=NULL;
    Lan_CBQ[1].psQTail=NULL;
    Lan_CBQ[1].dwQCount=0;

    t->init(t);
    t->status = CYG_NETDEVTAB_STATUS_AVAIL;
    memcpy(__local_enet_addr2,&((struct eth_drv_sc *)t->device_instance)->sc_arpcom.esa,ETHER_ADDR_LEN);
    diag_printf("Ethernet %s: MAC address %02x:%02x:%02x:%02x:%02x:%02x\n",
		t->name,
		__local_enet_addr2[0],
		__local_enet_addr2[1],
		__local_enet_addr2[2],
		__local_enet_addr2[3],
		__local_enet_addr2[4],
		__local_enet_addr2[5]);

}
/*
 * Non-blocking poll of ethernet link. Process packets until no more
 * are available.
 */
IOB *Lan_Receive1(void)
{
  struct eth_drv_sc *sc =(struct eth_drv_sc *)npe_netdev0.device_instance;

  (sc->funs->poll)(sc);
  return (IOB *)getCBFromQ(&Lan_CBQ[0],0);
}

/*
 * Send an ethernet packet.
 */
void
Lan_Transmit1(IOB *psIOB)
{
    struct eth_drv_sc *sc =(struct eth_drv_sc *)npe_netdev0.device_instance;

    eth_drv_write(sc, pbIOBData(psIOB), psIOB->dwLength);
    freeIOB(psIOB);
#ifdef ENET_STATS
    ++num_transmitted;
#endif
}

void Lan_Stop1(void)
{
    struct eth_drv_sc *sc =(struct eth_drv_sc *)npe_netdev0.device_instance;
    eth_drv_stop(sc);
}
/*
 * Non-blocking poll of ethernet link. Process packets until no more
 * are available.
 */
IOB *Lan_Receive2(void)
{
  struct eth_drv_sc *sc =(struct eth_drv_sc *)npe_netdev1.device_instance;

  (sc->funs->poll)(sc);
  return (IOB *)getCBFromQ(&Lan_CBQ[1],0);
}



/*
 * Send an ethernet packet.
 */
void
Lan_Transmit2(IOB *psIOB)
{
    struct eth_drv_sc *sc =(struct eth_drv_sc *)npe_netdev1.device_instance;

    eth_drv_write(sc, pbIOBData(psIOB), psIOB->dwLength);
    freeIOB(psIOB);
#ifdef ENET_STATS
    ++num_transmitted;
#endif
}

void Lan_Stop2(void)
{
    struct eth_drv_sc *sc =(struct eth_drv_sc *)npe_netdev1.device_instance;
    eth_drv_stop(sc);
}

#ifdef __LITTLE_ENDIAN__

unsigned long  
ntohl(unsigned long x)
{
    return (((x & 0x000000FF) << 24) |
            ((x & 0x0000FF00) <<  8) |
            ((x & 0x00FF0000) >>  8) |
            ((x & 0xFF000000) >> 24));
}

unsigned long
ntohs(unsigned short x)
{
    return (((x & 0x00FF) << 8) |
            ((x & 0xFF00) >> 8));
}

#endif
