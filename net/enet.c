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

#include <net/net.h>


#ifndef CYGDAT_REDBOOT_DEFAULT_ENET_ADDR
# define CYGDAT_REDBOOT_DEFAULT_ENET_ADDR   0x00, 0xC0, 0x02, 0x01, 0x01, 0x01
#endif

//
// Support for user handlers of additional ethernet packets (nonIP)
//

#define NUM_EXTRA_HANDLERS 4
static struct {
    int type;
    pkt_handler_t handler;
} eth_handlers[NUM_EXTRA_HANDLERS];

pkt_handler_t 
__eth_install_listener(int eth_type, pkt_handler_t handler)
{
    int i, empty;
    pkt_handler_t old;

    if (eth_type > 0x800 || handler != (pkt_handler_t)0) {
	empty = -1;
	for (i = 0;  i < NUM_EXTRA_HANDLERS;  i++) {
	    if (eth_handlers[i].type == eth_type) {
		// Replace existing handler
		old = eth_handlers[i].handler;
		eth_handlers[i].handler = handler;
		return old;
	    }
	    if (eth_handlers[i].type == 0) {
		empty = i;
	    }
	}
	if (empty >= 0) {
	    // Found a free slot
	    eth_handlers[empty].type = eth_type;
	    eth_handlers[empty].handler = handler;
	    return (pkt_handler_t)0;
	}
    }
    diag_printf("** Warning: can't install listener for ethernet type 0x%02x\n", eth_type);
    return (pkt_handler_t)0;
}

void 
__eth_remove_listener(int eth_type)
{
    int i;
    
    for (i = 0;  i < NUM_EXTRA_HANDLERS;  i++) {
        if (eth_handlers[i].type == eth_type) {
            eth_handlers[i].type = 0;
        }
    }    
}

int net_Initialize(unsigned char *node)
{
  //  return Lan_Initialize1();
  return Lan_Initialize3(node);
}

void net_GetAddr(unsigned char *node)
{
  //Lan_GetAddr1(node);
  Lan_GetAddr3(node);
}
IOB *net_Receive(void)
{
  //  return Lan_Receive1();
  return Lan_Receive3();
}
int net_Transmit(IOB *psIOB)
{
  //  Lan_Transmit1(psIOB);
  Lan_Transmit3(psIOB);
}

/*
 * Non-blocking poll of ethernet link. Process packets until no more
 * are available.
 */
void
__enet_poll(void)
{
  IOB *psIOB;
    eth_header_t *eth_hdr;
    int i, type;


    while (true) {

      if ((psIOB=net_Receive()) != 0) {
	    eth_hdr = IOBDATA(psIOB);
	    IOBOFF(psIOB) += ETH_HDR_SIZE;
	    IOBLEN(psIOB) -= ETH_HDR_SIZE;

            switch (type = ntohs(eth_hdr->type)) {

            case ETH_TYPE_IP:
                __ip_handler(psIOB, &eth_hdr->source);
                break;

            case ETH_TYPE_ARP:
                __arp_handler(psIOB);
                break;

            default:
	        if (type > 0x800 || type == 0x0015) {
		    for (i = 0;  i < NUM_EXTRA_HANDLERS;  i++) {
			if (eth_handlers[i].type == type) {
			    (eth_handlers[i].handler)(psIOB, eth_hdr);
			}
		    }
		}
		if(freeIOB(psIOB))
		  diag_printf("can't free IOB in __ip_handler");
                break;
            }
        } 
      else
	break;
    }
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
