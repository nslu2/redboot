//==========================================================================
//
//      net/arp.c
//
//      Stand-alone ARP support for RedBoot
//
//==========================================================================
//####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 1998, 1999, 2000, 2001, 2002 Red Hat, Inc.
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

static struct {
    int      waiting;
    char     *eth;
    char     *ip;
} arp_req;

int arp_send(IOB *psIOB,char *target_enet)
{
  IOB *newIOB;
  eth_header_t *eth_hdr;

  if(IOBOFF(psIOB)>ETH_HDR_SIZE)
    {
      IOBOFF(psIOB) -= ETH_HDR_SIZE;
    }
  else
    {
      if((newIOB = LAN_DriverGetIOB(ARP_PKT_SIZE + 4))==NULL)
	{
	  diag_printf("can't get IOB in arp_send\n");
	  return 1;
	}
      IOBOFF(newIOB) = 4; 
      
      memcpy(IOBDATA(newIOB)+ETH_HDR_SIZE,IOBDATA(psIOB),sizeof(arp_header_t));
      freeIOB(psIOB);
      psIOB = newIOB;
      
    }

  IOBLEN(psIOB) = ARP_PKT_SIZE;
  eth_hdr = IOBDATA(psIOB);
  memcpy(&eth_hdr->destination,target_enet,sizeof(enet_addr_t));
  net_GetAddr((char *)&eth_hdr->source);
  eth_hdr->type = ntohs(ETH_TYPE_ARP);

  return net_Transmit(psIOB);
}
/*
 * Handle incoming ARP packets.
 */
void
__arp_handler(IOB *psIOB)
{
    arp_header_t *arp = IOBDATA(psIOB);
    int hw_type, protocol;

    /*
     * Only handle ethernet hardware and IP protocol.
     */
    protocol = ntohs(arp->protocol);
    hw_type = ntohs(arp->hw_type);
    if ((hw_type == ARP_HW_ETHER) && (protocol == ETH_TYPE_IP)) {
	/*
	 * Handle requests for our ethernet address.
	 */
	if (!memcmp(arp->target_ip, __local_ip_addr, 4)) {
	    if (ntohs(arp->opcode) == ARP_REQUEST) {
		/* format response. */
		arp->opcode = htons(ARP_REPLY);
		memcpy(arp->target_ip, arp->sender_ip,
		       sizeof(ip_addr_t));
		memcpy(arp->target_enet, arp->sender_enet,
		       sizeof(enet_addr_t));
		memcpy(arp->sender_ip, __local_ip_addr,
		       sizeof(ip_addr_t));
		net_GetAddr((char *)arp->sender_enet);
		arp_send(psIOB,&arp->target_enet);

	    } else if (ntohs(arp->opcode) == ARP_REPLY && arp_req.waiting) {
		if (!memcmp(arp_req.ip, arp->sender_ip, sizeof(ip_addr_t))) {
                    memcpy(arp_req.eth, arp->sender_enet, sizeof(enet_addr_t));
		    arp_req.waiting = 0;
		}
	    }
	}
    }
    freeIOB(psIOB);
}


/* 
 * Find the ethernet address of the machine with the given
 * ip address.
 * Return 0 and fills in 'eth_addr' if successful,
 *       -1 if unsuccessful.
 */
int
__arp_request(ip_addr_t *ip_addr, enet_addr_t *eth_addr)
{
  //pktbuf_t *pkt;
  IOB *psIOB;
    arp_header_t *arp;
    unsigned long retry_start;
    enet_addr_t   bcast_addr;
    int           retry;

    // Special case request for self
    if (!memcmp(ip_addr, __local_ip_addr, 4)) {
      net_GetAddr((char *)eth_addr);
      return 0;
    }

    if((psIOB = LAN_DriverGetIOB(ARP_PKT_SIZE + 4))==NULL)
      {
	diag_printf("can't get IOB in arp_send\n");
	return -1;
      }
    IOBOFF(psIOB) = 4 + ETH_HDR_SIZE;

    arp = IOBDATA(psIOB);
    arp->opcode = htons(ARP_REQUEST);
    arp->hw_type = htons(ARP_HW_ETHER);
    arp->protocol = htons(0x800);
    arp->hw_len = sizeof(enet_addr_t);
    arp->proto_len = sizeof(ip_addr_t);

    memcpy(arp->sender_ip, __local_ip_addr, sizeof(ip_addr_t));
    net_GetAddr((char *)arp->sender_enet);
    memcpy(arp->target_ip, ip_addr, sizeof(ip_addr_t));

    bcast_addr[0] = 255;
    bcast_addr[1] = 255;
    bcast_addr[2] = 255;
    bcast_addr[3] = 255;
    bcast_addr[4] = 255;
    bcast_addr[5] = 255;

    arp_req.eth = (char *)eth_addr;
    arp_req.ip = (char *)ip_addr;
    arp_req.waiting = 1;

    retry = 8;
    while (retry-- > 0) {

       /* send the packet */
      arp_send(psIOB,&bcast_addr);

        retry_start = SYSTEMTIME_MS;
	while ((SYSTEMTIME_MS - retry_start) < 250) {
	    __enet_poll();
	    if (!arp_req.waiting) {
	      //__pktbuf_free(pkt);
	      freeIOB(psIOB);
		return 0;
	    }
	}
    }
    freeIOB(psIOB);
    return -1;
}

#define NUM_ARP 16
static ip_route_t routes[NUM_ARP];

int
__arp_lookup(ip_addr_t *host, ip_route_t *rt)
{
    int i;
    static int next_arp = 0;

    for (i = 0;  i < NUM_ARP;  i++) {
        if (memcmp(host, &routes[i].ip_addr, sizeof(*host)) == 0) {
            // This is a known host
            memcpy(rt, &routes[i], sizeof(*rt));
            return 0;
        }
    }
    memcpy(&rt->ip_addr, host, sizeof(*host));
    if (((*host)[0] == 0xFF) && ((*host)[1] == 0xFF) && ((*host)[2] == 0xFF)) {
        memset(&rt->enet_addr, 0xFF, sizeof(&rt->enet_addr));
        return 0;
#ifdef CYGSEM_REDBOOT_NETWORKING_USE_GATEWAY
    } else if (!__ip_addr_local(host)) {
        // non-local IP address -- look up Gateway's Ethernet address
        host = &__local_ip_gate;
#endif
    }
    if (__arp_request(host, &rt->enet_addr) < 0) {
        return -1;
    } else {
        memcpy(&routes[next_arp], rt, sizeof(*rt));
        if (++next_arp == NUM_ARP) next_arp = 0;
        return 0;
    }
}

