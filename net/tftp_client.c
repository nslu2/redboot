//==========================================================================
//
//      net/tftp_client.c
//
//      Stand-alone TFTP support for RedBoot
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

// TFTP client support

#include <redboot.h>     // have_net
#include <net/net.h>
#include <net/tftp.h>
#include <net/tftp_support.h>

// So we remember which ports have been used
static int get_port = 7700;

static struct {
    bool open;
    int  total_timeouts;
    unsigned short last_good_block;
    int  avail, actual_len;
    struct sockaddr_in local_addr, from_addr;
    char data[SEGSIZE+sizeof(struct tftphdr)];
    char *bufp;
} tftp_stream;

int
tftp_stream_open(connection_info_t *info,
                 int *err)
{
    struct tftphdr *hdr = (struct tftphdr *)tftp_stream.data;
    char *cp, *fp;
    char test_buf;

    if ( tftp_stream.open) {
        *err = TFTP_INVALID;  // Already open
        return -1;
    }

    // Create initial request
    hdr->th_opcode = htons(RRQ);  // Read file
    cp = (char *)&hdr->th_stuff;
    fp = info->filename;
    while (*fp) *cp++ = *fp++;
    *cp++ = '\0';
    // Since this is used for downloading data, OCTET (binary) is the
    // only mode that makes sense.
    fp = "OCTET";
    while (*fp) *cp++ = *fp++;
    *cp++ = '\0';

    memset((char *)&tftp_stream.local_addr, 0, sizeof(tftp_stream.local_addr));
    tftp_stream.local_addr.sin_family = AF_INET;
    tftp_stream.local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    tftp_stream.local_addr.sin_port = htons(get_port++);

    if (info->server->sin_port == 0) {
        info->server->sin_port = htons(TFTP_PORT);
    }

    // Send request - note: RFC 1350 (TFTP rev 2) indicates that this should be
    // only as long as required to hold the request, with the nul terminator.
    // Some servers silently go to lunch if the request is not the correct size.
    if (__udp_sendto(tftp_stream.data, cp-(char *)hdr, 
                     info->server, &tftp_stream.local_addr) < 0) {
        // Problem sending request
        *err = TFTP_NETERR;
        return -1;
    }

    tftp_stream.open = true;
    tftp_stream.avail = 0;
    tftp_stream.actual_len = -1;
    tftp_stream.last_good_block = 0;
    tftp_stream.total_timeouts = 0;
    tftp_stream.from_addr.sin_port = 0;

    // Try and read the first byte [block] since no errors are
    // reported until then.
    if (tftp_stream_read(&test_buf, 1, err) == 1) {
        // Back up [rewind] over this datum
        tftp_stream.bufp--;
        tftp_stream.avail++;
        return 0;  // Open and first read successful
    } else {
        tftp_stream.open = false;
        return -1; // Couldn't read
    }
}

void
tftp_stream_close(int *err)
{
    tftp_stream.open = false;
}

int
tftp_stream_read(char *buf,
                 int len,
                 int *err)
{
    int total_bytes = 0;
    int size, recv_len, data_len;
    struct timeval timeout;
    struct tftphdr *hdr = (struct tftphdr *)tftp_stream.data;

    while (total_bytes < len) {
        // Move any bytes which we've already read/buffered
        if (tftp_stream.avail > 0) {
            size = tftp_stream.avail;
            if (size > (len - total_bytes)) size = len - total_bytes;
            memcpy(buf, tftp_stream.bufp, size);
            buf += size;
            tftp_stream.bufp += size;
            tftp_stream.avail -= size;
            total_bytes += size;
        } else {
            if (tftp_stream.last_good_block != 0) {
                // Send out the ACK
                hdr->th_opcode = htons(ACK);
                hdr->th_block = htons(tftp_stream.last_good_block);
                if (__udp_sendto(tftp_stream.data, 4 /* FIXME */, 
                                 &tftp_stream.from_addr, &tftp_stream.local_addr) < 0) {
                    // Problem sending ACK
                    *err = TFTP_NETERR;
                    return -1;
                }
            }
            if ((tftp_stream.actual_len >= 0) && (tftp_stream.actual_len < SEGSIZE)) {
                // Out of data
                break;
            }
            timeout.tv_sec = TFTP_TIMEOUT_PERIOD;
            timeout.tv_usec = 0;
            recv_len = sizeof(tftp_stream.data);
            if ((data_len = __udp_recvfrom(&tftp_stream.data[0], recv_len, &tftp_stream.from_addr, 
                                           &tftp_stream.local_addr,  &timeout)) < 0) {
                // No data, try again
                if ((++tftp_stream.total_timeouts > TFTP_TIMEOUT_MAX) || 
                    (tftp_stream.last_good_block == 0)) {
                    // Timeout - no data received
                    *err = TFTP_TIMEOUT;
                    return -1;
                }
            } else {
                if (ntohs(hdr->th_opcode) == DATA) {
                    if (ntohs(hdr->th_block) == (tftp_stream.last_good_block+1)) {
                        // Consume this data
                        data_len -= 4;  /* Sizeof TFTP header */
                        tftp_stream.avail = tftp_stream.actual_len = data_len;
                        tftp_stream.bufp = hdr->th_data;
                        tftp_stream.last_good_block++;
                    }
                } else {
                    if (ntohs(hdr->th_opcode) == ERROR) {
                        *err = ntohs(hdr->th_code);
                        return -1;
                    } else {
                        // What kind of packet is this?
                        *err = TFTP_PROTOCOL;
                        return -1;
                    }
                }
            }
        }
    }
    return total_bytes;
}

char *
tftp_error(int err)
{
    char *errmsg = "Unknown error";

    switch (err) {
    case TFTP_ENOTFOUND:
        return "file not found";
    case TFTP_EACCESS:
        return "access violation";
    case TFTP_ENOSPACE:
        return "disk full or allocation exceeded";
    case TFTP_EBADOP:
        return "illegal TFTP operation";
    case TFTP_EBADID:
        return "unknown transfer ID";
    case TFTP_EEXISTS:
        return "file already exists";
    case TFTP_ENOUSER:
        return "no such user";
    case TFTP_TIMEOUT:
        return "operation timed out";
    case TFTP_NETERR:
        return "some sort of network error";
    case TFTP_INVALID:
        return "invalid parameter";
    case TFTP_PROTOCOL:
        return "protocol violation";
    case TFTP_TOOLARGE:
        return "file is larger than buffer";
    }
    return errmsg;
}

static void
show_addrs(void)
{
    diag_printf("IP: %s", inet_ntoa((in_addr_t *)&__local_ip_addr));
#ifdef CYGSEM_REDBOOT_NETWORKING_USE_GATEWAY
    diag_printf("/%s", inet_ntoa((in_addr_t *)&__local_ip_mask));
    diag_printf(", Gateway: %s\n", inet_ntoa((in_addr_t *)&__local_ip_gate));
#else
    diag_printf(", ");
#endif
    diag_printf("Default server: %s", inet_ntoa((in_addr_t *)&__local_tftp_server_ip_addr));
    diag_printf("\n");
}


void 
do_ip_addr(int argc, char *argv[])
{
    struct option_info opts[3];
    char *ip_addr, *host_addr;
    bool ip_addr_set, host_addr_set;
    struct sockaddr_in host;
    int num_opts;
        
    init_opts(&opts[0], 'l', true, OPTION_ARG_TYPE_STR, 
              (void **)&ip_addr, (bool *)&ip_addr_set, "local IP address");
    init_opts(&opts[1], 'h', true, OPTION_ARG_TYPE_STR, 
              (void **)&host_addr, (bool *)&host_addr_set, "default server address");
    num_opts = 2;

    if (!scan_opts(argc, argv, 1, opts, num_opts, 0, 0, "")) {
        return;
    }

    if (ip_addr_set) {
        if (!_gethostbyname(ip_addr, (in_addr_t *)&host)) {
            diag_printf("Invalid local IP address: %s\n", ip_addr);
            return;
        }
        // Of course, each address goes in its own place :-)
        memcpy(&__local_ip_addr, &host.sin_addr, sizeof(host.sin_addr));
    }
    if (host_addr_set) {
        if (!_gethostbyname(host_addr, (in_addr_t *)&host)) {
            diag_printf("Invalid server address: %s\n", host_addr);
            return;
        }
	memcpy(&__local_tftp_server_ip_addr,&host.sin_addr, sizeof(host.sin_addr));
        //my_bootp_info.bp_siaddr = host.sin_addr; //tftp server ip address
    }
    show_addrs();
    return;
}

//
// RedBoot interface
//
getc_io_funcs_t tftp_io = {  
  tftp_stream_open,
  tftp_stream_close,
  0,
  tftp_stream_read,
  tftp_error
};

//GETC_IO_FUNCS(tftp_io, tftp_stream_open, tftp_stream_close,
//              0, tftp_stream_read, tftp_error);
//RedBoot_load(tftp, tftp_io, true, true, 0);

