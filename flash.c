//==========================================================================
//
//      flash_erase_block.c
//
//      Flash programming
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
// Author(s):    gthomas, hmt
// Contributors: gthomas
// Date:         2001-02-14
// Purpose:      
// Description:  
//              
//####DESCRIPTIONEND####
//
//==========================================================================

#include "strata.h"

#include <pkgconf/hal.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_cache.h>
#include <cyg/io/flash.h>
//
// CAUTION!  This code must be copied to RAM before execution.  Therefore,
// it must not contain any code which might be position dependent!
//

// Map a hardware status to a package error
int
flash_hwr_map_error(int err)
{
    if (err & FLASH_ErrorMask) {
        if (err & FLASH_ErrorProgram)
            return FLASH_ERR_PROGRAM;
        else if (err & FLASH_ErrorErase)
            return FLASH_ERR_ERASE;
        else 
            return FLASH_ERR_HWR;  // FIXME
    } else {
        return FLASH_ERR_OK;
    }
}

int flash_erase(volatile flash_t *block, unsigned int size)
{
    volatile flash_t *ROM;
    flash_t stat = 0;
    int timeout = 50000;
    int cache_on;
    int len, block_len, erase_block_size;
    volatile flash_t *eb;

    HAL_DCACHE_IS_ENABLED(cache_on);
    if (cache_on) {
        HAL_DCACHE_SYNC();
        HAL_DCACHE_DISABLE();
    }

    // Get base address and map addresses to virtual addresses
    ROM = FLASH_P2V(CYGNUM_FLASH_BASE_MASK & (unsigned int)block);
    eb = block = FLASH_P2V(block);
    block_len = size;

#define BLOCKSIZE (0x20000)
        erase_block_size = BLOCKSIZE;



    // Clear any error conditions
    ROM[0] = FLASH_Clear_Status;

    // Erase block
    while (block_len > 0) {
        ROM[0] = FLASH_Block_Erase;
        *eb = FLASH_Confirm;
        timeout = 5000000;
        while(((stat = ROM[0]) & FLASH_Status_Ready) != FLASH_Status_Ready) {
            if (--timeout == 0) break;
        }
        block_len -= erase_block_size;
        eb = FLASH_P2V((unsigned int)eb + erase_block_size);
    }

    // Restore ROM to "normal" mode
    ROM[0] = FLASH_Reset;

    // If an error was reported, see if the block erased anyway
    if (stat & FLASH_ErrorMask ) {
        len = size;
        while (len > 0) {
            if (*block++ != FLASH_BlankValue ) break;
            len -= sizeof(*block);
        }
        if (len == 0) stat = 0;
    }

    if (cache_on) {
        HAL_DCACHE_ENABLE();
    }

    stat = flash_hwr_map_error(stat);
    return stat;
}

#define BLOCK_MASK 0xfffe0000 
#define BUFFER_SIZE 0x20
int
flash_program_buf(volatile flash_t *addr, flash_t *data, int len)
{
    volatile flash_t *ROM;
    volatile flash_t *BA;
    flash_t stat = 0;
    int timeout = 50000;
    int cache_on;
    int i, wc;

    HAL_DCACHE_IS_ENABLED(cache_on);
    if (cache_on) {
        HAL_DCACHE_SYNC();
        HAL_DCACHE_DISABLE();
    }

    // Get base address and map addresses to virtual addresses
    ROM = FLASH_P2V( CYGNUM_FLASH_BASE_MASK & (unsigned int)addr );
    BA = FLASH_P2V( BLOCK_MASK & (unsigned int)addr );
    addr = FLASH_P2V(addr);

    // Clear any error conditions
    ROM[0] = FLASH_Clear_Status;

    // Write any big chunks first
    while (len >= BUFFER_SIZE) {
        wc = BUFFER_SIZE;
        if (wc > len) wc = len;
        len -= wc;
	// convert 'wc' in bytes to 'wc' in 'flash_t' 
        wc = wc / sizeof(flash_t);  // Word count
        *BA = FLASH_Write_Buffer;
        timeout = 5000000;
        while(((stat = ROM[0]) & FLASH_Status_Ready) != FLASH_Status_Ready) {
            if (--timeout == 0) {
                goto bad;
            }
            *BA = FLASH_Write_Buffer;
        }
        *BA = FLASHWORD(wc-1);  // Count is 0..N-1
        for (i = 0;  i < wc;  i++) {
            *(addr+i) = *(data+i);
        }
        *BA = FLASH_Confirm;
    
        ROM[0] = FLASH_Read_Status;
        timeout = 5000000;
        while(((stat = ROM[0]) & FLASH_Status_Ready) != FLASH_Status_Ready) {
            if (--timeout == 0) {
                goto bad;
            }
        }
        // Jump out if there was an error
        if (stat & FLASH_ErrorMask) {
            goto bad;
        }
        // And verify the data - also increments the pointers.
        *BA = FLASH_Reset;            
        for (i = 0;  i < wc;  i++) {
            if ( *addr++ != *data++ ) {
                stat = FLASH_ErrorNotVerified;
                goto bad;
            }
        }
    }

    while (len > 0) {
        ROM[0] = FLASH_Program;
        *addr = *data;
        timeout = 5000000;
        while(((stat = ROM[0]) & FLASH_Status_Ready) != FLASH_Status_Ready) {
            if (--timeout == 0) {
                goto bad;
            }
        }
        if (stat & FLASH_ErrorMask) {
            break;
        }
        ROM[0] = FLASH_Reset;            
        if (*addr++ != *data++) {
            stat = FLASH_ErrorNotVerified;
            break;
        }
        len -= sizeof( flash_t );
    }

    // Restore ROM to "normal" mode
 bad:
    ROM[0] = FLASH_Reset;            

    if (cache_on) {
        HAL_DCACHE_ENABLE();
    }

    stat = flash_hwr_map_error(stat);
    return stat;
}

void byteswap_buffer(char *mem,unsigned long length)
{
  char ch;
  unsigned long i;

  for(i=0;i<length;i+=2)
    {
      ch = mem[i];
      mem[i] = mem[i+1];
      mem[i+1] = ch;
    }
  
}

void do_flash_copy(int argc, char *argv[])
{
    unsigned long mem_addr, flash_addr,length;
    unsigned long offset;
    int stat;

    if(argc != 3 && argc !=4)
      {
      diag_printf("*** Error: invalid argument number \n");
      diag_printf("fcopy <source of memory> <dest in flash> <length> [-s]\n");
      diag_printf("      -s swap\n");
      diag_printf("ex. fcopy 0x01000000 0x51000000 0x100000\n");
      return -1;
    }


    if(!parse_num(argv[1], &mem_addr, 0, 0)){
        diag_printf("*** Error: invalid address '%s' for source address in memory \n", argv[1]);
        return  -1;
    }

    if(!parse_num(argv[2], &flash_addr, 0, 0)){
        diag_printf("*** Error: invalid address '%s' for kernel \n", argv[2]);
        return  -1;
    }
    
    if(!parse_num(argv[3], &length, 0, 0)){
        diag_printf("*** Error: invalid length '%s'  \n", argv[3]);
        return  -1;
    }
    
    if((argc == 4) &&  argv[4][0] == '-' && argv[4][1] == 's')
      byteswap_buffer(mem_addr,length);

    length = ( (length + BLOCKSIZE -1)  & BLOCK_MASK );

    diag_printf("Erase the flash");
    for( offset = 0;offset < length;offset += BLOCKSIZE)
      {
	if( (offset & 0xfffff) == 0)
	  diag_printf(".");
	if ((stat = flash_erase((void *)(flash_addr + offset), BLOCKSIZE)) != 0) 
	  {
	    diag_printf("Error erasing at %d\n", stat);
	  }
      }
    diag_printf("\n");

    for( offset = 0;offset < length;offset += BLOCKSIZE)
      {

	if ((stat = flash_program_buf((void *)(flash_addr + offset),(void *)(mem_addr + offset), BLOCKSIZE)) != 0)
	  {
	    diag_printf("Can't program region at flash_addr %08x,stat=%d\n", flash_addr+offset,stat);
	  }
	else
	  diag_printf("Have programed at 0x%08x\n",flash_addr + offset);
      }
	
    diag_printf("Verify ");
    for(offset = 0;offset <length; offset +=4)
      {
	if(*(unsigned long *)(mem_addr + offset) != 
	   *(unsigned long *)(flash_addr + offset))
	  {
	    diag_printf(" error at offset 0x%08x\n",offset);
	    break;
	  }
	if( (offset & 0xfffff) == 0)
	  diag_printf(".");
      }
    if(offset >= length)
      diag_printf("OK\n");

    
}
