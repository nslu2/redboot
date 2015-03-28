/*==========================================================================
//
//      timer.c
//
//      Redboot syscall handling for GNUPro bsp support
//
//==========================================================================
//####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 1998, 1999, 2000, 2001, 2002, 2003 Red Hat, Inc.
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
*/
#include <redboot.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/drv_api.h>
#include <cyg/hal/hal_stub.h>



// Timer support

static cyg_handle_t  sys_timer_handle;
static cyg_interrupt sys_timer_interrupt;
static cyg_uint64    sys_timer_ticks = 0;


static unsigned int set_period = CYGNUM_HAL_RTC_PERIOD; // The default

typedef void *callback_func( char *pc, char *sp );
static callback_func *timer_callback = 0;


static void
sys_timer_dsr(cyg_vector_t vector, cyg_ucount32 count, cyg_addrword_t data)
{
    // do nothing
}


static cyg_uint32
sys_timer_isr(cyg_vector_t vector, cyg_addrword_t data)
{
  ++sys_timer_ticks;  //10ms,snowel

    HAL_CLOCK_RESET(CYGNUM_HAL_INTERRUPT_RTC, set_period);
    cyg_drv_interrupt_acknowledge(CYGNUM_HAL_INTERRUPT_RTC);

    cyg_drv_interrupt_mask(CYGNUM_HAL_INTERRUPT_RTC);
    if ( timer_callback ) {
        char *intrpc = (char *)0;
        char *intrsp = (char *)0;

        // There may be a number of ways to get the PC and (optional) SP
        // information out of the HAL.  Hence this is conditioned.  In some
        // configurations, a register-set pointer is available as
        // (invisible) argument 3 to this ISR call.

        CYGARC_HAL_SAVE_GP();
        timer_callback( intrpc, intrsp );
        CYGARC_HAL_RESTORE_GP();
    }
    cyg_drv_interrupt_unmask(CYGNUM_HAL_INTERRUPT_RTC);
    return CYG_ISR_HANDLED;
}


void sys_timer_init(void)
{
    HAL_CLOCK_INITIALIZE(set_period);
    
    cyg_drv_interrupt_create(
        CYGNUM_HAL_INTERRUPT_RTC,
        0,                      // Priority - unused
        (CYG_ADDRWORD)0,        // Data item passed to ISR & DSR
        sys_timer_isr,          // ISR
        sys_timer_dsr,          // DSR
        &sys_timer_handle,      // handle to intr obj
        &sys_timer_interrupt ); // space for int obj

    cyg_drv_interrupt_attach(sys_timer_handle);

    cyg_drv_interrupt_unmask(CYGNUM_HAL_INTERRUPT_RTC);


}



/* 100ms -- SYSTEMTIME */
unsigned long sys_times(void)
{
  return (sys_timer_ticks) / 10;
}

/* 1ms -- SYSTEMTIME_MS */
unsigned long sys_times_ms(void)
{
  return (sys_timer_ticks * 10);
}
