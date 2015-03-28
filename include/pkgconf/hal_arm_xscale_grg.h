#ifndef CYGONCE_PKGCONF_HAL_ARM_XSCALE_GRG_H
#define CYGONCE_PKGCONF_HAL_ARM_XSCALE_GRG_H
/*
 * File <pkgconf/hal_arm_xscale_grg.h>
 *
 * This file is generated automatically by the configuration
 * system. It should not be edited. Any changes to this file
 * may be overwritten.
 */

#define CYGBLD_HAL_PLF_INTS_H <cyg/hal/hal_plf_ints.h>
#define HAL_PLATFORM_CPU    "XScale"
#define HAL_PLATFORM_BOARD  "Intel Generic Residential Gateway"
#define HAL_PLATFORM_EXTRA  ""
//grg
//#define HAL_PLATFORM_MACHINE_TYPE  290
//ixdp425
#define HAL_PLATFORM_MACHINE_TYPE  245
#define CYGNUM_HAL_BREAKPOINT_LIST_SIZE 32
#define CYGNUM_HAL_BREAKPOINT_LIST_SIZE_32
#define CYGSEM_HAL_IXP425_PLF_USES_UART1 1
#define CYGSEM_HAL_IXP425_PLF_USES_UART2 1
#define CYGSEM_HAL_ROM_MONITOR 1

#endif
