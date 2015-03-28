/******************************************************************************
 *
 * Name:	sktypes.h
 * Project:	GEnesis, PCI Gigabit Ethernet Adapter
 * Version:	$Revision: 1.1.1.1 $
 * Date:	$Date: 2004/04/27 05:16:17 $
 * Purpose:	Define data types for Linux
 *
 ******************************************************************************/

/******************************************************************************
 *
 *	(C)Copyright 1998-2002 SysKonnect GmbH.
 *	(C)Copyright 2002-2003 Marvell.
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 *	The information in this file is provided "AS IS" without warranty.
 *
 ******************************************************************************/
 
/******************************************************************************
 *
 * Description:
 *
 * In this file, all data types that are needed by the common modules
 * are mapped to Linux data types.
 * 
 *
 * Include File Hierarchy:
 *
 *
 ******************************************************************************/

#ifndef __INC_SKTYPES_H
#define __INC_SKTYPES_H

#include <cyg/infra/cyg_type.h>
/* defines *******************************************************************/

/*
 * Data types with a specific size. 'I' = signed, 'U' = unsigned.
 */
#define SK_I8	cyg_int8
#define SK_U8	cyg_uint8
#define SK_I16	cyg_int16
#define SK_U16	cyg_uint16
#define SK_I32	cyg_int32
#define SK_U32	cyg_uint32
#define SK_I64	cyg_int64
#define SK_U64	cyg_uint64

#define SK_UPTR	cyg_uint32		/* casting pointer <-> integral */

/*
* Boolean type.
*/
#define SK_BOOL		SK_U8
#define SK_FALSE	0
#define SK_TRUE		(!SK_FALSE)

/* typedefs *******************************************************************/

/* function prototypes ********************************************************/

#endif	/* __INC_SKTYPES_H */
