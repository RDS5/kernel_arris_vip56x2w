/***************************************************************************
 *     Copyright (c) 1999-2013, Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Module Description:
 *                     DO NOT EDIT THIS FILE DIRECTLY
 *
 * This module was generated magically with RDB from a source description
 * file. You must edit the source file for changes to be made to this file.
 *
 *
 * Date:           Generated on              Fri Oct 25 17:33:28 2013
 *                 Full Compile MD5 Checksum 8344a9003ae3c954c1e4a33b9128d4d6
 *                   (minus title and desc)  
 *                 MD5 Checksum              383dbfcd91460427fa61afa66d6c98f8
 *
 * Compiled with:  RDB Utility               combo_header.pl
 *                 RDB Parser                3.0
 *                 unknown                   unknown
 *                 Perl Interpreter          5.008005
 *                 Operating System          linux
 *
 * Revision History:
 *
 * $brcm_Log: $
 *
 ***************************************************************************/

#ifndef BCHP_PCIE_0_EXT_CFG_H__
#define BCHP_PCIE_0_EXT_CFG_H__

/***************************************************************************
 *PCIE_0_EXT_CFG - PCIE EXTERNAL CFG Registers
 ***************************************************************************/
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_DATA_0  0x21088000 /* PCIe External Configuration Space Data[0] */
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_DATA_1023 0x21088ffc /* PCIe External Configuration Space Data[1023] */
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX   0x21089000 /* PCIE External Configuration Access Index */
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_DATA    0x21089004 /* PCIE External Configuration Access Data */
#define BCHP_PCIE_0_EXT_CFG_SCRATCH              0x21089008 /* Scratch Register */

/***************************************************************************
 *PCIE_EXT_CFG_DATA_0 - PCIe External Configuration Space Data[0]
 ***************************************************************************/
/* PCIE_0_EXT_CFG :: PCIE_EXT_CFG_DATA_0 :: DATA [31:00] */
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_DATA_0_DATA_MASK          0xffffffff
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_DATA_0_DATA_SHIFT         0

/***************************************************************************
 *PCIE_EXT_CFG_DATA_1023 - PCIe External Configuration Space Data[1023]
 ***************************************************************************/
/* PCIE_0_EXT_CFG :: PCIE_EXT_CFG_DATA_1023 :: DATA [31:00] */
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_DATA_1023_DATA_MASK       0xffffffff
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_DATA_1023_DATA_SHIFT      0

/***************************************************************************
 *PCIE_EXT_CFG_INDEX - PCIE External Configuration Access Index
 ***************************************************************************/
/* PCIE_0_EXT_CFG :: PCIE_EXT_CFG_INDEX :: UNUSED_1 [31:28] */
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX_UNUSED_1_MASK       0xf0000000
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX_UNUSED_1_SHIFT      28
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX_UNUSED_1_DEFAULT    0x00000000

/* PCIE_0_EXT_CFG :: PCIE_EXT_CFG_INDEX :: BUS_NUM [27:20] */
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX_BUS_NUM_MASK        0x0ff00000
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX_BUS_NUM_SHIFT       20
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX_BUS_NUM_DEFAULT     0x00000000

/* PCIE_0_EXT_CFG :: PCIE_EXT_CFG_INDEX :: DEV_NUM [19:15] */
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX_DEV_NUM_MASK        0x000f8000
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX_DEV_NUM_SHIFT       15
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX_DEV_NUM_DEFAULT     0x00000000

/* PCIE_0_EXT_CFG :: PCIE_EXT_CFG_INDEX :: FUNC_NUM [14:12] */
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX_FUNC_NUM_MASK       0x00007000
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX_FUNC_NUM_SHIFT      12
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX_FUNC_NUM_DEFAULT    0x00000000

/* PCIE_0_EXT_CFG :: PCIE_EXT_CFG_INDEX :: REG_NUM [11:02] */
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX_REG_NUM_MASK        0x00000ffc
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX_REG_NUM_SHIFT       2
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX_REG_NUM_DEFAULT     0x00000000

/* PCIE_0_EXT_CFG :: PCIE_EXT_CFG_INDEX :: UNUSED_0 [01:00] */
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX_UNUSED_0_MASK       0x00000003
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX_UNUSED_0_SHIFT      0
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_INDEX_UNUSED_0_DEFAULT    0x00000000

/***************************************************************************
 *PCIE_EXT_CFG_DATA - PCIE External Configuration Access Data
 ***************************************************************************/
/* PCIE_0_EXT_CFG :: PCIE_EXT_CFG_DATA :: DATA [31:00] */
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_DATA_DATA_MASK            0xffffffff
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_DATA_DATA_SHIFT           0
#define BCHP_PCIE_0_EXT_CFG_PCIE_EXT_CFG_DATA_DATA_DEFAULT         0x00000000

/***************************************************************************
 *SCRATCH - Scratch Register
 ***************************************************************************/
/* PCIE_0_EXT_CFG :: SCRATCH :: DATA [31:00] */
#define BCHP_PCIE_0_EXT_CFG_SCRATCH_DATA_MASK                      0xffffffff
#define BCHP_PCIE_0_EXT_CFG_SCRATCH_DATA_SHIFT                     0
#define BCHP_PCIE_0_EXT_CFG_SCRATCH_DATA_DEFAULT                   0x00000000

#endif /* #ifndef BCHP_PCIE_0_EXT_CFG_H__ */

/* End of File */
