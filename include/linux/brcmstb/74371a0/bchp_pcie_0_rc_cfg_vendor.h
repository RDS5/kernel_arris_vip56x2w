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
 * Date:           Generated on         Tue Apr 16 03:26:50 2013
 *                 MD5 Checksum         d41d8cd98f00b204e9800998ecf8427e
 *
 * Compiled with:  RDB Utility          combo_header.pl
 *                 RDB Parser           3.0
 *                 unknown              unknown
 *                 Perl Interpreter     5.008008
 *                 Operating System     linux
 *
 * Revision History:
 *
 * $brcm_Log: $
 *
 ***************************************************************************/

#ifndef BCHP_PCIE_0_RC_CFG_VENDOR_H__
#define BCHP_PCIE_0_RC_CFG_VENDOR_H__

/***************************************************************************
 *PCIE_0_RC_CFG_VENDOR
 ***************************************************************************/
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_CAP     0x00470180 /* vendor_cap */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_HEADER 0x00470184 /* vendor_specific_header */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1 0x00470188 /* Vendor Specific User Register 1 */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG2 0x0047018c /* Vendor Specific User Register 2 */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG3 0x00470190 /* Vendor Specific User Register 3 */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG4 0x00470194 /* Vendor Specific User Register 4 */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG5 0x00470198 /* Vendor Specific User Register 5 */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG6 0x0047019c /* Vendor Specific User Register 6 */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG7 0x004701a0 /* Vendor Specific User Register 7 */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG8 0x004701a4 /* Vendor Specific User Register 8 */

/***************************************************************************
 *VENDOR_CAP - vendor_cap
 ***************************************************************************/
/* PCIE_0_RC_CFG_VENDOR :: VENDOR_CAP :: NEXT [31:20] */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_CAP_NEXT_MASK             0xfff00000
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_CAP_NEXT_SHIFT            20
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_CAP_NEXT_DEFAULT          0x00000000

/* PCIE_0_RC_CFG_VENDOR :: VENDOR_CAP :: CAP_VER [19:16] */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_CAP_CAP_VER_MASK          0x000f0000
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_CAP_CAP_VER_SHIFT         16
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_CAP_CAP_VER_DEFAULT       0x00000001

/* PCIE_0_RC_CFG_VENDOR :: VENDOR_CAP :: VENDOR_SPEC_CAP_ID [15:00] */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_CAP_VENDOR_SPEC_CAP_ID_MASK 0x0000ffff
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_CAP_VENDOR_SPEC_CAP_ID_SHIFT 0
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_CAP_VENDOR_SPEC_CAP_ID_DEFAULT 0x0000000b

/***************************************************************************
 *VENDOR_SPECIFIC_HEADER - vendor_specific_header
 ***************************************************************************/
/* PCIE_0_RC_CFG_VENDOR :: VENDOR_SPECIFIC_HEADER :: VSEC_LENGTH [31:20] */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_HEADER_VSEC_LENGTH_MASK 0xfff00000
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_HEADER_VSEC_LENGTH_SHIFT 20
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_HEADER_VSEC_LENGTH_DEFAULT 0x00000028

/* PCIE_0_RC_CFG_VENDOR :: VENDOR_SPECIFIC_HEADER :: VSEC_REV [19:16] */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_HEADER_VSEC_REV_MASK 0x000f0000
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_HEADER_VSEC_REV_SHIFT 16
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_HEADER_VSEC_REV_DEFAULT 0x00000000

/* PCIE_0_RC_CFG_VENDOR :: VENDOR_SPECIFIC_HEADER :: VSEC_ID [15:00] */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_HEADER_VSEC_ID_MASK 0x0000ffff
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_HEADER_VSEC_ID_SHIFT 0
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_HEADER_VSEC_ID_DEFAULT 0x00000000

/***************************************************************************
 *VENDOR_SPECIFIC_REG1 - Vendor Specific User Register 1
 ***************************************************************************/
/* PCIE_0_RC_CFG_VENDOR :: VENDOR_SPECIFIC_REG1 :: Undefined [31:06] */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_Undefined_MASK 0xffffffc0
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_Undefined_SHIFT 6
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_Undefined_DEFAULT 0x00000000

/* PCIE_0_RC_CFG_VENDOR :: VENDOR_SPECIFIC_REG1 :: ENDIAN_MODE_BAR3 [05:04] */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR3_MASK 0x00000030
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR3_SHIFT 4
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR3_DEFAULT 0x00000000

/* PCIE_0_RC_CFG_VENDOR :: VENDOR_SPECIFIC_REG1 :: ENDIAN_MODE_BAR2 [03:02] */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR2_MASK 0x0000000c
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR2_SHIFT 2
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR2_DEFAULT 0x00000000

/* PCIE_0_RC_CFG_VENDOR :: VENDOR_SPECIFIC_REG1 :: ENDIAN_MODE_BAR1 [01:00] */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR1_MASK 0x00000003
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR1_SHIFT 0
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1_ENDIAN_MODE_BAR1_DEFAULT 0x00000000

/***************************************************************************
 *VENDOR_SPECIFIC_REG2 - Vendor Specific User Register 2
 ***************************************************************************/
/* PCIE_0_RC_CFG_VENDOR :: VENDOR_SPECIFIC_REG2 :: Undefined [31:00] */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG2_Undefined_MASK 0xffffffff
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG2_Undefined_SHIFT 0
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG2_Undefined_DEFAULT 0x00000000

/***************************************************************************
 *VENDOR_SPECIFIC_REG3 - Vendor Specific User Register 3
 ***************************************************************************/
/* PCIE_0_RC_CFG_VENDOR :: VENDOR_SPECIFIC_REG3 :: Undefined [31:00] */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG3_Undefined_MASK 0xffffffff
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG3_Undefined_SHIFT 0
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG3_Undefined_DEFAULT 0x00000000

/***************************************************************************
 *VENDOR_SPECIFIC_REG4 - Vendor Specific User Register 4
 ***************************************************************************/
/* PCIE_0_RC_CFG_VENDOR :: VENDOR_SPECIFIC_REG4 :: Undefined [31:00] */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG4_Undefined_MASK 0xffffffff
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG4_Undefined_SHIFT 0
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG4_Undefined_DEFAULT 0x00000000

/***************************************************************************
 *VENDOR_SPECIFIC_REG5 - Vendor Specific User Register 5
 ***************************************************************************/
/* PCIE_0_RC_CFG_VENDOR :: VENDOR_SPECIFIC_REG5 :: Undefined [31:00] */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG5_Undefined_MASK 0xffffffff
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG5_Undefined_SHIFT 0
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG5_Undefined_DEFAULT 0x00000000

/***************************************************************************
 *VENDOR_SPECIFIC_REG6 - Vendor Specific User Register 6
 ***************************************************************************/
/* PCIE_0_RC_CFG_VENDOR :: VENDOR_SPECIFIC_REG6 :: Undefined [31:00] */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG6_Undefined_MASK 0xffffffff
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG6_Undefined_SHIFT 0
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG6_Undefined_DEFAULT 0x00000000

/***************************************************************************
 *VENDOR_SPECIFIC_REG7 - Vendor Specific User Register 7
 ***************************************************************************/
/* PCIE_0_RC_CFG_VENDOR :: VENDOR_SPECIFIC_REG7 :: Undefined [31:00] */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG7_Undefined_MASK 0xffffffff
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG7_Undefined_SHIFT 0
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG7_Undefined_DEFAULT 0x00000000

/***************************************************************************
 *VENDOR_SPECIFIC_REG8 - Vendor Specific User Register 8
 ***************************************************************************/
/* PCIE_0_RC_CFG_VENDOR :: VENDOR_SPECIFIC_REG8 :: Undefined [31:00] */
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG8_Undefined_MASK 0xffffffff
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG8_Undefined_SHIFT 0
#define BCHP_PCIE_0_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG8_Undefined_DEFAULT 0x00000000

#endif /* #ifndef BCHP_PCIE_0_RC_CFG_VENDOR_H__ */

/* End of File */
