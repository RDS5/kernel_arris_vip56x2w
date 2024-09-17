/***************************************************************************
 *     Copyright (c) 1999-2012, Broadcom Corporation
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
 * Date:           Generated on         Wed Oct 17 03:11:32 2012
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

#ifndef BCHP_MOCA_HOSTMISC_H__
#define BCHP_MOCA_HOSTMISC_H__

/***************************************************************************
 *MOCA_HOSTMISC - MOCA_HOSTMISC registers
 ***************************************************************************/
#define BCHP_MOCA_HOSTMISC_MISC_CTRL             0x00fffd00 /* Moca Software Reset */
#define BCHP_MOCA_HOSTMISC_SCRATCH               0x00fffd04 /* Moca Scratch Register */
#define BCHP_MOCA_HOSTMISC_VERSION               0x00fffd08 /* MoCA version register */
#define BCHP_MOCA_HOSTMISC_H2M_INT_TRIG          0x00fffd0c /* Host-to-MoCA Interrupt Trigger */
#define BCHP_MOCA_HOSTMISC_WAKEUP                0x00fffd10 /* Host-to-MoCA Wakeup Interrupt */
#define BCHP_MOCA_HOSTMISC_SUBSYS_CFG            0x00fffd14 /* Moca Subsystem configuration */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_0      0x00fffd18 /* Host to MoCA MMP outbox registes , register set index 0. */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_1      0x00fffd1c /* Host to MoCA MMP outbox registes , register set index 1. */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_2      0x00fffd20 /* Host to MoCA MMP outbox registes , register set index 2. */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_3      0x00fffd24 /* Host to MoCA MMP outbox registes , register set index 3. */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_4      0x00fffd28 /* Host to MoCA MMP outbox registes , register set index 4. */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_5      0x00fffd2c /* Host to MoCA MMP outbox registes , register set index 5. */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_6      0x00fffd30 /* Host to MoCA MMP outbox registes , register set index 6. */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_7      0x00fffd34 /* Host to MoCA MMP outbox registes , register set index 7. */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_8      0x00fffd38 /* Host to MoCA MMP outbox registes , register set index 8. */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_9      0x00fffd3c /* Host to MoCA MMP outbox registes , register set index 9. */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_10     0x00fffd40 /* Host to MoCA MMP outbox registes , register set index 10. */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_11     0x00fffd44 /* Host to MoCA MMP outbox registes , register set index 11. */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_12     0x00fffd48 /* Host to MoCA MMP outbox registes , register set index 12. */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_13     0x00fffd4c /* Host to MoCA MMP outbox registes , register set index 13. */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_14     0x00fffd50 /* Host to MoCA MMP outbox registes , register set index 14. */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_15     0x00fffd54 /* Host to MoCA MMP outbox registes , register set index 15. */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_0       0x00fffd58 /* MoCA to Host MMP inbox registers , register set index 0. */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_1       0x00fffd5c /* MoCA to Host MMP inbox registers , register set index 1. */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_2       0x00fffd60 /* MoCA to Host MMP inbox registers , register set index 2. */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_3       0x00fffd64 /* MoCA to Host MMP inbox registers , register set index 3. */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_4       0x00fffd68 /* MoCA to Host MMP inbox registers , register set index 4. */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_5       0x00fffd6c /* MoCA to Host MMP inbox registers , register set index 5. */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_6       0x00fffd70 /* MoCA to Host MMP inbox registers , register set index 6. */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_7       0x00fffd74 /* MoCA to Host MMP inbox registers , register set index 7. */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_8       0x00fffd78 /* MoCA to Host MMP inbox registers , register set index 8. */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_9       0x00fffd7c /* MoCA to Host MMP inbox registers , register set index 9. */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_10      0x00fffd80 /* MoCA to Host MMP inbox registers , register set index 10. */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_11      0x00fffd84 /* MoCA to Host MMP inbox registers , register set index 11. */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_12      0x00fffd88 /* MoCA to Host MMP inbox registers , register set index 12. */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_13      0x00fffd8c /* MoCA to Host MMP inbox registers , register set index 13. */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_14      0x00fffd90 /* MoCA to Host MMP inbox registers , register set index 14. */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_15      0x00fffd94 /* MoCA to Host MMP inbox registers , register set index 15. */
#define BCHP_MOCA_HOSTMISC_DMPG_CHAINS_STATUS    0x00fffd98 /* "MoCA dynamic memory power gating chain power up (bit per chain),Active when moca_dmpg_gisb_en is high,0: Chain is on,1: Chain is off" */
#define BCHP_MOCA_HOSTMISC_DMPG_CHAINS_PWR_UP    0x00fffd9c /* "MoCA dynamic memory power gating chain status (bit per chain),0:  Power down chain,1:  Power up chain" */

/***************************************************************************
 *MISC_CTRL - Moca Software Reset
 ***************************************************************************/
/* MOCA_HOSTMISC :: MISC_CTRL :: spare_ctrl [31:17] */
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_spare_ctrl_MASK               0xfffe0000
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_spare_ctrl_SHIFT              17
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_spare_ctrl_DEFAULT            0x00007fff

/* MOCA_HOSTMISC :: MISC_CTRL :: moca_dmpg_sel [16:16] */
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_dmpg_sel_MASK            0x00010000
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_dmpg_sel_SHIFT           16
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_dmpg_sel_DEFAULT         0x00000000

/* MOCA_HOSTMISC :: MISC_CTRL :: moca_dmpg_en [15:15] */
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_dmpg_en_MASK             0x00008000
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_dmpg_en_SHIFT            15
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_dmpg_en_DEFAULT          0x00000001

/* MOCA_HOSTMISC :: MISC_CTRL :: spare_status [14:10] */
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_spare_status_MASK             0x00007c00
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_spare_status_SHIFT            10
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_spare_status_DEFAULT          0x0000001f

/* MOCA_HOSTMISC :: MISC_CTRL :: moca_phy1_disable_clk [09:09] */
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_phy1_disable_clk_MASK    0x00000200
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_phy1_disable_clk_SHIFT   9
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_phy1_disable_clk_DEFAULT 0x00000000

/* MOCA_HOSTMISC :: MISC_CTRL :: moca_phy0_disable_clk [08:08] */
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_phy0_disable_clk_MASK    0x00000100
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_phy0_disable_clk_SHIFT   8
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_phy0_disable_clk_DEFAULT 0x00000000

/* MOCA_HOSTMISC :: MISC_CTRL :: moca_disable_clocks [07:07] */
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_disable_clocks_MASK      0x00000080
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_disable_clocks_SHIFT     7
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_disable_clocks_DEFAULT   0x00000000

/* MOCA_HOSTMISC :: MISC_CTRL :: spare_reset [06:06] */
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_spare_reset_MASK              0x00000040
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_spare_reset_SHIFT             6
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_spare_reset_DEFAULT           0x00000001

/* MOCA_HOSTMISC :: MISC_CTRL :: moca_phy1_reset [05:05] */
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_phy1_reset_MASK          0x00000020
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_phy1_reset_SHIFT         5
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_phy1_reset_DEFAULT       0x00000000

/* MOCA_HOSTMISC :: MISC_CTRL :: moca_phy0_reset [04:04] */
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_phy0_reset_MASK          0x00000010
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_phy0_reset_SHIFT         4
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_phy0_reset_DEFAULT       0x00000000

/* MOCA_HOSTMISC :: MISC_CTRL :: moca_gmii_sw_init [03:03] */
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_gmii_sw_init_MASK        0x00000008
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_gmii_sw_init_SHIFT       3
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_gmii_sw_init_DEFAULT     0x00000000

/* MOCA_HOSTMISC :: MISC_CTRL :: moca_cpu_l_reset [02:02] */
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_cpu_l_reset_MASK         0x00000004
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_cpu_l_reset_SHIFT        2
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_cpu_l_reset_DEFAULT      0x00000001

/* MOCA_HOSTMISC :: MISC_CTRL :: moca_sys_reset [01:01] */
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_sys_reset_MASK           0x00000002
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_sys_reset_SHIFT          1
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_sys_reset_DEFAULT        0x00000001

/* MOCA_HOSTMISC :: MISC_CTRL :: moca_cpu_h_reset [00:00] */
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_cpu_h_reset_MASK         0x00000001
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_cpu_h_reset_SHIFT        0
#define BCHP_MOCA_HOSTMISC_MISC_CTRL_moca_cpu_h_reset_DEFAULT      0x00000001

/***************************************************************************
 *SCRATCH - Moca Scratch Register
 ***************************************************************************/
/* MOCA_HOSTMISC :: SCRATCH :: value [31:00] */
#define BCHP_MOCA_HOSTMISC_SCRATCH_value_MASK                      0xffffffff
#define BCHP_MOCA_HOSTMISC_SCRATCH_value_SHIFT                     0
#define BCHP_MOCA_HOSTMISC_SCRATCH_value_DEFAULT                   0x00000000

/***************************************************************************
 *VERSION - MoCA version register
 ***************************************************************************/
/* MOCA_HOSTMISC :: VERSION :: moca_id [31:16] */
#define BCHP_MOCA_HOSTMISC_VERSION_moca_id_MASK                    0xffff0000
#define BCHP_MOCA_HOSTMISC_VERSION_moca_id_SHIFT                   16
#define BCHP_MOCA_HOSTMISC_VERSION_moca_id_DEFAULT                 0x00006622

/* MOCA_HOSTMISC :: VERSION :: moca_spec_ver [15:12] */
#define BCHP_MOCA_HOSTMISC_VERSION_moca_spec_ver_MASK              0x0000f000
#define BCHP_MOCA_HOSTMISC_VERSION_moca_spec_ver_SHIFT             12
#define BCHP_MOCA_HOSTMISC_VERSION_moca_spec_ver_DEFAULT           0x00000003

/* MOCA_HOSTMISC :: VERSION :: core_version [11:08] */
#define BCHP_MOCA_HOSTMISC_VERSION_core_version_MASK               0x00000f00
#define BCHP_MOCA_HOSTMISC_VERSION_core_version_SHIFT              8
#define BCHP_MOCA_HOSTMISC_VERSION_core_version_DEFAULT            0x00000000

/* MOCA_HOSTMISC :: VERSION :: core_revision [07:04] */
#define BCHP_MOCA_HOSTMISC_VERSION_core_revision_MASK              0x000000f0
#define BCHP_MOCA_HOSTMISC_VERSION_core_revision_SHIFT             4
#define BCHP_MOCA_HOSTMISC_VERSION_core_revision_DEFAULT           0x00000000

/* MOCA_HOSTMISC :: VERSION :: core_mask [03:00] */
#define BCHP_MOCA_HOSTMISC_VERSION_core_mask_MASK                  0x0000000f
#define BCHP_MOCA_HOSTMISC_VERSION_core_mask_SHIFT                 0
#define BCHP_MOCA_HOSTMISC_VERSION_core_mask_DEFAULT               0x00000000

/***************************************************************************
 *H2M_INT_TRIG - Host-to-MoCA Interrupt Trigger
 ***************************************************************************/
/* MOCA_HOSTMISC :: H2M_INT_TRIG :: reserved0 [31:08] */
#define BCHP_MOCA_HOSTMISC_H2M_INT_TRIG_reserved0_MASK             0xffffff00
#define BCHP_MOCA_HOSTMISC_H2M_INT_TRIG_reserved0_SHIFT            8

/* MOCA_HOSTMISC :: H2M_INT_TRIG :: INT_TRIG [07:00] */
#define BCHP_MOCA_HOSTMISC_H2M_INT_TRIG_INT_TRIG_MASK              0x000000ff
#define BCHP_MOCA_HOSTMISC_H2M_INT_TRIG_INT_TRIG_SHIFT             0
#define BCHP_MOCA_HOSTMISC_H2M_INT_TRIG_INT_TRIG_DEFAULT           0x00000000

/***************************************************************************
 *WAKEUP - Host-to-MoCA Wakeup Interrupt
 ***************************************************************************/
/* MOCA_HOSTMISC :: WAKEUP :: reserved0 [31:02] */
#define BCHP_MOCA_HOSTMISC_WAKEUP_reserved0_MASK                   0xfffffffc
#define BCHP_MOCA_HOSTMISC_WAKEUP_reserved0_SHIFT                  2

/* MOCA_HOSTMISC :: WAKEUP :: cpu_l_wakeup_int [01:01] */
#define BCHP_MOCA_HOSTMISC_WAKEUP_cpu_l_wakeup_int_MASK            0x00000002
#define BCHP_MOCA_HOSTMISC_WAKEUP_cpu_l_wakeup_int_SHIFT           1
#define BCHP_MOCA_HOSTMISC_WAKEUP_cpu_l_wakeup_int_DEFAULT         0x00000000

/* MOCA_HOSTMISC :: WAKEUP :: cpu_h_wakeup_int [00:00] */
#define BCHP_MOCA_HOSTMISC_WAKEUP_cpu_h_wakeup_int_MASK            0x00000001
#define BCHP_MOCA_HOSTMISC_WAKEUP_cpu_h_wakeup_int_SHIFT           0
#define BCHP_MOCA_HOSTMISC_WAKEUP_cpu_h_wakeup_int_DEFAULT         0x00000000

/***************************************************************************
 *SUBSYS_CFG - Moca Subsystem configuration
 ***************************************************************************/
/* MOCA_HOSTMISC :: SUBSYS_CFG :: spare_cfg [31:01] */
#define BCHP_MOCA_HOSTMISC_SUBSYS_CFG_spare_cfg_MASK               0xfffffffe
#define BCHP_MOCA_HOSTMISC_SUBSYS_CFG_spare_cfg_SHIFT              1
#define BCHP_MOCA_HOSTMISC_SUBSYS_CFG_spare_cfg_DEFAULT            0x00000000

/* MOCA_HOSTMISC :: SUBSYS_CFG :: moca_arb_rr_sel [00:00] */
#define BCHP_MOCA_HOSTMISC_SUBSYS_CFG_moca_arb_rr_sel_MASK         0x00000001
#define BCHP_MOCA_HOSTMISC_SUBSYS_CFG_moca_arb_rr_sel_SHIFT        0
#define BCHP_MOCA_HOSTMISC_SUBSYS_CFG_moca_arb_rr_sel_DEFAULT      0x00000001

/***************************************************************************
 *H2M_MMP_OUTBOX_0 - Host to MoCA MMP outbox registes , register set index 0.
 ***************************************************************************/
/* MOCA_HOSTMISC :: H2M_MMP_OUTBOX_0 :: m2h_mmp_outbox [31:00] */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_0_m2h_mmp_outbox_MASK    0xffffffff
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_0_m2h_mmp_outbox_SHIFT   0
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_0_m2h_mmp_outbox_DEFAULT 0x00000000

/***************************************************************************
 *H2M_MMP_OUTBOX_1 - Host to MoCA MMP outbox registes , register set index 1.
 ***************************************************************************/
/* MOCA_HOSTMISC :: H2M_MMP_OUTBOX_1 :: m2h_mmp_outbox [31:00] */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_1_m2h_mmp_outbox_MASK    0xffffffff
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_1_m2h_mmp_outbox_SHIFT   0
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_1_m2h_mmp_outbox_DEFAULT 0x00000000

/***************************************************************************
 *H2M_MMP_OUTBOX_2 - Host to MoCA MMP outbox registes , register set index 2.
 ***************************************************************************/
/* MOCA_HOSTMISC :: H2M_MMP_OUTBOX_2 :: m2h_mmp_outbox [31:00] */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_2_m2h_mmp_outbox_MASK    0xffffffff
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_2_m2h_mmp_outbox_SHIFT   0
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_2_m2h_mmp_outbox_DEFAULT 0x00000000

/***************************************************************************
 *H2M_MMP_OUTBOX_3 - Host to MoCA MMP outbox registes , register set index 3.
 ***************************************************************************/
/* MOCA_HOSTMISC :: H2M_MMP_OUTBOX_3 :: m2h_mmp_outbox [31:00] */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_3_m2h_mmp_outbox_MASK    0xffffffff
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_3_m2h_mmp_outbox_SHIFT   0
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_3_m2h_mmp_outbox_DEFAULT 0x00000000

/***************************************************************************
 *H2M_MMP_OUTBOX_4 - Host to MoCA MMP outbox registes , register set index 4.
 ***************************************************************************/
/* MOCA_HOSTMISC :: H2M_MMP_OUTBOX_4 :: m2h_mmp_outbox [31:00] */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_4_m2h_mmp_outbox_MASK    0xffffffff
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_4_m2h_mmp_outbox_SHIFT   0
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_4_m2h_mmp_outbox_DEFAULT 0x00000000

/***************************************************************************
 *H2M_MMP_OUTBOX_5 - Host to MoCA MMP outbox registes , register set index 5.
 ***************************************************************************/
/* MOCA_HOSTMISC :: H2M_MMP_OUTBOX_5 :: m2h_mmp_outbox [31:00] */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_5_m2h_mmp_outbox_MASK    0xffffffff
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_5_m2h_mmp_outbox_SHIFT   0
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_5_m2h_mmp_outbox_DEFAULT 0x00000000

/***************************************************************************
 *H2M_MMP_OUTBOX_6 - Host to MoCA MMP outbox registes , register set index 6.
 ***************************************************************************/
/* MOCA_HOSTMISC :: H2M_MMP_OUTBOX_6 :: m2h_mmp_outbox [31:00] */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_6_m2h_mmp_outbox_MASK    0xffffffff
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_6_m2h_mmp_outbox_SHIFT   0
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_6_m2h_mmp_outbox_DEFAULT 0x00000000

/***************************************************************************
 *H2M_MMP_OUTBOX_7 - Host to MoCA MMP outbox registes , register set index 7.
 ***************************************************************************/
/* MOCA_HOSTMISC :: H2M_MMP_OUTBOX_7 :: m2h_mmp_outbox [31:00] */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_7_m2h_mmp_outbox_MASK    0xffffffff
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_7_m2h_mmp_outbox_SHIFT   0
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_7_m2h_mmp_outbox_DEFAULT 0x00000000

/***************************************************************************
 *H2M_MMP_OUTBOX_8 - Host to MoCA MMP outbox registes , register set index 8.
 ***************************************************************************/
/* MOCA_HOSTMISC :: H2M_MMP_OUTBOX_8 :: m2h_mmp_outbox [31:00] */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_8_m2h_mmp_outbox_MASK    0xffffffff
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_8_m2h_mmp_outbox_SHIFT   0
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_8_m2h_mmp_outbox_DEFAULT 0x00000000

/***************************************************************************
 *H2M_MMP_OUTBOX_9 - Host to MoCA MMP outbox registes , register set index 9.
 ***************************************************************************/
/* MOCA_HOSTMISC :: H2M_MMP_OUTBOX_9 :: m2h_mmp_outbox [31:00] */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_9_m2h_mmp_outbox_MASK    0xffffffff
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_9_m2h_mmp_outbox_SHIFT   0
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_9_m2h_mmp_outbox_DEFAULT 0x00000000

/***************************************************************************
 *H2M_MMP_OUTBOX_10 - Host to MoCA MMP outbox registes , register set index 10.
 ***************************************************************************/
/* MOCA_HOSTMISC :: H2M_MMP_OUTBOX_10 :: m2h_mmp_outbox [31:00] */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_10_m2h_mmp_outbox_MASK   0xffffffff
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_10_m2h_mmp_outbox_SHIFT  0
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_10_m2h_mmp_outbox_DEFAULT 0x00000000

/***************************************************************************
 *H2M_MMP_OUTBOX_11 - Host to MoCA MMP outbox registes , register set index 11.
 ***************************************************************************/
/* MOCA_HOSTMISC :: H2M_MMP_OUTBOX_11 :: m2h_mmp_outbox [31:00] */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_11_m2h_mmp_outbox_MASK   0xffffffff
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_11_m2h_mmp_outbox_SHIFT  0
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_11_m2h_mmp_outbox_DEFAULT 0x00000000

/***************************************************************************
 *H2M_MMP_OUTBOX_12 - Host to MoCA MMP outbox registes , register set index 12.
 ***************************************************************************/
/* MOCA_HOSTMISC :: H2M_MMP_OUTBOX_12 :: m2h_mmp_outbox [31:00] */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_12_m2h_mmp_outbox_MASK   0xffffffff
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_12_m2h_mmp_outbox_SHIFT  0
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_12_m2h_mmp_outbox_DEFAULT 0x00000000

/***************************************************************************
 *H2M_MMP_OUTBOX_13 - Host to MoCA MMP outbox registes , register set index 13.
 ***************************************************************************/
/* MOCA_HOSTMISC :: H2M_MMP_OUTBOX_13 :: m2h_mmp_outbox [31:00] */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_13_m2h_mmp_outbox_MASK   0xffffffff
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_13_m2h_mmp_outbox_SHIFT  0
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_13_m2h_mmp_outbox_DEFAULT 0x00000000

/***************************************************************************
 *H2M_MMP_OUTBOX_14 - Host to MoCA MMP outbox registes , register set index 14.
 ***************************************************************************/
/* MOCA_HOSTMISC :: H2M_MMP_OUTBOX_14 :: m2h_mmp_outbox [31:00] */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_14_m2h_mmp_outbox_MASK   0xffffffff
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_14_m2h_mmp_outbox_SHIFT  0
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_14_m2h_mmp_outbox_DEFAULT 0x00000000

/***************************************************************************
 *H2M_MMP_OUTBOX_15 - Host to MoCA MMP outbox registes , register set index 15.
 ***************************************************************************/
/* MOCA_HOSTMISC :: H2M_MMP_OUTBOX_15 :: m2h_mmp_outbox [31:00] */
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_15_m2h_mmp_outbox_MASK   0xffffffff
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_15_m2h_mmp_outbox_SHIFT  0
#define BCHP_MOCA_HOSTMISC_H2M_MMP_OUTBOX_15_m2h_mmp_outbox_DEFAULT 0x00000000

/***************************************************************************
 *M2H_MMP_INBOX_0 - MoCA to Host MMP inbox registers , register set index 0.
 ***************************************************************************/
/* MOCA_HOSTMISC :: M2H_MMP_INBOX_0 :: h2m_mmp_inbox [31:00] */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_0_h2m_mmp_inbox_MASK      0xffffffff
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_0_h2m_mmp_inbox_SHIFT     0
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_0_h2m_mmp_inbox_DEFAULT   0x00000000

/***************************************************************************
 *M2H_MMP_INBOX_1 - MoCA to Host MMP inbox registers , register set index 1.
 ***************************************************************************/
/* MOCA_HOSTMISC :: M2H_MMP_INBOX_1 :: h2m_mmp_inbox [31:00] */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_1_h2m_mmp_inbox_MASK      0xffffffff
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_1_h2m_mmp_inbox_SHIFT     0
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_1_h2m_mmp_inbox_DEFAULT   0x00000000

/***************************************************************************
 *M2H_MMP_INBOX_2 - MoCA to Host MMP inbox registers , register set index 2.
 ***************************************************************************/
/* MOCA_HOSTMISC :: M2H_MMP_INBOX_2 :: h2m_mmp_inbox [31:00] */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_2_h2m_mmp_inbox_MASK      0xffffffff
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_2_h2m_mmp_inbox_SHIFT     0
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_2_h2m_mmp_inbox_DEFAULT   0x00000000

/***************************************************************************
 *M2H_MMP_INBOX_3 - MoCA to Host MMP inbox registers , register set index 3.
 ***************************************************************************/
/* MOCA_HOSTMISC :: M2H_MMP_INBOX_3 :: h2m_mmp_inbox [31:00] */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_3_h2m_mmp_inbox_MASK      0xffffffff
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_3_h2m_mmp_inbox_SHIFT     0
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_3_h2m_mmp_inbox_DEFAULT   0x00000000

/***************************************************************************
 *M2H_MMP_INBOX_4 - MoCA to Host MMP inbox registers , register set index 4.
 ***************************************************************************/
/* MOCA_HOSTMISC :: M2H_MMP_INBOX_4 :: h2m_mmp_inbox [31:00] */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_4_h2m_mmp_inbox_MASK      0xffffffff
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_4_h2m_mmp_inbox_SHIFT     0
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_4_h2m_mmp_inbox_DEFAULT   0x00000000

/***************************************************************************
 *M2H_MMP_INBOX_5 - MoCA to Host MMP inbox registers , register set index 5.
 ***************************************************************************/
/* MOCA_HOSTMISC :: M2H_MMP_INBOX_5 :: h2m_mmp_inbox [31:00] */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_5_h2m_mmp_inbox_MASK      0xffffffff
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_5_h2m_mmp_inbox_SHIFT     0
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_5_h2m_mmp_inbox_DEFAULT   0x00000000

/***************************************************************************
 *M2H_MMP_INBOX_6 - MoCA to Host MMP inbox registers , register set index 6.
 ***************************************************************************/
/* MOCA_HOSTMISC :: M2H_MMP_INBOX_6 :: h2m_mmp_inbox [31:00] */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_6_h2m_mmp_inbox_MASK      0xffffffff
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_6_h2m_mmp_inbox_SHIFT     0
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_6_h2m_mmp_inbox_DEFAULT   0x00000000

/***************************************************************************
 *M2H_MMP_INBOX_7 - MoCA to Host MMP inbox registers , register set index 7.
 ***************************************************************************/
/* MOCA_HOSTMISC :: M2H_MMP_INBOX_7 :: h2m_mmp_inbox [31:00] */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_7_h2m_mmp_inbox_MASK      0xffffffff
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_7_h2m_mmp_inbox_SHIFT     0
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_7_h2m_mmp_inbox_DEFAULT   0x00000000

/***************************************************************************
 *M2H_MMP_INBOX_8 - MoCA to Host MMP inbox registers , register set index 8.
 ***************************************************************************/
/* MOCA_HOSTMISC :: M2H_MMP_INBOX_8 :: h2m_mmp_inbox [31:00] */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_8_h2m_mmp_inbox_MASK      0xffffffff
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_8_h2m_mmp_inbox_SHIFT     0
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_8_h2m_mmp_inbox_DEFAULT   0x00000000

/***************************************************************************
 *M2H_MMP_INBOX_9 - MoCA to Host MMP inbox registers , register set index 9.
 ***************************************************************************/
/* MOCA_HOSTMISC :: M2H_MMP_INBOX_9 :: h2m_mmp_inbox [31:00] */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_9_h2m_mmp_inbox_MASK      0xffffffff
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_9_h2m_mmp_inbox_SHIFT     0
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_9_h2m_mmp_inbox_DEFAULT   0x00000000

/***************************************************************************
 *M2H_MMP_INBOX_10 - MoCA to Host MMP inbox registers , register set index 10.
 ***************************************************************************/
/* MOCA_HOSTMISC :: M2H_MMP_INBOX_10 :: h2m_mmp_inbox [31:00] */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_10_h2m_mmp_inbox_MASK     0xffffffff
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_10_h2m_mmp_inbox_SHIFT    0
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_10_h2m_mmp_inbox_DEFAULT  0x00000000

/***************************************************************************
 *M2H_MMP_INBOX_11 - MoCA to Host MMP inbox registers , register set index 11.
 ***************************************************************************/
/* MOCA_HOSTMISC :: M2H_MMP_INBOX_11 :: h2m_mmp_inbox [31:00] */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_11_h2m_mmp_inbox_MASK     0xffffffff
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_11_h2m_mmp_inbox_SHIFT    0
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_11_h2m_mmp_inbox_DEFAULT  0x00000000

/***************************************************************************
 *M2H_MMP_INBOX_12 - MoCA to Host MMP inbox registers , register set index 12.
 ***************************************************************************/
/* MOCA_HOSTMISC :: M2H_MMP_INBOX_12 :: h2m_mmp_inbox [31:00] */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_12_h2m_mmp_inbox_MASK     0xffffffff
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_12_h2m_mmp_inbox_SHIFT    0
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_12_h2m_mmp_inbox_DEFAULT  0x00000000

/***************************************************************************
 *M2H_MMP_INBOX_13 - MoCA to Host MMP inbox registers , register set index 13.
 ***************************************************************************/
/* MOCA_HOSTMISC :: M2H_MMP_INBOX_13 :: h2m_mmp_inbox [31:00] */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_13_h2m_mmp_inbox_MASK     0xffffffff
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_13_h2m_mmp_inbox_SHIFT    0
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_13_h2m_mmp_inbox_DEFAULT  0x00000000

/***************************************************************************
 *M2H_MMP_INBOX_14 - MoCA to Host MMP inbox registers , register set index 14.
 ***************************************************************************/
/* MOCA_HOSTMISC :: M2H_MMP_INBOX_14 :: h2m_mmp_inbox [31:00] */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_14_h2m_mmp_inbox_MASK     0xffffffff
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_14_h2m_mmp_inbox_SHIFT    0
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_14_h2m_mmp_inbox_DEFAULT  0x00000000

/***************************************************************************
 *M2H_MMP_INBOX_15 - MoCA to Host MMP inbox registers , register set index 15.
 ***************************************************************************/
/* MOCA_HOSTMISC :: M2H_MMP_INBOX_15 :: h2m_mmp_inbox [31:00] */
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_15_h2m_mmp_inbox_MASK     0xffffffff
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_15_h2m_mmp_inbox_SHIFT    0
#define BCHP_MOCA_HOSTMISC_M2H_MMP_INBOX_15_h2m_mmp_inbox_DEFAULT  0x00000000

/***************************************************************************
 *DMPG_CHAINS_STATUS - "MoCA dynamic memory power gating chain power up (bit per chain),Active when moca_dmpg_gisb_en is high,0: Chain is on,1: Chain is off"
 ***************************************************************************/
/* MOCA_HOSTMISC :: DMPG_CHAINS_STATUS :: dmpg_pda_out_status [31:00] */
#define BCHP_MOCA_HOSTMISC_DMPG_CHAINS_STATUS_dmpg_pda_out_status_MASK 0xffffffff
#define BCHP_MOCA_HOSTMISC_DMPG_CHAINS_STATUS_dmpg_pda_out_status_SHIFT 0
#define BCHP_MOCA_HOSTMISC_DMPG_CHAINS_STATUS_dmpg_pda_out_status_DEFAULT 0x00000000

/***************************************************************************
 *DMPG_CHAINS_PWR_UP - "MoCA dynamic memory power gating chain status (bit per chain),0:  Power down chain,1:  Power up chain"
 ***************************************************************************/
/* MOCA_HOSTMISC :: DMPG_CHAINS_PWR_UP :: dmpg_pwr_up [31:00] */
#define BCHP_MOCA_HOSTMISC_DMPG_CHAINS_PWR_UP_dmpg_pwr_up_MASK     0xffffffff
#define BCHP_MOCA_HOSTMISC_DMPG_CHAINS_PWR_UP_dmpg_pwr_up_SHIFT    0
#define BCHP_MOCA_HOSTMISC_DMPG_CHAINS_PWR_UP_dmpg_pwr_up_DEFAULT  0xffffffff

#endif /* #ifndef BCHP_MOCA_HOSTMISC_H__ */

/* End of File */
