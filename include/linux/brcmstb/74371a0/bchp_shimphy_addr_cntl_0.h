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

#ifndef BCHP_SHIMPHY_ADDR_CNTL_0_H__
#define BCHP_SHIMPHY_ADDR_CNTL_0_H__

/***************************************************************************
 *SHIMPHY_ADDR_CNTL_0 - 0 DDR SHIMPHY   Control Registers
 ***************************************************************************/
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG          0x00908000 /* SHIMPHY Config register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_SHIMPHY_REV_ID  0x00908004 /* SHIMPHY Revision ID Register. */
#define BCHP_SHIMPHY_ADDR_CNTL_0_RESET           0x00908008 /* DDR soft reset register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_CONTROL     0x00908038 /* DFI Interface Control Register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_STATUS      0x0090803c /* DFI Interface Status Register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_PHY_LPM_STAT    0x00908040 /* PHY Power Control Status Register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_IDLE_POWER_SAVING 0x00908048 /* DDR PHY Idle power saving Control register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_PHY_STANDBY_EXIT 0x0090804c /* DDR PHY standby exit register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_ANALOG_BYPASS_CNTRL 0x0090806c /* Analog macro register bypass control */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PLL_EXT_CLKSEL 0x00908070 /* DDR PLL external clock select register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_TEST_MODE_CNTRL_REG 0x00908074 /* SHIMPHY testport control register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DISABLE_CHIP_BYPASS_PLL 0x0090807c /* DDR bypass pll mode disable register. */
#define BCHP_SHIMPHY_ADDR_CNTL_0_VECTOR_MODE_CLK_SEL 0x00908088 /* DDR VECTOR PLL bypass mode clock select */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PAD_CNTRL   0x0090808c /* DDR Pad control register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_CLK_GATE        0x00908098 /* CLK_667_ENABLE Register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_SHIMPHY_STATUS  0x0090809c /* SHIMPHY Status Register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_CMD_DATA_FIFO   0x00908028 /* Command and Data FIFO Status Register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_RD_DATAPATH     0x0090802c /* Read Datapath Status Register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_FLAG_BUS        0x00908030 /* TP_OUT bus value Register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC            0x00908034 /* Miscellaneous Register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_SPARE0_RW       0x009080a4 /* Spare register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_SPARE1_RW       0x009080a8 /* Spare register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_SPARE0_RO       0x009080ac /* Spare register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_SPARE1_RO       0x009080b0 /* Spare register */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR3_RESET_CNTRL 0x009080b4 /* FORCE_DDR3_RESET Deassert  Register */

/***************************************************************************
 *CONFIG - SHIMPHY Config register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: CONFIG :: DFI_CLK_DISABLE [31:31] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_DFI_CLK_DISABLE_MASK       0x80000000
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_DFI_CLK_DISABLE_SHIFT      31
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_DFI_CLK_DISABLE_DEFAULT    0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: CONFIG :: DRAM_NOP_OR_DSEL_CMD [30:30] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_DRAM_NOP_OR_DSEL_CMD_MASK  0x40000000
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_DRAM_NOP_OR_DSEL_CMD_SHIFT 30
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_DRAM_NOP_OR_DSEL_CMD_DEFAULT 0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: CONFIG :: LAST_RD_STRETCH [29:29] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_LAST_RD_STRETCH_MASK       0x20000000
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_LAST_RD_STRETCH_SHIFT      29
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_LAST_RD_STRETCH_DEFAULT    0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: CONFIG :: reserved0 [28:24] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_reserved0_MASK             0x1f000000
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_reserved0_SHIFT            24

/* SHIMPHY_ADDR_CNTL_0 :: CONFIG :: LAST_READ_LATENCY [23:16] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_LAST_READ_LATENCY_MASK     0x00ff0000
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_LAST_READ_LATENCY_SHIFT    16
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_LAST_READ_LATENCY_DEFAULT  0x0000000b

/* SHIMPHY_ADDR_CNTL_0 :: CONFIG :: READ_LATENCY [15:08] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_READ_LATENCY_MASK          0x0000ff00
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_READ_LATENCY_SHIFT         8
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_READ_LATENCY_DEFAULT       0x00000007

/* SHIMPHY_ADDR_CNTL_0 :: CONFIG :: reserved1 [07:06] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_reserved1_MASK             0x000000c0
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_reserved1_SHIFT            6

/* SHIMPHY_ADDR_CNTL_0 :: CONFIG :: WRITE_LATENCY [05:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_WRITE_LATENCY_MASK         0x0000003f
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_WRITE_LATENCY_SHIFT        0
#define BCHP_SHIMPHY_ADDR_CNTL_0_CONFIG_WRITE_LATENCY_DEFAULT      0x0000000e

/***************************************************************************
 *SHIMPHY_REV_ID - SHIMPHY Revision ID Register.
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: SHIMPHY_REV_ID :: reserved0 [31:16] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_SHIMPHY_REV_ID_reserved0_MASK     0xffff0000
#define BCHP_SHIMPHY_ADDR_CNTL_0_SHIMPHY_REV_ID_reserved0_SHIFT    16

/* SHIMPHY_ADDR_CNTL_0 :: SHIMPHY_REV_ID :: MAJOR_ID [15:08] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_SHIMPHY_REV_ID_MAJOR_ID_MASK      0x0000ff00
#define BCHP_SHIMPHY_ADDR_CNTL_0_SHIMPHY_REV_ID_MAJOR_ID_SHIFT     8
#define BCHP_SHIMPHY_ADDR_CNTL_0_SHIMPHY_REV_ID_MAJOR_ID_DEFAULT   0x00000001

/* SHIMPHY_ADDR_CNTL_0 :: SHIMPHY_REV_ID :: MINOR_ID [07:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_SHIMPHY_REV_ID_MINOR_ID_MASK      0x000000ff
#define BCHP_SHIMPHY_ADDR_CNTL_0_SHIMPHY_REV_ID_MINOR_ID_SHIFT     0
#define BCHP_SHIMPHY_ADDR_CNTL_0_SHIMPHY_REV_ID_MINOR_ID_DEFAULT   0x00000000

/***************************************************************************
 *RESET - DDR soft reset register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: RESET :: reserved0 [31:03] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_RESET_reserved0_MASK              0xfffffff8
#define BCHP_SHIMPHY_ADDR_CNTL_0_RESET_reserved0_SHIFT             3

/* SHIMPHY_ADDR_CNTL_0 :: RESET :: DATAPATH_216_RESET [02:02] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_RESET_DATAPATH_216_RESET_MASK     0x00000004
#define BCHP_SHIMPHY_ADDR_CNTL_0_RESET_DATAPATH_216_RESET_SHIFT    2
#define BCHP_SHIMPHY_ADDR_CNTL_0_RESET_DATAPATH_216_RESET_DEFAULT  0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: RESET :: DATAPATH_DDR_RESET [01:01] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_RESET_DATAPATH_DDR_RESET_MASK     0x00000002
#define BCHP_SHIMPHY_ADDR_CNTL_0_RESET_DATAPATH_DDR_RESET_SHIFT    1
#define BCHP_SHIMPHY_ADDR_CNTL_0_RESET_DATAPATH_DDR_RESET_DEFAULT  0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: RESET :: PHY_PWRUP_RSB [00:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_RESET_PHY_PWRUP_RSB_MASK          0x00000001
#define BCHP_SHIMPHY_ADDR_CNTL_0_RESET_PHY_PWRUP_RSB_SHIFT         0
#define BCHP_SHIMPHY_ADDR_CNTL_0_RESET_PHY_PWRUP_RSB_DEFAULT       0x00000000

/***************************************************************************
 *DFI_CONTROL - DFI Interface Control Register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: DFI_CONTROL :: reserved0 [31:13] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_CONTROL_reserved0_MASK        0xffffe000
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_CONTROL_reserved0_SHIFT       13

/* SHIMPHY_ADDR_CNTL_0 :: DFI_CONTROL :: PHY_PLL_PWRDWN [12:12] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_CONTROL_PHY_PLL_PWRDWN_MASK   0x00001000
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_CONTROL_PHY_PLL_PWRDWN_SHIFT  12
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_CONTROL_PHY_PLL_PWRDWN_DEFAULT 0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: DFI_CONTROL :: reserved1 [11:10] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_CONTROL_reserved1_MASK        0x00000c00
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_CONTROL_reserved1_SHIFT       10

/* SHIMPHY_ADDR_CNTL_0 :: DFI_CONTROL :: TM2_MUX_SEL [09:09] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_CONTROL_TM2_MUX_SEL_MASK      0x00000200
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_CONTROL_TM2_MUX_SEL_SHIFT     9
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_CONTROL_TM2_MUX_SEL_DEFAULT   0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: DFI_CONTROL :: PHY_PLL_HOLD_CH [08:08] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_CONTROL_PHY_PLL_HOLD_CH_MASK  0x00000100
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_CONTROL_PHY_PLL_HOLD_CH_SHIFT 8
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_CONTROL_PHY_PLL_HOLD_CH_DEFAULT 0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: DFI_CONTROL :: reserved2 [07:01] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_CONTROL_reserved2_MASK        0x000000fe
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_CONTROL_reserved2_SHIFT       1

/* SHIMPHY_ADDR_CNTL_0 :: DFI_CONTROL :: LATCH_FIRST_ERROR [00:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_CONTROL_LATCH_FIRST_ERROR_MASK 0x00000001
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_CONTROL_LATCH_FIRST_ERROR_SHIFT 0
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_CONTROL_LATCH_FIRST_ERROR_DEFAULT 0x00000001

/***************************************************************************
 *DFI_STATUS - DFI Interface Status Register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: DFI_STATUS :: reserved0 [31:18] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_STATUS_reserved0_MASK         0xfffc0000
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_STATUS_reserved0_SHIFT        18

/* SHIMPHY_ADDR_CNTL_0 :: DFI_STATUS :: PHY_EDC_MONITOR_STATUS [17:17] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_STATUS_PHY_EDC_MONITOR_STATUS_MASK 0x00020000
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_STATUS_PHY_EDC_MONITOR_STATUS_SHIFT 17
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_STATUS_PHY_EDC_MONITOR_STATUS_DEFAULT 0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: DFI_STATUS :: PHY_VDL_MONITOR_STATUS [16:16] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_STATUS_PHY_VDL_MONITOR_STATUS_MASK 0x00010000
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_STATUS_PHY_VDL_MONITOR_STATUS_SHIFT 16
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_STATUS_PHY_VDL_MONITOR_STATUS_DEFAULT 0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: DFI_STATUS :: reserved1 [15:09] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_STATUS_reserved1_MASK         0x0000fe00
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_STATUS_reserved1_SHIFT        9

/* SHIMPHY_ADDR_CNTL_0 :: DFI_STATUS :: ERROR_VALID [08:08] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_STATUS_ERROR_VALID_MASK       0x00000100
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_STATUS_ERROR_VALID_SHIFT      8
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_STATUS_ERROR_VALID_DEFAULT    0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: DFI_STATUS :: reserved2 [07:04] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_STATUS_reserved2_MASK         0x000000f0
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_STATUS_reserved2_SHIFT        4

/* SHIMPHY_ADDR_CNTL_0 :: DFI_STATUS :: ERROR_INFO [03:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_STATUS_ERROR_INFO_MASK        0x0000000f
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_STATUS_ERROR_INFO_SHIFT       0
#define BCHP_SHIMPHY_ADDR_CNTL_0_DFI_STATUS_ERROR_INFO_DEFAULT     0x00000000

/***************************************************************************
 *PHY_LPM_STAT - PHY Power Control Status Register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: PHY_LPM_STAT :: reserved0 [31:10] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_PHY_LPM_STAT_reserved0_MASK       0xfffffc00
#define BCHP_SHIMPHY_ADDR_CNTL_0_PHY_LPM_STAT_reserved0_SHIFT      10

/* SHIMPHY_ADDR_CNTL_0 :: PHY_LPM_STAT :: PHY_RBUS_IS_IDLE [09:09] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_PHY_LPM_STAT_PHY_RBUS_IS_IDLE_MASK 0x00000200
#define BCHP_SHIMPHY_ADDR_CNTL_0_PHY_LPM_STAT_PHY_RBUS_IS_IDLE_SHIFT 9
#define BCHP_SHIMPHY_ADDR_CNTL_0_PHY_LPM_STAT_PHY_RBUS_IS_IDLE_DEFAULT 0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: PHY_LPM_STAT :: PHY_IS_IDLE [08:08] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_PHY_LPM_STAT_PHY_IS_IDLE_MASK     0x00000100
#define BCHP_SHIMPHY_ADDR_CNTL_0_PHY_LPM_STAT_PHY_IS_IDLE_SHIFT    8
#define BCHP_SHIMPHY_ADDR_CNTL_0_PHY_LPM_STAT_PHY_IS_IDLE_DEFAULT  0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: PHY_LPM_STAT :: reserved1 [07:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_PHY_LPM_STAT_reserved1_MASK       0x000000ff
#define BCHP_SHIMPHY_ADDR_CNTL_0_PHY_LPM_STAT_reserved1_SHIFT      0

/***************************************************************************
 *IDLE_POWER_SAVING - DDR PHY Idle power saving Control register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: IDLE_POWER_SAVING :: reserved0 [31:06] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_IDLE_POWER_SAVING_reserved0_MASK  0xffffffc0
#define BCHP_SHIMPHY_ADDR_CNTL_0_IDLE_POWER_SAVING_reserved0_SHIFT 6

/* SHIMPHY_ADDR_CNTL_0 :: IDLE_POWER_SAVING :: PhyAddrCntl [05:04] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_IDLE_POWER_SAVING_PhyAddrCntl_MASK 0x00000030
#define BCHP_SHIMPHY_ADDR_CNTL_0_IDLE_POWER_SAVING_PhyAddrCntl_SHIFT 4
#define BCHP_SHIMPHY_ADDR_CNTL_0_IDLE_POWER_SAVING_PhyAddrCntl_DEFAULT 0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: IDLE_POWER_SAVING :: ByteLane3 [03:03] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_IDLE_POWER_SAVING_ByteLane3_MASK  0x00000008
#define BCHP_SHIMPHY_ADDR_CNTL_0_IDLE_POWER_SAVING_ByteLane3_SHIFT 3
#define BCHP_SHIMPHY_ADDR_CNTL_0_IDLE_POWER_SAVING_ByteLane3_DEFAULT 0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: IDLE_POWER_SAVING :: ByteLane2 [02:02] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_IDLE_POWER_SAVING_ByteLane2_MASK  0x00000004
#define BCHP_SHIMPHY_ADDR_CNTL_0_IDLE_POWER_SAVING_ByteLane2_SHIFT 2
#define BCHP_SHIMPHY_ADDR_CNTL_0_IDLE_POWER_SAVING_ByteLane2_DEFAULT 0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: IDLE_POWER_SAVING :: ByteLane1 [01:01] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_IDLE_POWER_SAVING_ByteLane1_MASK  0x00000002
#define BCHP_SHIMPHY_ADDR_CNTL_0_IDLE_POWER_SAVING_ByteLane1_SHIFT 1
#define BCHP_SHIMPHY_ADDR_CNTL_0_IDLE_POWER_SAVING_ByteLane1_DEFAULT 0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: IDLE_POWER_SAVING :: ByteLane0 [00:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_IDLE_POWER_SAVING_ByteLane0_MASK  0x00000001
#define BCHP_SHIMPHY_ADDR_CNTL_0_IDLE_POWER_SAVING_ByteLane0_SHIFT 0
#define BCHP_SHIMPHY_ADDR_CNTL_0_IDLE_POWER_SAVING_ByteLane0_DEFAULT 0x00000000

/***************************************************************************
 *PHY_STANDBY_EXIT - DDR PHY standby exit register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: PHY_STANDBY_EXIT :: reserved0 [31:01] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_PHY_STANDBY_EXIT_reserved0_MASK   0xfffffffe
#define BCHP_SHIMPHY_ADDR_CNTL_0_PHY_STANDBY_EXIT_reserved0_SHIFT  1

/* SHIMPHY_ADDR_CNTL_0 :: PHY_STANDBY_EXIT :: ENABLE [00:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_PHY_STANDBY_EXIT_ENABLE_MASK      0x00000001
#define BCHP_SHIMPHY_ADDR_CNTL_0_PHY_STANDBY_EXIT_ENABLE_SHIFT     0
#define BCHP_SHIMPHY_ADDR_CNTL_0_PHY_STANDBY_EXIT_ENABLE_DEFAULT   0x00000001

/***************************************************************************
 *ANALOG_BYPASS_CNTRL - Analog macro register bypass control
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: ANALOG_BYPASS_CNTRL :: reserved0 [31:01] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_ANALOG_BYPASS_CNTRL_reserved0_MASK 0xfffffffe
#define BCHP_SHIMPHY_ADDR_CNTL_0_ANALOG_BYPASS_CNTRL_reserved0_SHIFT 1

/* SHIMPHY_ADDR_CNTL_0 :: ANALOG_BYPASS_CNTRL :: BYPASS_PLL [00:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_ANALOG_BYPASS_CNTRL_BYPASS_PLL_MASK 0x00000001
#define BCHP_SHIMPHY_ADDR_CNTL_0_ANALOG_BYPASS_CNTRL_BYPASS_PLL_SHIFT 0
#define BCHP_SHIMPHY_ADDR_CNTL_0_ANALOG_BYPASS_CNTRL_BYPASS_PLL_DEFAULT 0x00000000

/***************************************************************************
 *DDR_PLL_EXT_CLKSEL - DDR PLL external clock select register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: DDR_PLL_EXT_CLKSEL :: reserved0 [31:01] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PLL_EXT_CLKSEL_reserved0_MASK 0xfffffffe
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PLL_EXT_CLKSEL_reserved0_SHIFT 1

/* SHIMPHY_ADDR_CNTL_0 :: DDR_PLL_EXT_CLKSEL :: EXT_CLK_SEL [00:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PLL_EXT_CLKSEL_EXT_CLK_SEL_MASK 0x00000001
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PLL_EXT_CLKSEL_EXT_CLK_SEL_SHIFT 0
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PLL_EXT_CLKSEL_EXT_CLK_SEL_DEFAULT 0x00000000

/***************************************************************************
 *TEST_MODE_CNTRL_REG - SHIMPHY testport control register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: TEST_MODE_CNTRL_REG :: reserved_for_eco0 [31:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_TEST_MODE_CNTRL_REG_reserved_for_eco0_MASK 0xffffffff
#define BCHP_SHIMPHY_ADDR_CNTL_0_TEST_MODE_CNTRL_REG_reserved_for_eco0_SHIFT 0
#define BCHP_SHIMPHY_ADDR_CNTL_0_TEST_MODE_CNTRL_REG_reserved_for_eco0_DEFAULT 0x00000000

/***************************************************************************
 *DISABLE_CHIP_BYPASS_PLL - DDR bypass pll mode disable register.
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: DISABLE_CHIP_BYPASS_PLL :: reserved0 [31:01] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DISABLE_CHIP_BYPASS_PLL_reserved0_MASK 0xfffffffe
#define BCHP_SHIMPHY_ADDR_CNTL_0_DISABLE_CHIP_BYPASS_PLL_reserved0_SHIFT 1

/* SHIMPHY_ADDR_CNTL_0 :: DISABLE_CHIP_BYPASS_PLL :: DISABLE_BYPASS_PLL [00:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DISABLE_CHIP_BYPASS_PLL_DISABLE_BYPASS_PLL_MASK 0x00000001
#define BCHP_SHIMPHY_ADDR_CNTL_0_DISABLE_CHIP_BYPASS_PLL_DISABLE_BYPASS_PLL_SHIFT 0
#define BCHP_SHIMPHY_ADDR_CNTL_0_DISABLE_CHIP_BYPASS_PLL_DISABLE_BYPASS_PLL_DEFAULT 0x00000000

/***************************************************************************
 *VECTOR_MODE_CLK_SEL - DDR VECTOR PLL bypass mode clock select
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: VECTOR_MODE_CLK_SEL :: reserved0 [31:01] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_VECTOR_MODE_CLK_SEL_reserved0_MASK 0xfffffffe
#define BCHP_SHIMPHY_ADDR_CNTL_0_VECTOR_MODE_CLK_SEL_reserved0_SHIFT 1

/* SHIMPHY_ADDR_CNTL_0 :: VECTOR_MODE_CLK_SEL :: SEL [00:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_VECTOR_MODE_CLK_SEL_SEL_MASK      0x00000001
#define BCHP_SHIMPHY_ADDR_CNTL_0_VECTOR_MODE_CLK_SEL_SEL_SHIFT     0
#define BCHP_SHIMPHY_ADDR_CNTL_0_VECTOR_MODE_CLK_SEL_SEL_DEFAULT   0x00000000

/***************************************************************************
 *DDR_PAD_CNTRL - DDR Pad control register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: DDR_PAD_CNTRL :: reserved0 [31:07] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PAD_CNTRL_reserved0_MASK      0xffffff80
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PAD_CNTRL_reserved0_SHIFT     7

/* SHIMPHY_ADDR_CNTL_0 :: DDR_PAD_CNTRL :: IDDQ_MODE_ON_SELFREF [06:06] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PAD_CNTRL_IDDQ_MODE_ON_SELFREF_MASK 0x00000040
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PAD_CNTRL_IDDQ_MODE_ON_SELFREF_SHIFT 6
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PAD_CNTRL_IDDQ_MODE_ON_SELFREF_DEFAULT 0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: DDR_PAD_CNTRL :: PHY_IDLE_ENABLE [05:05] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PAD_CNTRL_PHY_IDLE_ENABLE_MASK 0x00000020
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PAD_CNTRL_PHY_IDLE_ENABLE_SHIFT 5
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PAD_CNTRL_PHY_IDLE_ENABLE_DEFAULT 0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: DDR_PAD_CNTRL :: HIZ_ON_SELFREF [04:04] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PAD_CNTRL_HIZ_ON_SELFREF_MASK 0x00000010
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PAD_CNTRL_HIZ_ON_SELFREF_SHIFT 4
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PAD_CNTRL_HIZ_ON_SELFREF_DEFAULT 0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: DDR_PAD_CNTRL :: CNTRL [03:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PAD_CNTRL_CNTRL_MASK          0x0000000f
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PAD_CNTRL_CNTRL_SHIFT         0
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR_PAD_CNTRL_CNTRL_DEFAULT       0x00000000

/***************************************************************************
 *CLK_GATE - CLK_667_ENABLE Register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: CLK_GATE :: UNUSED [31:01] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_CLK_GATE_UNUSED_MASK              0xfffffffe
#define BCHP_SHIMPHY_ADDR_CNTL_0_CLK_GATE_UNUSED_SHIFT             1
#define BCHP_SHIMPHY_ADDR_CNTL_0_CLK_GATE_UNUSED_DEFAULT           0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: CLK_GATE :: CLK_667_ENABLE [00:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_CLK_GATE_CLK_667_ENABLE_MASK      0x00000001
#define BCHP_SHIMPHY_ADDR_CNTL_0_CLK_GATE_CLK_667_ENABLE_SHIFT     0
#define BCHP_SHIMPHY_ADDR_CNTL_0_CLK_GATE_CLK_667_ENABLE_DEFAULT   0x00000000

/***************************************************************************
 *SHIMPHY_STATUS - SHIMPHY Status Register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: SHIMPHY_STATUS :: reserved0 [31:01] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_SHIMPHY_STATUS_reserved0_MASK     0xfffffffe
#define BCHP_SHIMPHY_ADDR_CNTL_0_SHIMPHY_STATUS_reserved0_SHIFT    1

/* SHIMPHY_ADDR_CNTL_0 :: SHIMPHY_STATUS :: READY [00:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_SHIMPHY_STATUS_READY_MASK         0x00000001
#define BCHP_SHIMPHY_ADDR_CNTL_0_SHIMPHY_STATUS_READY_SHIFT        0
#define BCHP_SHIMPHY_ADDR_CNTL_0_SHIMPHY_STATUS_READY_DEFAULT      0x00000000

/***************************************************************************
 *CMD_DATA_FIFO - Command and Data FIFO Status Register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: CMD_DATA_FIFO :: reserved0 [31:26] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_CMD_DATA_FIFO_reserved0_MASK      0xfc000000
#define BCHP_SHIMPHY_ADDR_CNTL_0_CMD_DATA_FIFO_reserved0_SHIFT     26

/* SHIMPHY_ADDR_CNTL_0 :: CMD_DATA_FIFO :: FIFO_FULL [25:25] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_CMD_DATA_FIFO_FIFO_FULL_MASK      0x02000000
#define BCHP_SHIMPHY_ADDR_CNTL_0_CMD_DATA_FIFO_FIFO_FULL_SHIFT     25

/* SHIMPHY_ADDR_CNTL_0 :: CMD_DATA_FIFO :: FIFO_EMPTY [24:24] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_CMD_DATA_FIFO_FIFO_EMPTY_MASK     0x01000000
#define BCHP_SHIMPHY_ADDR_CNTL_0_CMD_DATA_FIFO_FIFO_EMPTY_SHIFT    24

/* SHIMPHY_ADDR_CNTL_0 :: CMD_DATA_FIFO :: reserved1 [23:10] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_CMD_DATA_FIFO_reserved1_MASK      0x00fffc00
#define BCHP_SHIMPHY_ADDR_CNTL_0_CMD_DATA_FIFO_reserved1_SHIFT     10

/* SHIMPHY_ADDR_CNTL_0 :: CMD_DATA_FIFO :: WR_PNTR [09:05] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_CMD_DATA_FIFO_WR_PNTR_MASK        0x000003e0
#define BCHP_SHIMPHY_ADDR_CNTL_0_CMD_DATA_FIFO_WR_PNTR_SHIFT       5

/* SHIMPHY_ADDR_CNTL_0 :: CMD_DATA_FIFO :: RD_PNTR [04:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_CMD_DATA_FIFO_RD_PNTR_MASK        0x0000001f
#define BCHP_SHIMPHY_ADDR_CNTL_0_CMD_DATA_FIFO_RD_PNTR_SHIFT       0

/***************************************************************************
 *RD_DATAPATH - Read Datapath Status Register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: RD_DATAPATH :: reserved0 [31:10] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_RD_DATAPATH_reserved0_MASK        0xfffffc00
#define BCHP_SHIMPHY_ADDR_CNTL_0_RD_DATAPATH_reserved0_SHIFT       10

/* SHIMPHY_ADDR_CNTL_0 :: RD_DATAPATH :: WR_PNTR [09:05] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_RD_DATAPATH_WR_PNTR_MASK          0x000003e0
#define BCHP_SHIMPHY_ADDR_CNTL_0_RD_DATAPATH_WR_PNTR_SHIFT         5

/* SHIMPHY_ADDR_CNTL_0 :: RD_DATAPATH :: RD_PNTR [04:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_RD_DATAPATH_RD_PNTR_MASK          0x0000001f
#define BCHP_SHIMPHY_ADDR_CNTL_0_RD_DATAPATH_RD_PNTR_SHIFT         0

/***************************************************************************
 *FLAG_BUS - TP_OUT bus value Register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: FLAG_BUS :: FLAG_BUS [31:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_FLAG_BUS_FLAG_BUS_MASK            0xffffffff
#define BCHP_SHIMPHY_ADDR_CNTL_0_FLAG_BUS_FLAG_BUS_SHIFT           0

/***************************************************************************
 *MISC - Miscellaneous Register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: MISC :: reserved0 [31:20] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_reserved0_MASK               0xfff00000
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_reserved0_SHIFT              20

/* SHIMPHY_ADDR_CNTL_0 :: MISC :: ASYNC_FIFO_AF_THRESHOLD [19:15] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_ASYNC_FIFO_AF_THRESHOLD_MASK 0x000f8000
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_ASYNC_FIFO_AF_THRESHOLD_SHIFT 15
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_ASYNC_FIFO_AF_THRESHOLD_DEFAULT 0x0000000a

/* SHIMPHY_ADDR_CNTL_0 :: MISC :: reserved_for_eco1 [14:12] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_reserved_for_eco1_MASK       0x00007000
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_reserved_for_eco1_SHIFT      12
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_reserved_for_eco1_DEFAULT    0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: MISC :: DFI_ERROR_STATUS_CLR [11:11] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_DFI_ERROR_STATUS_CLR_MASK    0x00000800
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_DFI_ERROR_STATUS_CLR_SHIFT   11
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_DFI_ERROR_STATUS_CLR_DEFAULT 0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: MISC :: EDC_MONTIOR_STATUS_CLR [10:10] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_EDC_MONTIOR_STATUS_CLR_MASK  0x00000400
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_EDC_MONTIOR_STATUS_CLR_SHIFT 10
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_EDC_MONTIOR_STATUS_CLR_DEFAULT 0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: MISC :: VDL_MONITOR_STATUS_CLR [09:09] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_VDL_MONITOR_STATUS_CLR_MASK  0x00000200
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_VDL_MONITOR_STATUS_CLR_SHIFT 9
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_VDL_MONITOR_STATUS_CLR_DEFAULT 0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: MISC :: FUNC1 [08:08] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_FUNC1_MASK                   0x00000100
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_FUNC1_SHIFT                  8
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_FUNC1_DEFAULT                0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: MISC :: FUNC0 [07:07] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_FUNC0_MASK                   0x00000080
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_FUNC0_SHIFT                  7
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_FUNC0_DEFAULT                0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: MISC :: C2IO_INIT_RDY_OVR [06:06] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_C2IO_INIT_RDY_OVR_MASK       0x00000040
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_C2IO_INIT_RDY_OVR_SHIFT      6
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_C2IO_INIT_RDY_OVR_DEFAULT    0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: MISC :: RD_FIFO_HOLD_CLR [05:05] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_RD_FIFO_HOLD_CLR_MASK        0x00000020
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_RD_FIFO_HOLD_CLR_SHIFT       5
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_RD_FIFO_HOLD_CLR_DEFAULT     0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: MISC :: CMD_FIFO_HOLD_CLR [04:04] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_CMD_FIFO_HOLD_CLR_MASK       0x00000010
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_CMD_FIFO_HOLD_CLR_SHIFT      4
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_CMD_FIFO_HOLD_CLR_DEFAULT    0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: MISC :: reserved2 [03:01] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_reserved2_MASK               0x0000000e
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_reserved2_SHIFT              1

/* SHIMPHY_ADDR_CNTL_0 :: MISC :: DATA_OVERRUN_CLR [00:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_DATA_OVERRUN_CLR_MASK        0x00000001
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_DATA_OVERRUN_CLR_SHIFT       0
#define BCHP_SHIMPHY_ADDR_CNTL_0_MISC_DATA_OVERRUN_CLR_DEFAULT     0x00000000

/***************************************************************************
 *SPARE0_RW - Spare register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: SPARE0_RW :: reserved_for_eco0 [31:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_SPARE0_RW_reserved_for_eco0_MASK  0xffffffff
#define BCHP_SHIMPHY_ADDR_CNTL_0_SPARE0_RW_reserved_for_eco0_SHIFT 0
#define BCHP_SHIMPHY_ADDR_CNTL_0_SPARE0_RW_reserved_for_eco0_DEFAULT 0x00000000

/***************************************************************************
 *SPARE1_RW - Spare register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: SPARE1_RW :: reserved_for_eco0 [31:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_SPARE1_RW_reserved_for_eco0_MASK  0xffffffff
#define BCHP_SHIMPHY_ADDR_CNTL_0_SPARE1_RW_reserved_for_eco0_SHIFT 0
#define BCHP_SHIMPHY_ADDR_CNTL_0_SPARE1_RW_reserved_for_eco0_DEFAULT 0x00000000

/***************************************************************************
 *SPARE0_RO - Spare register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: SPARE0_RO :: reserved_for_eco0 [31:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_SPARE0_RO_reserved_for_eco0_MASK  0xffffffff
#define BCHP_SHIMPHY_ADDR_CNTL_0_SPARE0_RO_reserved_for_eco0_SHIFT 0
#define BCHP_SHIMPHY_ADDR_CNTL_0_SPARE0_RO_reserved_for_eco0_DEFAULT 0x00000000

/***************************************************************************
 *SPARE1_RO - Spare register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: SPARE1_RO :: reserved_for_eco0 [31:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_SPARE1_RO_reserved_for_eco0_MASK  0xffffffff
#define BCHP_SHIMPHY_ADDR_CNTL_0_SPARE1_RO_reserved_for_eco0_SHIFT 0
#define BCHP_SHIMPHY_ADDR_CNTL_0_SPARE1_RO_reserved_for_eco0_DEFAULT 0x00000000

/***************************************************************************
 *DDR3_RESET_CNTRL - FORCE_DDR3_RESET Deassert  Register
 ***************************************************************************/
/* SHIMPHY_ADDR_CNTL_0 :: DDR3_RESET_CNTRL :: UNUSED [31:01] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR3_RESET_CNTRL_UNUSED_MASK      0xfffffffe
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR3_RESET_CNTRL_UNUSED_SHIFT     1
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR3_RESET_CNTRL_UNUSED_DEFAULT   0x00000000

/* SHIMPHY_ADDR_CNTL_0 :: DDR3_RESET_CNTRL :: FORCE_DDR3_RESET [00:00] */
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR3_RESET_CNTRL_FORCE_DDR3_RESET_MASK 0x00000001
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR3_RESET_CNTRL_FORCE_DDR3_RESET_SHIFT 0
#define BCHP_SHIMPHY_ADDR_CNTL_0_DDR3_RESET_CNTRL_FORCE_DDR3_RESET_DEFAULT 0x00000001

#endif /* #ifndef BCHP_SHIMPHY_ADDR_CNTL_0_H__ */

/* End of File */
