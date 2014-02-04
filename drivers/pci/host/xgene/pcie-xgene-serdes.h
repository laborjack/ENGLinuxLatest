/**
 * APM X-Gene PCIe Serdes Header File
 *
 * Copyright (c) 2013 Applied Micro Circuits Corporation.
 *
 * Author: Tanmay Inamdar <tinamdar@apm.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#if !defined(__XGENE_PCIE_SERDES_H__)
#define __XGENE_PCIE_SERDES_H__
#include "pcie-xgene.h"
#include "pcie-xgene-core.h"

#define XGENE_PCIE_TIMEOUT	(500*1000) /* us */
#define SDS1_OFF		0x30000
#define CH_OFF			0x200

#define PCIECORE_CTLANDSTATUS		0x50

/* Serdes */
#define PIPE_CONTROL			0x10004
#define PIPE_COMMON_STATUS              0x10008
#define RX_LOS_VREF_COMMON              0x1002c
#define SKIP_PARAMETER                  0x1004c
#define RECEIVER_DETECT_CONTROL         0x10050
#define RECEIVER_DETECT_STATUS          0x10054
#define PIPE_STATUS_LANE0               0x10064
#define PIPE_STATUS_LANE1               0x1006c
#define PIPE_STATUS_LANE2               0x10074
#define PIPE_STATUS_LANE3               0x1007c
#define PIPE_STATUS_LANE4               0x10084
#define PIPE_STATUS_LANE5               0x1008c
#define PIPE_STATUS_LANE6               0x10094
#define PIPE_STATUS_LANE7               0x1009c
#define LOS_PARAM			0x100a0
#define SERDES_CONTROL0			0x100fc
#define SERDES_CONTROL1			0x10100
#define SERDES_CONTROL2			0x10104
#define SERDES_CONTROL3			0x10108
#define SERDES_CONTROL4			0x1010c
#define SERDES_CONTROL5			0x10110
#define SERDES_CONTROL6			0x10114
#define SERDES_CONTROL7			0x10118
#define SERDES_CONTROL8			0x1011c
#define SERDES_CONTROL9			0x10120
#define SERDES_CONTROL10		0x10124
#define SERDES_STATUS0			0x10128
#define EFIFO_CONTROL0			0x1012c
#define EFIFO_CONTROL1			0x10130
#define DFE_CONTROL0			0x10134
#define CLK_CONTROL0			0x10154
#define CLK_CONTROL1			0x10158
#define DETECT_CONTROL			0x1015c
#define LFPF_CTRL                       0x10160
#define USB_PCIE_CTRL                   0x10164
#define BCA_NO_ACK_TMR_SEL              0x10168

#define PIPECTLREG			0x1b0
#define PCIE_SDS_CTL0			0xa00c
#define IND_WDATA_REG			0xa01c
#define IND_RDATA_REG			0xa018
#define IND_CMD_REG			0xa014
#define SDS_MUX				0xa090
#define SDS0_CMU_STATUS0		0xa02c
#define SDS1_CMU_STATUS0		0xa064
#define IND_WR_CMD_MASK			0x1
#define IND_RD_CMD_MASK			0x2
#define IND_CMD_DONE_MASK		0x4

#define PCIE_SRST			0xc000
#define PCIE_CLKEN			0xc008
#define CORE_CLKEN_MASK			0x00000002
#define CORE_RESET_MASK			0x00000002

#define MEM_RAM_SHUTDOWN		0xd070
#define BLOCK_MEM_RDY			0xd074
#define CFG_CONSTANTS_383_352		0x202c

#define CH0_RXTX_REG0			0x400
#define CH0_RXTX_REG1			0x402
#define CH0_RXTX_REG2			0x404
#define CH0_RXTX_REG3			0x406
#define CH0_RXTX_REG4			0x408
#define CH0_RXTX_REG5			0x40a
#define CH0_RXTX_REG6			0x40c
#define CH0_RXTX_REG7			0x40e
#define CH0_RXTX_REG8			0x410
#define CH0_RXTX_REG9			0x412
#define CH0_RXTX_REG10			0x414
#define CH0_RXTX_REG11			0x416
#define CH0_RXTX_REG12			0x418
#define CH0_RXTX_REG13			0x41a
#define CH0_RXTX_REG14			0x41c
#define CH0_RXTX_REG15			0x41e
#define CH0_RXTX_REG16			0x420
#define CH0_RXTX_REG17			0x422
#define CH0_RXTX_REG18			0x424
#define CH0_RXTX_REG19			0x426
#define CH0_RXTX_REG20			0x428
#define CH0_RXTX_REG21			0x42a
#define CH0_RXTX_REG22			0x42c
#define CH0_RXTX_REG23			0x42e
#define CH0_RXTX_REG24			0x430
#define CH0_RXTX_REG25			0x432
#define CH0_RXTX_REG26			0x434
#define CH0_RXTX_REG27			0x436
#define CH0_RXTX_REG28			0x438
#define CH0_RXTX_REG29			0x43a
#define CH0_RXTX_REG30			0x43c
#define CH0_RXTX_REG31			0x43e
#define CH0_RXTX_REG32			0x440
#define CH0_RXTX_REG33			0x442
#define CH0_RXTX_REG34			0x444
#define CH0_RXTX_REG35			0x446
#define CH0_RXTX_REG36			0x448
#define CH0_RXTX_REG37			0x44a
#define CH0_RXTX_REG38			0x44c
#define CH0_RXTX_REG39			0x44e
#define CH0_RXTX_REG40			0x450
#define CH0_RXTX_REG41			0x452
#define CH0_RXTX_REG42			0x454
#define CH0_RXTX_REG43			0x456
#define CH0_RXTX_REG44			0x458
#define CH0_RXTX_REG45			0x45a
#define CH0_RXTX_REG46			0x45c
#define CH0_RXTX_REG47			0x45e
#define CH0_RXTX_REG48			0x460
#define CH0_RXTX_REG49			0x462
#define CH0_RXTX_REG50			0x464
#define CH0_RXTX_REG51			0x466
#define CH0_RXTX_REG52			0x468
#define CH0_RXTX_REG53			0x46a
#define CH0_RXTX_REG54			0x46c
#define CH0_RXTX_REG55			0x46e
#define CH0_RXTX_REG56			0x470
#define CH0_RXTX_REG57			0x472
#define CH0_RXTX_REG58			0x474
#define CH0_RXTX_REG59			0x476
#define CH0_RXTX_REG60			0x478
#define CH0_RXTX_REG61			0x47a
#define CH0_RXTX_REG62			0x47c
#define CH0_RXTX_REG63			0x47e
#define CH0_RXTX_REG64			0x480
#define CH0_RXTX_REG65			0x482
#define CH0_RXTX_REG66			0x484
#define CH0_RXTX_REG67			0x486
#define CH0_RXTX_REG68			0x488
#define CH0_RXTX_REG69			0x48a
#define CH0_RXTX_REG70			0x48c
#define CH0_RXTX_REG71			0x48e
#define CH0_RXTX_REG72			0x490
#define CH0_RXTX_REG73			0x492
#define CH0_RXTX_REG74			0x494
#define CH0_RXTX_REG75			0x496
#define CH0_RXTX_REG76			0x498
#define CH0_RXTX_REG77			0x49a
#define CH0_RXTX_REG78			0x49c
#define CH0_RXTX_REG79			0x49e
#define CH0_RXTX_REG80			0x4a0
#define CH0_RXTX_REG81			0x4a2
#define CH0_RXTX_REG82			0x4a4
#define CH0_RXTX_REG83			0x4a6
#define CH0_RXTX_REG84			0x4a8
#define CH0_RXTX_REG85			0x4aa
#define CH0_RXTX_REG86			0x4ac
#define CH0_RXTX_REG87			0x4ae
#define CH0_RXTX_REG88			0x4b0
#define CH0_RXTX_REG89			0x4b2
#define CH0_RXTX_REG90			0x4b4
#define CH0_RXTX_REG91			0x4b6
#define CH0_RXTX_REG92			0x4b8
#define CH0_RXTX_REG93			0x4ba
#define CH0_RXTX_REG94			0x4bc
#define CH0_RXTX_REG95			0x4be
#define CH0_RXTX_REG96			0x4c0
#define CH0_RXTX_REG97			0x4c2
#define CH0_RXTX_REG98			0x4c4
#define CH0_RXTX_REG99			0x4c6
#define CH0_RXTX_REG100			0x4c8
#define CH0_RXTX_REG101			0x4ca
#define CH0_RXTX_REG102			0x4cc
#define CH0_RXTX_REG103			0x4ce
#define CH0_RXTX_REG104			0x4d0
#define CH0_RXTX_REG105			0x4d2
#define CH0_RXTX_REG106			0x4d4
#define CH0_RXTX_REG107			0x4d6
#define CH0_RXTX_REG108			0x4d8
#define CH0_RXTX_REG109			0x4da
#define CH0_RXTX_REG110			0x4dc
#define CH0_RXTX_REG111			0x4de
#define CH0_RXTX_REG112			0x4e0
#define CH0_RXTX_REG113			0x4e2
#define CH0_RXTX_REG114			0x4e4
#define CH0_RXTX_REG115			0x4e6
#define CH0_RXTX_REG116			0x4e8
#define CH0_RXTX_REG117			0x4ea
#define CH0_RXTX_REG118			0x4ec
#define CH0_RXTX_REG119			0x4ee
#define CH0_RXTX_REG120			0x4f0
#define CH0_RXTX_REG121			0x4f2
#define CH0_RXTX_REG122			0x4f4
#define CH0_RXTX_REG123			0x4f6
#define CH0_RXTX_REG124			0x4f8
#define CH0_RXTX_REG125			0x4fa
#define CH0_RXTX_REG126			0x4fc
#define CH0_RXTX_REG127			0x4fe
#define CH0_RXTX_REG128			0x500
#define CH0_RXTX_REG129			0x502
#define CH0_RXTX_REG130			0x504
#define CH0_RXTX_REG131			0x506
#define CH0_RXTX_REG132			0x508
#define CH0_RXTX_REG133			0x50a
#define CH0_RXTX_REG134			0x50c
#define CH0_RXTX_REG135			0x50e
#define CH0_RXTX_REG136			0x510
#define CH0_RXTX_REG137			0x512
#define CH0_RXTX_REG138			0x514
#define CH0_RXTX_REG139			0x516
#define CH0_RXTX_REG140			0x518
#define CH0_RXTX_REG141			0x51a
#define CH0_RXTX_REG142			0x51c
#define CH0_RXTX_REG143			0x51e
#define CH0_RXTX_REG144			0x520
#define CH0_RXTX_REG145			0x522
#define CH0_RXTX_REG146			0x524
#define CH0_RXTX_REG147			0x526
#define CH0_RXTX_REG148			0x528
#define CH0_RXTX_REG149			0x52a
#define CH0_RXTX_REG150			0x52c
#define CH0_RXTX_REG151			0x52e
#define CH0_RXTX_REG152			0x530
#define CH0_RXTX_REG153			0x532
#define CH0_RXTX_REG154			0x534
#define CH0_RXTX_REG155			0x536
#define CH0_RXTX_REG156			0x538
#define CH0_RXTX_REG157			0x53a
#define CH0_RXTX_REG158			0x53c
#define CH0_RXTX_REG159			0x53e
#define CH0_RXTX_REG160			0x540
#define CH0_RXTX_REG161			0x542
#define CH0_RXTX_REG162			0x544

#define CMU_REG0	0x0
#define CMU_REG1	0x2
#define CMU_REG2	0x4
#define CMU_REG3	0x6
#define CMU_REG4	0x8
#define CMU_REG5	0xa
#define CMU_REG6	0xc
#define CMU_REG7	0xe
#define CMU_REG8	0x10
#define CMU_REG9	0x12
#define CMU_REG10	0x14
#define CMU_REG11	0x16
#define CMU_REG12	0x18
#define CMU_REG13	0x1a
#define CMU_REG14	0x1c
#define CMU_REG15	0x1e
#define CMU_REG16	0x20
#define CMU_REG17	0x22
#define CMU_REG18	0x24
#define CMU_REG19	0x26
#define CMU_REG20	0x28
#define CMU_REG21	0x2a
#define CMU_REG22	0x2c
#define CMU_REG23	0x2e
#define CMU_REG24	0x30
#define CMU_REG25	0x32
#define CMU_REG26	0x34
#define CMU_REG27	0x36
#define CMU_REG28	0x38
#define CMU_REG29	0x3a
#define CMU_REG30	0x3c
#define CMU_REG31	0x3e
#define CMU_REG32	0x40
#define CMU_REG33	0x42
#define CMU_REG34	0x44
#define CMU_REG35	0x46
#define CMU_REG36	0x48
#define CMU_REG37	0x4a
#define CMU_REG38	0x4c
#define CMU_REG39	0x4e

#define PIPE_REGS_LOS_PARAM		0x100a0
#define PIPE_REGS_EFIFO_CONTROL0 	0x1012c
#define PIPE_REGS_PIPE_CONTROL		0x10004
#define CFG_8G_CONSTANTS_287_256	0x2120
#define CFG_8G_CONSTANTS_319_288	0x2124
#define CFG_8G_CONSTANTS_351_320	0x2128
#define CFG_8G_CONSTANTS_383_352	0x212c
#define CFG_8G_CONSTANTS_159_128	0x2110
#define CFG_CONTROL_95_64 		0x2208
#define CFG_CONSTANTS_415_384		0x2030

/* generated macros to access bitfields */
#define ONE_CNT_TH_MASK			0x0000ffff
#define ONE_CNT_TH_SET(dst, src)	(((dst) & ~ONE_CNT_TH_MASK) |	\
					((u32)(src) & ONE_CNT_TH_MASK))

#define ONE_CNT_CMP_TH_MASK		0x0000ffff
#define ONE_CNT_CMP_TH_SET(dst, src)	(((dst) & ~ONE_CNT_CMP_TH_MASK) | \
					((u32)(src) & ONE_CNT_CMP_TH_MASK))

#define SEL_CDR_OVR_LN_MASK		0x0000000f
#define SEL_CDR_OVR_LN_SET(dst, src)	(((dst) & ~SEL_CDR_OVR_LN_MASK) | \
					((u32)(src) & SEL_CDR_OVR_LN_MASK))

#define TX_AMP_EN_LN0_MASK		0x00000040
#define TX_AMP_EN_LN0_SET(dst, src)	(((dst) & ~TX_AMP_EN_LN0_MASK) |      \
					(((u32)(src) << 0x6) &\
					TX_AMP_EN_LN0_MASK))

#define TX_AMP_LN0_MASK			0x00000780
#define TX_AMP_LN0_SET(dst, src)	(((dst) & ~TX_AMP_LN0_MASK) |	    \
					(((u32)(src) << 0x7) & \
					TX_AMP_LN0_MASK))

#define TX_AMP_EN_LN1_MASK		0x00010000
#define TX_AMP_EN_LN1_SET(dst, src)	(((dst) & ~TX_AMP_EN_LN1_MASK) |       \
					(((u32)(src) << 0x10) & \
					TX_AMP_EN_LN1_MASK))

#define TX_AMP_LN1_MASK			0x001e0000
#define TX_AMP_LN1_SET(dst, src)	(((dst) & ~TX_AMP_LN1_MASK) | \
					(((u32)(src) << 0x11) & \
					TX_AMP_LN1_MASK))

#define PLL_VCOMOMSEL_MASK		0x0000fc00
#define PLL_VCOMOMSEL_RD(src)		((PLL_VCOMOMSEL_MASK & (u32)(src)) >> \
					  0xa)
#define PLL_VCOMOMSEL_SET(dst, src)	(((dst) & ~PLL_VCOMOMSEL_MASK) | \
					(((u32)(src) << 0xa) & \
					PLL_VCOMOMSEL_MASK))

#define PLL_CALIB_DONE_MASK		0x00004000
#define PLL_CALIB_DONE_SHIFT		0xe
#define PLL_CALIB_DONE_RD(src)		((PLL_CALIB_DONE_MASK & (u32)(src)) >> \
					  0xe)
#define PLL_CALIB_DONE_SET(dst, src)	(((dst) & ~PLL_CALIB_DONE_MASK) | \
					(((u32)(src) << 0xe) &\
					PLL_CALIB_DONE_MASK))

#define VCOCAL_START_MASK		0x00004000
#define VCOCAL_START_SET(dst, src)	(((dst) & ~VCOCAL_START_MASK) | \
					(((u32)(src) << 0xe) & \
					VCOCAL_START_MASK))

#define POST_DIVBY2_MASK		0x00000008
#define POST_DIVBY2_SET(dst, src)	(((dst) & ~POST_DIVBY2_MASK) | \
					(((u32)(src) << 0x3) & \
					POST_DIVBY2_MASK))

#define PLL_REFDIV_MASK			0x0000c000
#define PLL_REFDIV_SET(dst, src)	(((dst) & ~PLL_REFDIV_MASK) | \
					(((u32)(src) << 0xe) & \
					PLL_REFDIV_MASK))

#define PCIE_MODE_MASK			0x00000008
#define PCIE_MODE_SET(dst, src)		(((dst) & ~PCIE_MODE_MASK) | \
					(((u32)(src) << 0x3) & \
					PCIE_MODE_MASK))

#define CUST_MODE_INV_MASK		0x0000ffff
#define CUST_MODE_INV_SET(dst, src)	(((dst) & ~CUST_MODE_INV_MASK) | \
					((u32)(src) & CUST_MODE_INV_MASK))

#define SEARCH_DONE_OVR_MASK		0x0000000f
#define SEARCH_DONE_OVR_SET(dst, src)	(((dst) & ~SEARCH_DONE_OVR_MASK) | \
					((u32)(src) & SEARCH_DONE_OVR_MASK))

#define CAL_DONE_OVR_MASK		0x0000f000
#define CAL_DONE_OVR_SET(dst, src)	(((dst) & ~CAL_DONE_OVR_MASK) | \
					(((u32)(src) << 0xc) & \
					CAL_DONE_OVR_MASK))

#define OVERRIDE_CH0_MASK		0x00000008
#define OVERRIDE_CH0_SET(dst, src)	(((dst) & ~OVERRIDE_CH0_MASK) | \
					(((u32)(src) << 0x3) & \
					OVERRIDE_CH0_MASK))

#define OVERRIDE_CH1_MASK		0x00000004
#define OVERRIDE_CH1_SET(dst, src)	(((dst) & ~OVERRIDE_CH1_MASK) | \
					(((u32)(src) << 0x2) \
					& OVERRIDE_CH1_MASK))

#define OVERRIDE_CH2_MASK		0x00000002
#define OVERRIDE_CH2_SHIFT		0x1
#define OVERRIDE_CH2_SET(dst, src)	(((dst) & ~OVERRIDE_CH2_MASK) | \
					(((u32)(src) << 0x1) & \
					OVERRIDE_CH2_MASK))

#define OVERRIDE_CH3_MASK		0x00000001
#define OVERRIDE_CH3_SET(dst, src)	(((dst) & ~OVERRIDE_CH3_MASK) | \
					((u32)(src) & OVERRIDE_CH3_MASK))

#define CAL_WAIT_SEL_MASK		0x00000006
#define CAL_WAIT_SEL_SET(dst, src)	(((dst) & ~CAL_WAIT_SEL_MASK) | \
					(((u32)(src) << 0x1) & \
					CAL_WAIT_SEL_MASK))

#define STATE_DELAY8_MASK		0x0000000f
#define STATE_DELAY8_SET(dst, src)	(((dst) & ~STATE_DELAY8_MASK) | \
					((u32)(src) & STATE_DELAY8_MASK))

#define STATE_DELAY7_MASK		0x000000f0
#define STATE_DELAY7_SET(dst, src)	(((dst) & ~STATE_DELAY7_MASK) | \
					(((u32)(src) << 0x4) & \
					STATE_DELAY7_MASK))

#define STATE_DELAY6_MASK		0x00000f00
#define STATE_DELAY6_SET(dst, src)	(((dst) & ~STATE_DELAY6_MASK) | \
					(((u32)(src) << 0x8) & \
					STATE_DELAY6_MASK))

#define STATE_DELAY1_MASK		0x0000f000
#define STATE_DELAY1_SET(dst, src)	(((dst) & ~STATE_DELAY1_MASK) | \
					(((u32)(src) << 0xc) & \
					STATE_DELAY1_MASK))

#define STATE_DELAY2_MASK		0x00000f00
#define STATE_DELAY2_SET(dst, src)	(((dst) & ~STATE_DELAY2_MASK) | \
					(((u32)(src) << 0x8) & \
					STATE_DELAY2_MASK))

#define STATE_DELAY3_MASK		0x000000f0
#define STATE_DELAY3_SET(dst, src)	(((dst) & ~STATE_DELAY3_MASK) | \
					(((u32)(src) << 0x4) & \
					STATE_DELAY3_MASK))

#define STATE_DELAY4_MASK		0x0000000f
#define STATE_DELAY4_SET(dst, src)	(((dst) & ~STATE_DELAY4_MASK) | \
					((u32)(src) & STATE_DELAY4_MASK))

#define STATE_DELAY5_MASK		0x0000f000
#define STATE_DELAY5_SET(dst, src)	(((dst) & ~STATE_DELAY5_MASK) | \
					(((u32)(src) << 0xc) & \
					STATE_DELAY5_MASK))

#define STATE_DELAY9_MASK		0x000000f0
#define STATE_DELAY9_SET(dst, src)	(((dst) & ~STATE_DELAY9_MASK) | \
					(((u32)(src) << 0x4) & \
					STATE_DELAY9_MASK))

#define FBDIV_GEN3_MASK			0x00003fe0
#define FBDIV_GEN3_SET(dst, src)	(((dst) & ~FBDIV_GEN3_MASK) | \
					(((u32)(src) << 0x5) & \
					FBDIV_GEN3_MASK))

#define LOCK_COUNT_MASK			0x00000006
#define LOCK_COUNT_SET(dst, src)	(((dst) & ~LOCK_COUNT_MASK) | \
					(((u32)(src) << 0x1) & \
					LOCK_COUNT_MASK))

#define REFDIV_GEN3_MASK		0x0000c000
#define REFDIV_GEN3_SET(dst, src)	(((dst) & ~REFDIV_GEN3_MASK) | \
					(((u32)(src) << 0xe) & \
					REFDIV_GEN3_MASK))

#define WAIT_BTW_CODE_MASK		0x0000001c
#define WAIT_BTW_CODE_SET(dst, src)	(((dst) & ~WAIT_BTW_CODE_MASK) | \
					(((u32)(src) << 0x2) & \
					WAIT_BTW_CODE_MASK))

#define CAL_COUNT_RESOL_MASK		0x000000e0
#define CAL_COUNT_RESOL_SET(dst, src)	(((dst) & ~CAL_COUNT_RESOL_MASK) | \
					(((u32)(src) << 0x5) & \
					CAL_COUNT_RESOL_MASK))

#define CAL_VTH_LO_MAX_MASK		0x0000000f
#define CAL_VTH_LO_MAX_SET(dst, src)	(((dst) & ~CAL_VTH_LO_MAX_MASK) | \
					((u32)(src) & CAL_VTH_LO_MAX_MASK))

#define CAL_VTH_LO_MIN_MASK		0x000000f0
#define CAL_VTH_LO_MIN_SET(dst, src)	(((dst) & ~CAL_VTH_LO_MIN_MASK) | \
					(((u32)(src) << 0x4) & \
					CAL_VTH_LO_MIN_MASK))

#define CAL_VTH_HI_MIN_MASK		0x0000f000
#define CAL_VTH_HI_MIN_SET(dst, src)	(((dst) & ~CAL_VTH_HI_MIN_MASK) | \
					(((u32)(src) << 0xc) & \
					CAL_VTH_HI_MIN_MASK))

#define CAL_VTH_HI_MAX_MASK		0x00000f00
#define CAL_VTH_HI_MAX_SET(dst, src)	(((dst) & ~CAL_VTH_HI_MAX_MASK) | \
					(((u32)(src) << 0x8) &\
					CAL_VTH_HI_MAX_MASK))

#define MOMSEL_INIT_PCIE_MASK		0x000003f0
#define MOMSEL_INIT_PCIE_SET(dst, src)	(((dst) & ~MOMSEL_INIT_PCIE_MASK) | \
					(((u32)(src) <<  0x4) & \
					MOMSEL_INIT_PCIE_MASK))

#define MOMSEL_INIT_MASK		0x000003f0
#define MOMSEL_INIT_SET(dst, src)	(((dst) & ~MOMSEL_INIT_MASK) | \
					(((u32)(src) << 0x4) & \
					MOMSEL_INIT_MASK))

#define IREF_ADJ_MASK			0x00000180
#define IREF_ADJ_SET(dst, src)		(((dst) & ~IREF_ADJ_MASK) | \
					(((u32)(src) << 0x7) & \
					IREF_ADJ_MASK))

#define VCOVARSEL_PCIE_MASK		0x0000000f
#define VCOVARSEL_PCIE_SET(dst, src)	(((dst) & ~VCOVARSEL_PCIE_MASK) | \
					((u32)(src) & VCOVARSEL_PCIE_MASK))

#define PLL_LFSMCAP_MASK		0x0000c000
#define PLL_LFSMCAP_SET(dst, src)	(((dst) & ~PLL_LFSMCAP_MASK) | \
					(((u32)(src) << 0xe) & \
					PLL_LFSMCAP_MASK))

#define PLL_REF_SEL_MASK		0x00002000
#define PLL_REF_SEL_SHIFT		0xd
#define PLL_REF_SEL_SET(dst, src)	(((dst) & ~PLL_REF_SEL_MASK) | \
					(((u32)(src) << 0xd) & \
					PLL_REF_SEL_MASK))

#define PLL_LFCAP_MASK			0x00003000
#define PLL_LFCAP_SHIFT			0xc
#define PLL_LFCAP_SET(dst, src)		(((dst) & ~PLL_LFCAP_MASK) | \
					(((u32)(src) << 0xc) & \
					PLL_LFCAP_MASK))

#define PLL_FBDIV_MASK			0x00003fe0
#define PLL_FBDIV_SHIFT			0x5
#define PLL_FBDIV_SET(dst, src)		(((dst) & ~PLL_FBDIV_MASK) | \
					(((u32)(src) << 0x5) & \
					PLL_FBDIV_MASK))

#define PLL_LFRES_MASK			0x0000001e
#define PLL_LFRES_SHIFT			0x1
#define PLL_LFRES_SET(dst, src)		(((dst) & ~PLL_LFRES_MASK) | \
					(((u32)(src) << 0x1) & \
					PLL_LFRES_MASK))

#define VCOVARSEL_MASK			0x0000000f
#define VCOVARSEL_SET(dst, src)		(((dst) & ~VCOVARSEL_MASK) | \
					((u32)(src) & VCOVARSEL_MASK))

#define PLL_CP_MASK			0x00003c00
#define PLL_CP_SET(dst, src)		(((dst) & ~PLL_CP_MASK) | \
					(((u32)(src) << 0xa) & \
					PLL_CP_MASK))

#define PLL_CP_SEL_MASK			0x000003e0
#define PLL_CP_SEL_SET(dst, src)	(((dst) & ~PLL_CP_SEL_MASK) | \
					(((u32)(src) << 0x5) & \
					PLL_CP_SEL_MASK))

#define PCIEGEN3_MASK			0x00000001
#define PCIEGEN3_SET(dst, src)		(((dst) & ~PCIEGEN3_MASK) | \
					((u32)(src) & PCIEGEN3_MASK))

#define DN_MAN_ENA_MASK			0x00000001
#define DN_MAN_ENA_SET(dst, src)	(((dst) & ~DN_MAN_ENA_MASK) | \
					((u32)(src) & DN_MAN_ENA_MASK))

#define TERM_MAN_ENA_MASK		0x00008000
#define TERM_MAN_ENA_SET(dst, src)	(((dst) & ~TERM_MAN_ENA_MASK) | \
					(((u32)(src) << 0xf) & \
					TERM_MAN_ENA_MASK))

#define PVT_CODE_R2A_MASK		0x00007f00
#define PVT_CODE_R2A_SET(dst, src)	(((dst) & ~PVT_CODE_R2A_MASK) | \
					(((u32)(src) << 0x8) & \
					PVT_CODE_R2A_MASK))

#define RESERVED_7_MASK			0x000000e0
#define RESERVED_7_SET(dst, src)	(((dst) & ~RESERVED_7_MASK) | \
					(((u32)(src) << 0x5) & \
					RESERVED_7_MASK))

#define MAN_PVT_CAL_MASK		0x00000004
#define MAN_PVT_CAL_SET(dst, src)	(((dst) & ~MAN_PVT_CAL_MASK) | \
					(((u32)(src) << 0x2) & \
					MAN_PVT_CAL_MASK))

#define PLL_VREGTRIM_MASK		0x00000600
#define PLL_VREGTRIM_SET(dst, src)	(((dst) & ~PLL_VREGTRIM_MASK) | \
					(((u32)(src) << 0x9) & \
					PLL_VREGTRIM_MASK))

#define PLL_MANUALCAL_MASK		0x00000008
#define PLL_MANUALCAL_SET(dst, src)	(((dst) & ~PLL_MANUALCAL_MASK) | \
					(((u32)(src) << 0x3) & \
					PLL_MANUALCAL_MASK))

#define VCO_MANMOMSEL_MASK		0x0000fc00
#define VCO_MANMOMSEL_SET(dst, src)	(((dst) & ~VCO_MANMOMSEL_MASK) | \
					(((u32)(src) << 0xa) & \
					VCO_MANMOMSEL_MASK))

#define MANMOMSEL_PCIE_MASK		0x0000fc00
#define MANMOMSEL_PCIE_SET(dst, src)	(((dst) & ~MANMOMSEL_PCIE_MASK) | \
					(((u32)(src) << 0xa) & \
					MANMOMSEL_PCIE_MASK))

#define UP_MAN_ENA_MASK			0x00000002
#define UP_MAN_ENA_SET(dst, src)	(((dst) & ~UP_MAN_ENA_MASK) | \
					(((u32)(src) << 0x1) & \
					UP_MAN_ENA_MASK))

#define RX_INV_MASK			0x00000800
#define RX_INV_SET(dst, src)		(((dst) & ~RX_INV_MASK) | \
					(((u32)(src) << 0xb) & \
					RX_INV_MASK))

#define BIST_ENA_RX_MASK		0x00000040
#define BIST_ENA_RX_SET(dst, src)	(((dst) & ~BIST_ENA_RX_MASK) | \
					(((u32)(src) << 0x6) & \
					BIST_ENA_RX_MASK))

#define LP_ENA_CTLE_MASK		0x00004000
#define LP_ENA_CTLE_SET(dst, src)	(((dst) & ~LP_ENA_CTLE_MASK) | \
					(((u32)(src) << 0xe) & \
					LP_ENA_CTLE_MASK))

#define RESETB_RXD_MASK			0x00000100
#define RESETB_RXD_SET(dst, src)	(((dst) & ~RESETB_RXD_MASK) | \
					(((u32)(src) << 0x8) & \
					RESETB_RXD_MASK))

#define RXDFE_CONFIG_MASK		0x0000c000
#define RXDFE_CONFIG_SET(dst, src)	(((dst) & ~RXDFE_CONFIG_MASK) | \
					(((u32)(src) << 0xe) & \
					RXDFE_CONFIG_MASK))

#define MU_DFE3_MASK		0x0000003e
#define MU_DFE3_SET(dst, src)	(((dst) & ~MU_DFE3_MASK) | \
				(((u32)(src) << 0x1) & \
				MU_DFE3_MASK))

#define MU_DFE1_MASK		0x0000f800
#define MU_DFE1_SET(dst, src)	(((dst) & ~MU_DFE1_MASK) | \
				(((u32)(src) << 0xb) & \
				MU_DFE1_MASK))

#define MU_DFE2_MASK		0x000007c0
#define MU_DFE2_SET(dst, src)	(((dst) & ~MU_DFE2_MASK) | \
				(((u32)(src) << 0x6) & \
				MU_DFE2_MASK))

#define BLWC_ENA_MASK		0x00000008
#define BLWC_ENA_SET(dst, src)	(((dst) & ~BLWC_ENA_MASK) | \
				(((u32)(src) << 0x3) & \
				BLWC_ENA_MASK))

#define SD_DISABLE_MASK		0x00000100
#define SD_DISABLE_SET(dst, src) (((dst) & ~SD_DISABLE_MASK) | \
				 (((u32)(src) << 0x8) & \
				 SD_DISABLE_MASK))

#define MU_PHASE3_MASK		0x0000003e
#define MU_PHASE3_SET(dst, src)	(((dst) & ~MU_PHASE3_MASK) | \
				(((u32)(src) << 0x1) & \
				MU_PHASE3_MASK))

#define MU_PHASE1_MASK		0x0000f800
#define MU_PHASE1_SET(dst, src)	(((dst) & ~MU_PHASE1_MASK) | \
				(((u32)(src) << 0xb) & \
				MU_PHASE1_MASK))

#define MU_PHASE2_MASK		0x000007c0
#define MU_PHASE2_SET(dst, src)	(((dst) & ~MU_PHASE2_MASK) | \
				(((u32)(src) << 0x6) & \
				MU_PHASE2_MASK))

#define MU_FREQ1_MASK		0x0000f800
#define MU_FREQ1_SET(dst, src)	(((dst) & ~MU_FREQ1_MASK) | \
				(((u32)(src) << 0xb) & \
				MU_FREQ1_MASK))

#define MU_FREQ2_MASK		0x000007c0
#define MU_FREQ2_SET(dst, src)	(((dst) & ~MU_FREQ2_MASK) | \
				(((u32)(src) << 0x6) & \
				MU_FREQ2_MASK))

#define MU_FREQ3_MASK		0x0000003e
#define MU_FREQ3_SET(dst, src)	(((dst) & ~MU_FREQ3_MASK) | \
				(((u32)(src) << 0x1) & \
				MU_FREQ3_MASK))

#define SSC_ENABLE_MASK			0x00000200
#define SSC_ENABLE_SET(dst, src)	(((dst) & ~SSC_ENABLE_MASK) | \
					(((u32)(src) << 0x9) & \
					SSC_ENABLE_MASK))

#define FREQLOOP_LIMIT_MASK		0x00000060
#define FREQLOOP_LIMIT_SHIFT		0x5
#define FREQLOOP_LIMIT_SET(dst, src)	(((dst) & ~FREQLOOP_LIMIT_MASK) | \
					(((u32)(src) << 0x5) & \
					FREQLOOP_LIMIT_MASK))

#define LOADFREQ_SHIFT_MASK		0x00000008
#define LOADFREQ_SHIFT_SET(dst, src)	(((dst) & ~LOADFREQ_SHIFT_MASK) | \
					(((u32)(src) << 0x3) & \
					LOADFREQ_SHIFT_MASK))

#define PHASE_ADJUST_LIMIT_MASK		0x0000f800
#define PHASE_ADJUST_LIMIT_SET(dst, src) (((dst) & ~PHASE_ADJUST_LIMIT_MASK) | \
					(((u32)(src) << 0xb) & \
					PHASE_ADJUST_LIMIT_MASK))

#define PHZ_MANUAL_MASK			0x00000002
#define PHZ_MANUAL_SET(dst, src)	(((dst) & ~PHZ_MANUAL_MASK) | \
					(((u32)(src) << 0x1) & \
					PHZ_MANUAL_MASK))

#define PQ_REG_MASK			0x0000fe00
#define PQ_REG_SET(dst, src)		(((dst) & ~PQ_REG_MASK) | \
					(((u32)(src) << 0x9) & \
					PQ_REG_MASK))

#define CDR_BYPASS_RXLOS_MASK		0x00000800
#define CDR_BYPASS_RXLOS_SET(dst, src)	(((dst) & ~CDR_BYPASS_RXLOS_MASK) | \
					(((u32)(src) << 0xb) & \
					CDR_BYPASS_RXLOS_MASK))

#define CDR_LOOP_ENA_MASK		0x00004000
#define CDR_LOOP_ENA_SET(dst, src)	(((dst) & ~CDR_LOOP_ENA_MASK) | \
					(((u32)(src) << 0xe) & \
					CDR_LOOP_ENA_MASK))

#define LATCH_CAL_WAIT_SEL_MASK		0x0000000c
#define LATCH_CAL_WAIT_SEL_SET(dst, src) (((dst) & ~LATCH_CAL_WAIT_SEL_MASK) |\
					 (((u32)(src) << 0x2) & \
					 LATCH_CAL_WAIT_SEL_MASK))

#define LATCH_OFF_ENA_MASK		0x00002000
#define LATCH_OFF_ENA_SET(dst, src)	(((dst) & ~LATCH_OFF_ENA_MASK) | \
					(((u32)(src) << 0xd) & \
					LATCH_OFF_ENA_MASK))

#define CTLE_EQ_HR_MASK			0x0000f800
#define CTLE_EQ_HR_SET(dst, src)	(((dst) & ~CTLE_EQ_HR_MASK) | \
					(((u32)(src) << 0xb) & \
					CTLE_EQ_HR_MASK))

#define CTLE_EQ_QR_MASK			0x000007c0
#define CTLE_EQ_QR_SET(dst, src)	(((dst) & ~CTLE_EQ_QR_MASK) | \
					(((u32)(src) << 0x6) & \
					CTLE_EQ_QR_MASK))

#define CTLE_EQ_FR_MASK			0x0000003e
#define CTLE_EQ_FR_SET(dst, src)	(((dst) & ~CTLE_EQ_FR_MASK) | \
					(((u32)(src) << 0x1) & \
					CTLE_EQ_FR_MASK))

#define CTLE_EQ_MASK			0x00000f80
#define CTLE_EQ_SET(dst, src)		(((dst) & ~CTLE_EQ_MASK) | \
					(((u32)(src) << 0x7) & \
					CTLE_EQ_MASK))

#define STMC_OVERRIDE_MASK		0x0000ffff
#define STMC_OVERRIDE_SET(dst, src)	(((dst) & ~STMC_OVERRIDE_MASK) | \
					((u32)(src) & STMC_OVERRIDE_MASK))

#define RX_DET_TERM_ENABLE_MASK		0x00000002
#define RX_DET_TERM_ENABLE_SET(dst, src) (((dst) & ~RX_DET_TERM_ENABLE_MASK) |\
					 (((u32)(src) << 0x1) & \
					 RX_DET_TERM_ENABLE_MASK))

#define RXACVCM_MASK			0x0000f000
#define RXACVCM_SET(dst, src)		(((dst) & ~RXACVCM_MASK) | \
					(((u32)(src) << 0xc) & \
					RXACVCM_MASK))

#define VTT_ENA_MASK			0x00000100
#define VTT_ENA_SET(dst, src)		(((dst) & ~VTT_ENA_MASK) | \
					(((u32)(src) << 0x8) & \
					VTT_ENA_MASK))

#define VTT_SEL_MASK			0x000000c0
#define VTT_SEL_SET(dst, src)		(((dst) & ~VTT_SEL_MASK) | \
					(((u32)(src) << 0x6) & \
					VTT_SEL_MASK))

#define RESETB_TERM_MASK		0x00004000
#define RESETB_TERM_SET(dst, src)	(((dst) & ~RESETB_TERM_MASK) | \
					(((u32)(src) << 0xe) & \
					RESETB_TERM_MASK))

#define TX_IDLE_SATA_MASK		0x00000001
#define TX_IDLE_SATA_SET(dst, src)	(((dst) & ~TX_IDLE_SATA_MASK) | \
					((u32)(src) & TX_IDLE_SATA_MASK))

#define TX_LOOPBACK_BUF_EN_MASK		0x00000040
#define TX_LOOPBACK_BUF_EN_SET(dst, src) (((dst) & ~TX_LOOPBACK_BUF_EN_MASK) |\
					 (((u32)(src) << 0x6) & \
					 TX_LOOPBACK_BUF_EN_MASK))

#define TX_CN1_MASK			0x0000f800
#define TX_CN1_SET(dst, src)		(((dst) & ~TX_CN1_MASK) | \
					(((u32)(src) << 0xb) & \
					TX_CN1_MASK))

#define TX_CP1_MASK			0x000007e0
#define TX_CP1_SET(dst, src)		(((dst) & ~TX_CP1_MASK) | \
					(((u32)(src) << 0x5) & \
					TX_CP1_MASK))

#define TX_CN2_MASK			0x0000001f
#define TX_CN2_SET(dst, src)		(((dst) & ~TX_CN2_MASK) | \
					((u32)(src) & TX_CN2_MASK))

#define TXAMP_CNTL_MASK			0x00000780
#define TXAMP_CNTL_SET(dst, src)	(((dst) & ~TXAMP_CNTL_MASK) | \
					(((u32)(src) << 0x7) & \
					TXAMP_CNTL_MASK))

#define TXAMP_ENA_MASK			0x00000040
#define TXAMP_ENA_SET(dst, src)		(((dst) & ~TXAMP_ENA_MASK) | \
					(((u32)(src) << 0x6) & \
					TXAMP_ENA_MASK))

#define TX_IDLE_MASK			0x00000008
#define TX_IDLE_SET(dst, src)		(((dst) & ~TX_IDLE_MASK) | \
					(((u32)(src) << 0x3) & \
					TX_IDLE_MASK))

#define RESETB_TXD_MASK			0x00001000
#define RESETB_TXD_SET(dst, src)	(((dst) & ~RESETB_TXD_MASK) | \
					(((u32)(src) << 0xc) & \
					RESETB_TXD_MASK))

#define BIST_ENA_TX_MASK		0x00000800
#define BIST_ENA_TX_SET(dst, src)	(((dst) & ~BIST_ENA_TX_MASK) | \
					(((u32)(src) << 0xb) & \
					BIST_ENA_TX_MASK))

#define TX_INV_MASK			0x00000400
#define TX_INV_SET(dst, src)		(((dst) & ~TX_INV_MASK) | \
					(((u32)(src) << 0xa) & \
					TX_INV_MASK))

#define TX_FIFO_ENA_MASK		0x00000020
#define TX_FIFO_ENA_SET(dst, src)	(((dst) & ~TX_FIFO_ENA_MASK) | \
					(((u32)(src) << 0x5) & \
					TX_FIFO_ENA_MASK))

#define TX_RCVDET_SEL_MASK		0x0000000c
#define TX_RCVDET_SEL_SHIFT_MASK	0x2
#define TX_RCVDET_SEL_SET(dst, src)	(((dst) & ~TX_RCVDET_SEL_MASK) | \
					 (((uint32_t)(src) << TX_RCVDET_SEL_SHIFT_MASK) \
					& TX_RCVDET_SEL_MASK))

#define RXPD_CONFIG_MASK		0x0000c000
#define RXPD_CONFIG_SHIFT_MASK		0xe
#define RXPD_CONFIG_SET(dst, src)	(((dst) & ~RXPD_CONFIG_MASK) | \
					 (((uint32_t)(src) << RXPD_CONFIG_SHIFT_MASK) & \
					 RXPD_CONFIG_MASK))

#define SD_VREF_MASK			0x000000f0
#define SD_VREF_SHIFT_MASK		0x4
#define SD_VREF_SET(dst, src)		(((dst) & ~SD_VREF_MASK) | \
					(((uint32_t)(src) << SD_VREF_SHIFT_MASK) \
					& SD_VREF_MASK))

#define VOLT_SEL_CH0_MASK		0x0000e000
#define VOLT_SEL_CH0_SHIFT_MASK		0xd
#define VOLT_SEL_CH0_SET(dst, src)	(((dst) & ~VOLT_SEL_CH0_MASK) | \
					(((uint32_t)(src) << VOLT_SEL_CH0_SHIFT_MASK) \
					& VOLT_SEL_CH0_MASK))

#define VOLT_SEL_CH1_MASK		0x00001c00
#define VOLT_SEL_CH1_SHIFT_MASK		0xa
#define VOLT_SEL_CH1_SET(dst, src)	(((dst) & ~VOLT_SEL_CH1_MASK) | \
					(((uint32_t)(src) << VOLT_SEL_CH1_SHIFT_MASK) \
					& VOLT_SEL_CH1_MASK))

#define VOLT_SEL_CH2_MASK		0x00000380
#define VOLT_SEL_CH2_SHIFT_MASK		0x7
#define VOLT_SEL_CH2_SET(dst, src)	(((dst) & ~VOLT_SEL_CH2_MASK) | \
					(((uint32_t)(src) << VOLT_SEL_CH2_SHIFT_MASK) \
					& VOLT_SEL_CH2_MASK))

#define VOLT_SEL_CH3_MASK		0x00000070
#define VOLT_SEL_CH3_SHIFT_MASK		0x4
#define VOLT_SEL_CH3_SET(dst, src)	(((dst) & ~VOLT_SEL_CH3_MASK) | \
					(((uint32_t)(src) << VOLT_SEL_CH3_SHIFT_MASK) \
					& VOLT_SEL_CH3_MASK))

#define FORCE_LAT_CAL_START_MASK		0x00000004
#define FORCE_LAT_CAL_START_SHIFT_MASK		0x2
#define FORCE_LAT_CAL_START_SET(dst, src)	(((dst) & ~FORCE_LAT_CAL_START_MASK) | \
						(((uint32_t)(src) << FORCE_LAT_CAL_START_SHIFT_MASK) \
						& FORCE_LAT_CAL_START_MASK))

#define FORCE_SUM_CAL_START_MASK		0x00000002
#define FORCE_SUM_CAL_START_SHIFT_MASK		0x1
#define FORCE_SUM_CAL_START_SET(dst, src)	(((dst) & ~FORCE_SUM_CAL_START_MASK) | \
						(((uint32_t)(src) << FORCE_SUM_CAL_START_SHIFT_MASK \
						 & FORCE_SUM_CAL_START_MASK)))


#define DO_LATCH_CALOUT_MASK			0x0000fc00
#define DO_LATCH_CALOUT_SHIFT_MASK		0xa
#define DO_LATCH_CALOUT_RD(src)			((DO_LATCH_CALOUT_MASK & (uint32_t)(src)) \
						>> DO_LATCH_CALOUT_SHIFT_MASK)

#define XO_LATCH_CALOUT_MASK			0x000003f0
#define XO_LATCH_CALOUT_SHIFT_MASK		0x4
#define XO_LATCH_CALOUT_RD(src)			((XO_LATCH_CALOUT_MASK & (uint32_t)(src)) \
						>> XO_LATCH_CALOUT_SHIFT_MASK)

#define LATCH_CAL_FAIL_ODD_MASK			0x0000000f
#define LATCH_CAL_FAIL_ODD_SHIFT_MASK		0x0
#define LATCH_CAL_FAIL_ODD_RD(src)		((LATCH_CAL_FAIL_ODD_MASK & (uint32_t)(src)))

#define EO_LATCH_MANCAL_MASK		0x0000fc00
#define EO_LATCH_MANCAL_SHIFT_MASK	0xa
#define EO_LATCH_MANCAL_RD(src)		((EO_LATCH_MANCAL_MASK & (uint32_t)(src)) \
					>> EO_LATCH_MANCAL_SHIFT_MASK)

#define SO_LATCH_MANCAL_MASK		0x000003f0
#define SO_LATCH_MANCAL_SHIFT_MASK	0x4
#define SO_LATCH_MANCAL_RD(src)		((SO_LATCH_MANCAL_MASK & (uint32_t)(src)) \
					>> SO_LATCH_MANCAL_SHIFT_MASK)

#define EO_LATCH_CALOUT_MASK		0x0000fc00
#define EO_LATCH_CALOUT_SHIFT_MASK	0xa
#define EO_LATCH_CALOUT_RD(src)		((EO_LATCH_CALOUT_MASK & (uint32_t)(src)) \
					>> EO_LATCH_CALOUT_SHIFT_MASK)

#define SO_LATCH_CALOUT_MASK		0x000003f0
#define SO_LATCH_CALOUT_SHIFT_MASK	0x4
#define SO_LATCH_CALOUT_RD(src)		((SO_LATCH_CALOUT_MASK & (uint32_t)(src)) \
					>> SO_LATCH_CALOUT_SHIFT_MASK)

#define LATCH_CAL_FAIL_EVEN_MASK	0x0000000f
#define LATCH_CAL_FAIL_EVEN_SHIFT_MASK	0x0
#define LATCH_CAL_FAIL_EVEN_RD(src)	((LATCH_CAL_FAIL_EVEN_MASK & (uint32_t)(src)))

#define DE_LATCH_CALOUT_MASK		0x0000fc00
#define DE_LATCH_CALOUT_SHIFT_MASK	0xa
#define DE_LATCH_CALOUT_RD(src)		((DE_LATCH_CALOUT_MASK & (uint32_t)(src)) \
					>> DE_LATCH_CALOUT_SHIFT_MASK)

#define XE_LATCH_CALOUT_MASK		0x000003f0
#define XE_LATCH_CALOUT_SHIFT_MASK	0x4
#define XE_LATCH_CALOUT_RD(src)		((XE_LATCH_CALOUT_MASK & (uint32_t)(src)) \
					>> XE_LATCH_CALOUT_SHIFT_MASK)

#define EE_LATCH_CALOUT_MASK		0x0000fc00
#define EE_LATCH_CALOUT_SHIFT_MASK	0xa
#define EE_LATCH_CALOUT_RD(src)		((EE_LATCH_CALOUT_MASK & (uint32_t)(src)) \
					>> EE_LATCH_CALOUT_SHIFT_MASK)

#define SE_LATCH_CALOUT_MASK		0x000003f0
#define SE_LATCH_CALOUT_SHIFT_MASK	0x4
#define SE_LATCH_CALOUT_RD(src)		((SE_LATCH_CALOUT_MASK & (uint32_t)(src)) \
					>> SE_LATCH_CALOUT_SHIFT_MASK)

#define SUMOS_CAL_CODE_MASK		0x0000003e
#define SUMOS_CAL_CODE_SHIFT_MASK	0x1
#define SUMOS_CAL_CODE_RD(src)		((SUMOS_CAL_CODE_MASK & (uint32_t)(src)) \
					>> SUMOS_CAL_CODE_SHIFT_MASK)

#define DO_LATCH_MANCAL_MASK		0x0000fc00
#define DO_LATCH_MANCAL_SHIFT_MASK	0xa
#define DO_LATCH_MANCAL_SET(dst, src)	(((dst) & ~DO_LATCH_MANCAL_MASK) | \
					(((uint32_t)(src) << DO_LATCH_MANCAL_SHIFT_MASK) \
					& DO_LATCH_MANCAL_MASK))

#define XO_LATCH_MANCAL_MASK		0x000003f0
#define XO_LATCH_MANCAL_SHIFT_MASK	0x4
#define XO_LATCH_MANCAL_SET(dst, src)	(((dst) & ~XO_LATCH_MANCAL_MASK) | \
					(((uint32_t)(src) << XO_LATCH_MANCAL_SHIFT_MASK) \
					& XO_LATCH_MANCAL_MASK))

#define EO_LATCH_MANCAL_MASK		0x0000fc00
#define EO_LATCH_MANCAL_SHIFT_MASK	0xa
#define EO_LATCH_MANCAL_SET(dst, src)	(((dst) & ~EO_LATCH_MANCAL_MASK) | \
					(((uint32_t)(src) << EO_LATCH_MANCAL_SHIFT_MASK) \
					& EO_LATCH_MANCAL_MASK))

#define SO_LATCH_MANCAL_MASK		0x000003f0
#define SO_LATCH_MANCAL_SHIFT_MASK	0x4
#define SO_LATCH_MANCAL_SET(dst, src)	(((dst) & ~SO_LATCH_MANCAL_MASK) | \
					(((uint32_t)(src) << SO_LATCH_MANCAL_SHIFT_MASK) \
					& SO_LATCH_MANCAL_MASK))

#define DE_LATCH_MANCAL_MASK		0x0000fc00
#define DE_LATCH_MANCAL_SHIFT_MASK	0xa
#define DE_LATCH_MANCAL_SET(dst, src)	(((dst) & ~DE_LATCH_MANCAL_MASK) | \
					(((uint32_t)(src) << DE_LATCH_MANCAL_SHIFT_MASK) \
					& DE_LATCH_MANCAL_MASK))

#define XE_LATCH_MANCAL_MASK		0x000003f0
#define XE_LATCH_MANCAL_SHIFT_MASK	0x4
#define XE_LATCH_MANCAL_SET(dst, src)	(((dst) & ~XE_LATCH_MANCAL_MASK) | \
					(((uint32_t)(src) << XE_LATCH_MANCAL_SHIFT_MASK) \
					& XE_LATCH_MANCAL_MASK))


#define EE_LATCH_MANCAL_MASK		0x0000fc00
#define EE_LATCH_MANCAL_SHIFT_MASK	0xa
#define EE_LATCH_MANCAL_SET(dst, src)	(((dst) & ~EE_LATCH_MANCAL_MASK) | \
					(((uint32_t)(src) << EE_LATCH_MANCAL_SHIFT_MASK) & \
					EE_LATCH_MANCAL_MASK))

#define SE_LATCH_MANCAL_MASK		0x000003f0
#define SE_LATCH_MANCAL_SHIFT_MASK	0x4
#define SE_LATCH_MANCAL_SET(dst, src)	(((dst) & ~SE_LATCH_MANCAL_MASK) | \
					(((uint32_t)(src) << SE_LATCH_MANCAL_SHIFT_MASK) & \
					SE_LATCH_MANCAL_MASK))

#define CLTE_LATCAL_MAN_PROG_MASK		0x0000003f
#define CLTE_LATCAL_MAN_PROG_SHIFT_MASK		0x0
#define CLTE_LATCAL_MAN_PROG_SET(dst, src)	(((dst) & ~CLTE_LATCAL_MAN_PROG_MASK) | \
						(((uint32_t)(src) << CLTE_LATCAL_MAN_PROG_SHIFT_MASK) & \
						CLTE_LATCAL_MAN_PROG_MASK))


#define CTLE_LATCAL_MAN_ENA_MASK	0x00000040
#define CTLE_LATCAL_MAN_ENA_SHIFT_MASK	0x6
#define CTLE_LATCAL_MAN_ENA_SET(dst, src)	(((dst) & ~CTLE_LATCAL_MAN_ENA_MASK) | \
					(((uint32_t)(src) << CTLE_LATCAL_MAN_ENA_SHIFT_MASK) & \
					CTLE_LATCAL_MAN_ENA_MASK))

#define LATCH_MAN_CAL_ENA_MASK		0x00000008
#define LATCH_MAN_CAL_ENA_SHIFT_MASK	0x3
#define LATCH_MAN_CAL_ENA_SET(dst, src)	(((dst) & ~LATCH_MAN_CAL_ENA_MASK) | \
					(((uint32_t)(src) << LATCH_MAN_CAL_ENA_SHIFT_MASK) & \
					LATCH_MAN_CAL_ENA_MASK))


#define ENABLE_L1S_POWER_MGMT_MASK		0x02000000
#define ENABLE_L1S_POWER_MGMT_SHIFT_MASK		0x19
#define ENABLE_L1S_POWER_MGMT_SET(dst, src)	(((dst) & ~ENABLE_L1S_POWER_MGMT_MASK) | \
						(((uint32_t)(src) << ENABLE_L1S_POWER_MGMT_SHIFT_MASK) & \
						ENABLE_L1S_POWER_MGMT_MASK))

#define EQ_UPDN_POST_STEP_MASK			0x00000030
#define EQ_UPDN_POST_STEP_SHIFT_MASK		0x4
#define EQ_UPDN_POST_STEP_SET(dst, src)	(((dst) & ~EQ_UPDN_POST_STEP_MASK) | \
						(((uint32_t)(src) << EQ_UPDN_POST_STEP_SHIFT_MASK) & \
						EQ_UPDN_POST_STEP_MASK))


#define EQ_PRE_CURSOR_LANE0_MASK		0x000000ff
#define EQ_PRE_CURSOR_LANE0_SHIFT_MASK		0x0
#define EQ_PRE_CURSOR_LANE0_SET(dst, src)	(((dst) & ~EQ_PRE_CURSOR_LANE0_MASK) | \
						(((uint32_t)(src) << EQ_PRE_CURSOR_LANE0_SHIFT_MASK) & \
						EQ_PRE_CURSOR_LANE0_MASK))

#define EQ_PRE_CURSOR_LANE1_MASK		0x00ff0000
#define EQ_PRE_CURSOR_LANE1_SHIFT_MASK		0x10
#define EQ_PRE_CURSOR_LANE1_SET(dst, src)	(((dst) & ~EQ_PRE_CURSOR_LANE1_MASK) | \
						(((uint32_t)(src) << EQ_PRE_CURSOR_LANE1_SHIFT_MASK) & \
						EQ_PRE_CURSOR_LANE1_MASK))

#define EQ_PRE_CURSOR_LANE2_MASK		0x000000ff
#define EQ_PRE_CURSOR_LANE2_SHIFT_MASK		0x0
#define EQ_PRE_CURSOR_LANE2_SET(dst, src)	(((dst) & ~EQ_PRE_CURSOR_LANE2_MASK) | \
						(((uint32_t)(src) << EQ_PRE_CURSOR_LANE2_SHIFT_MASK) & \
						EQ_PRE_CURSOR_LANE2_MASK))

#define EQ_PRE_CURSOR_LANE3_MASK		0x00ff0000
#define EQ_PRE_CURSOR_LANE3_SHIFT_MASK		0x10
#define EQ_PRE_CURSOR_LANE3_SET(dst, src)	(((dst) & ~EQ_PRE_CURSOR_LANE3_MASK) | \
						(((uint32_t)(src) << EQ_PRE_CURSOR_LANE3_SHIFT_MASK) & \
						EQ_PRE_CURSOR_LANE3_MASK))

#define EQ_PRE_CURSOR_LANE4_MASK		0x000000ff
#define EQ_PRE_CURSOR_LANE4_SHIFT_MASK		0x0
#define EQ_PRE_CURSOR_LANE4_SET(dst, src)	(((dst) & ~EQ_PRE_CURSOR_LANE4_MASK) | \
						(((uint32_t)(src) << EQ_PRE_CURSOR_LANE4_SHIFT_MASK) & \
						EQ_PRE_CURSOR_LANE4_MASK))

#define EQ_PRE_CURSOR_LANE5_MASK		0x00ff0000
#define EQ_PRE_CURSOR_LANE5_SHIFT_MASK		0x10
#define EQ_PRE_CURSOR_LANE5_SET(dst, src)	(((dst) & ~EQ_PRE_CURSOR_LANE5_MASK) | \
						(((uint32_t)(src) << EQ_PRE_CURSOR_LANE5_SHIFT_MASK) & \
						EQ_PRE_CURSOR_LANE5_MASK))

#define EQ_PRE_CURSOR_LANE6_MASK		0x000000ff
#define EQ_PRE_CURSOR_LANE6_SHIFT_MASK		0x0
#define EQ_PRE_CURSOR_LANE6_SET(dst, src)	(((dst) & ~EQ_PRE_CURSOR_LANE6_MASK) | \
						(((uint32_t)(src) << EQ_PRE_CURSOR_LANE6_SHIFT_MASK) & \
						EQ_PRE_CURSOR_LANE6_MASK))

#define EQ_PRE_CURSOR_LANE7_MASK		0x00ff0000
#define EQ_PRE_CURSOR_LANE7_SHIFT_MASK		0x10
#define EQ_PRE_CURSOR_LANE7_SET(dst, src)	(((dst) & ~EQ_PRE_CURSOR_LANE7_MASK) | \
						(((uint32_t)(src) << EQ_PRE_CURSOR_LANE7_SHIFT_MASK) & \
						 EQ_PRE_CURSOR_LANE7_MASK))

#define EQ_TX_PARAMS_VALID_MASK			0x01000000
#define EQ_TX_PARAMS_VALID_SHIFT_MASK		0x18
#define EQ_TX_PARAMS_VALID_SET(dst, src)	(((dst) & ~EQ_TX_PARAMS_VALID_MASK) | \
						(((uint32_t)(src) << EQ_TX_PARAMS_VALID_SHIFT_MASK) & \
						EQ_TX_PARAMS_VALID_MASK))

#define CNTON_GEN12_MASK			0xf0000000
#define CNTON_GEN12_SHIFT_MASK			0x1c
#define CNTON_GEN12_SET(dst, src)		(((dst) & ~CNTON_GEN12_MASK) | \
						(((uint32_t)(src) << CNTON_GEN12_SHIFT_MASK) & \
						CNTON_GEN12_MASK))

#define CNTOFF_GEN12_MASK			0x0f000000
#define CNTOFF_GEN12_SHIFT_MASK			0x18
#define CNTOFF_GEN12_SET(dst, src)		(((dst) & ~CNTOFF_GEN12_MASK) | \
						(((uint32_t)(src) << CNTOFF_GEN12_SHIFT_MASK) & \
						CNTOFF_GEN12_MASK))

#define CNTON_GEN3_MASK				0x00f00000
#define CNTON_GEN3_SHIFT_MASK			0x14
#define CNTON_GEN3_SET(dst, src)		(((dst) & ~CNTON_GEN3_MASK) | \
						(((uint32_t)(src) << CNTON_GEN3_SHIFT_MASK) & \
						CNTON_GEN3_MASK))

#define CNTOFF_GEN3_MASK			0x000f0000
#define CNTOFF_GEN3_SHIFT_MASK			0x10
#define CNTOFF_GEN3_SET(dst, src)		(((dst) & ~CNTOFF_GEN3_MASK) | \
						(((uint32_t)(src) << CNTOFF_GEN3_SHIFT_MASK) & \
						CNTOFF_GEN3_MASK))

#define WMSELECT_MASK				0x0000001e
#define WMSELECT_SHIFT_MASK			0x1
#define WMSELECT_SET(dst, src)			(((dst) & ~WMSELECT_MASK) | \
						(((uint32_t)(src) << WMSELECT_SHIFT_MASK) & \
						WMSELECT_MASK))

#define BUF_DEPTH_PCI_MASK			0x00001f00
#define BUF_DEPTH_PCI_SHIFT_MASK		0x8
#define BUF_DEPTH_PCI_SET(dst, src)		(((dst) & ~BUF_DEPTH_PCI_MASK) | \
						(((uint32_t)(src) << BUF_DEPTH_PCI_SHIFT_MASK) & \
						BUF_DEPTH_PCI_MASK))


#define CUSTOMER_PIN_MODE_SET(dst, src)		(((dst) & ~0x7fff) | \
						(((u32)(src)) & 0x7fff))
#define PIPECTLREG_PHY_EQ_TX_FS_SET(dst, src)	(((dst) & ~0xfc0000) | \
						(((u32)(src) << 0x12) & \
						0x00fc0000))
#define PIPECTLREG_PHY_EQ_TX_LF_SET(dst, src)	(((dst) & ~0x3f000) | \
						(((u32)(src) << 0xc) & \
						0x3f000))
#define PIPECTLREG_PHY_EQ_TX_MAX_PRE_SET(dst, src)	(((dst) & ~0xfc0) | \
							(((u32)(src) << 0x6) &\
							0xfc0))
#define PIPECTLREG_PHY_EQ_TX_MAX_POST_SET(dst, src)	(((dst) & ~0x3f) | \
							(((u32)(src)) & 0x3f))

#define BYPASS_RECEIVER_DETECTION_SET(dst, src)	(((dst) & ~0x10000000) | \
						(((u32)(src) << 0x1c)  & \
						0x10000000))
#define PIPE_PHY_RATE_RD(src)			((0xc000 & (u32)(src)) >> 0xe)
#define MGMT_US_PORT_TX_PRESET_SET(dst, src)	(((dst) & ~0xf00)| \
						(((u32)(src) << 0x8) & 0xf00))
#define MGMT_DS_PORT_TX_PRESET_SET(dst, src)	(((dst) & ~0xf) | \
						(((u32)(src)) & 0xf))
#define BLOCK_MEM_RDY_VAL			0xFFFFFFFF

enum {
	EXTERNAL_DIFFERENTIAL_CLK       = 0x0,
	INTERNAL_DIFFERENTIAL_CLK       = 0x1,
	INTERNAL_SINGLE_ENDED_CLK       = 0x2,
};

extern int xgene_pcie_out32(void *addr, u32 val);
extern int xgene_pcie_in32(void *addr, u32 *val);
void xgene_pcie_reset_pcie_core_clk(struct xgene_pcie_port *port);
int xgene_pcie_serdes_init(struct xgene_pcie_port *port);
#endif /* __XGENE_PCIE_SERDES_H__ */
