/**
 * sata_apm88xxxx.c - AppliedMicro APM88xxxx SATA PHY driver
 *
 * Copyright (c) 2013, Applied Micro Circuits Corporation
 * Author: Loc Ho <lho@apm.com>, Tuan Phan <tphan@apm.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 * SATA CSR Region Port 0/1
 *     Base Address: 0x1F21.0000
 *    Port Addresss: 0x1A00.0000
 * SATA CSR Region Port 2/3
 *     Base Address: 0x1F22.0000
 *    Port Addresss: 0x1A40.0000
 * SATA CSR Region Port 4/5
 *     Base Address: 0x1F23.0000
 *   Port Addresss: 0x1800.0000
 *
 *    Serdes Offset: 0x0000.A000
 *    Clock Offset: 0x0000.C000
 *     Diag Offset: 0x0000.D000
 *   Global Offset: 0x0000.D850
 *     Shim Offset: 0x0000.E000
 *   Master Offset: 0x0000.F000
 *   Port 0 Offset: 0x0000.0100
 *   Port 1 Offset: 0x0000.0180
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/ahci_platform.h>
#include <linux/libata.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include "ahci.h"
#ifndef CONFIG_ARCH_MSLIM
#endif
#include "sata_xgene_csr.h"
#include "sata_xgene_serdes_csr.h"
#ifdef CONFIG_ARCH_MSLIM
#include <asm/hardware/mslim-iof-map.h>
#include <asm/cacheflush.h>
#include <linux/dma-mapping.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi_cmnd.h>
#include <linux/delay.h>
#include <linux/device.h>
#endif

#undef SATA_DEBUG
#undef PHY_DEBUG
#undef PHY_ERROR
#undef PHYCSR_DEBUG

//#define SATA_DEBUG
//#define PHY_DEBUG
//#define PHY_ERROR

#ifdef SATA_DEBUG
#define SATADEBUG(fmt, args...) 	\
	do { \
		printk(KERN_INFO "APMSATA: " fmt "\n", ## args); \
	} while (0)
#else
#define SATADEBUG(fmt, args...)
#endif

#ifdef PHY_DEBUG
#define PHYDEBUG(fmt, args...) 		\
	do { \
		printk(KERN_INFO "APMSATA PHY: " fmt "\n", ## args); \
	} while (0)
#else
#define PHYDEBUG(fmt, args...)
#endif

#ifdef PHYCSR_DEBUG
#define PHYCSRDEBUG(fmt, args...) 	\
	do { \
		printk(KERN_INFO "APMSATA PHY CSR: " fmt "\n", ## args); \
	} while (0)
#else
#define PHYCSRDEBUG(fmt, args...)
#endif

#ifdef PHY_ERROR
#define PHYERROR(fmt, args...) 		\
	do { \
		printk(KERN_INFO "APMSATA PHY ERROR: " fmt "\n", ## args); \
	} while (0)
#else
#define PHYERROR(fmt, args...)
#endif

#define MAX_AHCI_CTR			3
#define MAX_AHCI_CHN_PERCTR	 	2

#define MAX_AHCI_PORT			(MAX_AHCI_CTR * MAX_AHCI_CHN_PERCTR)

#define SATA_ETH_MUX_OFFSET		0x00007000
#define SATA_SERDES_OFFSET		0x0000A000
#define SATA_CLK_OFFSET			0x0000C000
#define SATA_DIAG_OFFSET		0x0000D000
#define SATA_GLB_OFFSET			0x0000D850
#define SATA_SHIM_OFFSET		0x0000E000
#define SATA_MASTER_OFFSET		0x0000F000
#define SATA_PORT0_OFFSET		0x00000100
#define SATA_PORT1_OFFSET		0x00000180

#define SATA_RESET_MEM_RAM_TO		100000
#define SATA_PLL_TO			(1000*1000)
#define SATA_TXRDY_TO			(1000*1000)
#define SATA_DISPARITY_CLEAN_TO		100
/* serdes tunning macro */
#define FBDIV_VAL_50M   0x77
#define REFDIV_VAL_50M  0x1
#define FBDIV_VAL_100M 0x3B
#define REFDIV_VAL_100M 0x0
#define FBDIV_VAL  FBDIV_VAL_50M
#define REFDIV_VAL  REFDIV_VAL_50M 
#define CTLE_EQ 0x9
#define PQ_REG  0x8
#define SPD_SEL 0x5


struct apm88xxxx_sata_context {
	struct ahci_host_priv  hpriv;
	u8 cid;			/* Controller ID */
	int irq;		/* IRQ */
	void *csr_base;		/* CSR base address of IP - serdes */
	void *mmio_base;	/* AHCI I/O base address */
	void  *pcie_base;
	u64 csr_phys;		/* Physical address of CSR base address */
	u64 mmio_phys;		/* Physical address of MMIO base address */
};

/**
 * Configure CLK:
 *              External differential   (0)
 *              Internal differential   (1)
 *              Internal single ended   (2)
 */
#define SATA_CLK_EXT_DIFF	0
#define SATA_CLK_INT_DIFF	1
#define SATA_CLK_INT_SING	2
extern unsigned ata_exec_internal(struct ata_device *dev,
			   struct ata_taskfile *tf, const u8 *cdb,
			   int dma_dir, void *buf, unsigned int buflen,
			   unsigned long timeout);


static int apm_in32(void *addr, u32 *val)
{
        *val = readl(addr);
        PHYCSRDEBUG("SATAPHY CSR RD: 0x%llx value: 0x%08x", addr, *val);
        return 0;
}

static int apm_out32(void *addr, u32 val)
{
        writel(val, addr);
        PHYCSRDEBUG("SATAPHY CSR WR: 0x%llx value: 0x%08x", addr, val);
        return 0;
}

static int apm_out32_flush(void *addr, u32 val)
{
		writel(val, addr);
		PHYCSRDEBUG("SATAPHY CSR WR: 0x%llx value: 0x%08x", addr, val);
		val = readl(addr);
		return 0;
}

static void apm88xxxx_sds_wr_op(void *csr_base, u32 ind_cmd_reg_addr,
			u32 ind_wdata_reg_addr, u32 offset, u32 data)
{
	u32 cap_value;
	u32 ind_addr_cmd;

	ind_addr_cmd = (offset << 4) | 0x00000005;

	apm_out32(csr_base + ind_wdata_reg_addr, data);
	apm_out32(csr_base + ind_cmd_reg_addr, ind_addr_cmd);
	wmb();
	cap_value = 0;
	udelay(1000);
	while (cap_value != 0x00000004) {
		apm_in32(csr_base + ind_cmd_reg_addr, &cap_value);
		cap_value = cap_value & 0x00000004;
	}
}

static void apm88xxxx_sds_rd_op(void *csr_base, u32 ind_cmd_reg_addr,
			u32 ind_rdata_reg_addr, u32 offset, u32 *data)
{
	u32 cap_value;
	u32 ind_addr_cmd;

	ind_addr_cmd = (offset << 4) | 0x000000006;

	apm_out32(csr_base + ind_cmd_reg_addr, ind_addr_cmd);
	wmb();
    udelay(1000);
	cap_value = 0;
	while (cap_value != 0x00000004) {
		apm_in32(csr_base + ind_cmd_reg_addr, &cap_value);
		cap_value = cap_value & 0x00000004;
	}
	apm_in32(csr_base + ind_rdata_reg_addr, data);
}
void kc_serdes_pcie_wr (void *base ,unsigned int addr, unsigned int data) {
    int timeout;
  
     unsigned int wrdata;

    wrdata = 0x00;
    wrdata =  FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_ADDR_SET (wrdata, addr);
    wrdata =  FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_WR_CMD_SET (wrdata, 0x1); 
    wrdata =  FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_CMD_DONE_SET (wrdata, 0x1); 
   // printf("[SATA_INFO] : KOOLCHIP Write Reg 0x%08x ; DATA =0x%08x " , addr, data); printf ("\n\r");
    
    //write(SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_SDS_IND_WDATA_REG__ADDR, data);
    apm_out32(base+ SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_WDATA_REG__ADDR , data );
   // write(SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR, wrdata);    

    apm_out32(base+ SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR , wrdata );
    //wrdata = read(SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR);
    apm_in32(base+ SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR ,&wrdata );
    while ( FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_CMD_DONE_RD (wrdata)  != 0x1) {
       //wrdata = read(SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR);
       apm_in32((base+ SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR),&wrdata);
    } 

//    printf("[SATA_INFO] :  KOOLCHIP_WR_DONE ......");printf ("\n\r");

      
    wrdata =  FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_RD_CMD_SET (wrdata, 0x1); 
    wrdata =  FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_CMD_DONE_SET (wrdata, 0x1); 
    //write(SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR,wrdata); // Read register
    apm_out32((base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR),wrdata);
    //wrdata = read(SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR);
    apm_in32((base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR) ,&wrdata);
    while ( FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_CMD_DONE_RD (wrdata)  != 0x1) {
       //wrdata = read(SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR);
       apm_in32((base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR),&wrdata);
    }  
    //wrdata = read(SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_SDS_IND_RDATA_REG__ADDR);
    apm_in32((base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_RDATA_REG__ADDR) , &wrdata);

   for(timeout=0; timeout<0x1200; ++timeout);  // Tao comments need added this delay if no the test does not pass	
   	
   SATADEBUG("WRITE KC REG 0x%08x : DATA_RD=0x%08x <-> EXP_DATA=0x%08x \n",addr,wrdata,data);

}


unsigned int  kc_serdes_pcie_rd (void *base , unsigned int addr)  {
       
    unsigned int  wrdata;
    wrdata = 0x0;
    wrdata =  FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_ADDR_SET (wrdata, addr);
//    printf("[SATA_INFO] : KOOLCHIP Read Reg x%08x \n" ,wrdata );

    wrdata =  FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_RD_CMD_SET (wrdata, 0x1); 
    wrdata =  FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_CMD_DONE_SET (wrdata, 0x1); 
    //write(SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR,wrdata); // Read register
     apm_out32((base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR) ,wrdata);
    //wrdata = read(SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR);
    apm_in32((base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR) ,&wrdata );
    while ( FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_CMD_DONE_RD (wrdata)  != 0x1) {
      //wrdata = read(SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR);
      apm_in32((base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR), &wrdata);
     }  
    //wrdata = read(SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_SDS_IND_RDATA_REG__ADDR);
      apm_in32((base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_RDATA_REG__ADDR), &wrdata);
//  #ifdef  PRINT_IND
   // printf("[SATA_INFO] : KOOLCHIP READ REG 0x%08x >> DATA= 0x%08x \n",addr, wrdata);
// #endif
//    printf(" [SATA_INFO] : : KOOLCHIP_READ_DONE ....... \n");
   SATADEBUG("READ KC REG 0x%08x : DATA_RD=0x%08x  \n",addr,wrdata);
    return wrdata;

}
int kc_macro_calib_ready_check(struct apm88xxxx_sata_context *ctx) {

    int rc = 0;
    unsigned int val;
     void *csr_base = ctx->csr_base;
    void *csr_serdes_base = csr_base + SATA_SERDES_OFFSET;
	int loopcount =5;

    int timeout;
  // ********************
// TERM CALIBRATION CH0 
// ********************
    //data32 = kc_serdes_rd( KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR);
     	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR,
                &val);

    val = FIELD_CMU_REG17_PVT_CODE_R2A_SET(val,0x0d);
    val = FIELD_CMU_REG17_RESERVED_7_SET(val,0x0); 
   // kc_serdes_wr( KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR, data32);
	apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR,
                val);

    //data32 = kc_serdes_rd( KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR);
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR,
                &val);

      val = FIELD_CMU_REG17_PVT_TERM_MAN_ENA_SET(val,0x1);
    //kc_serdes_wr( KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR, data32);
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR,
                val);

 
    //data32 = kc_serdes_rd( KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR);
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR,
                &val);


    	val = FIELD_CMU_REG17_PVT_TERM_MAN_ENA_SET(val,0x0);

    //kc_serdes_wr( KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR, data32);
       apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR,
                val);




// *********************
// DOWN CALIBRATION CH0
// *********************
    //data32 = kc_serdes_rd( KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR);
    apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR,
                &val);

    val = FIELD_CMU_REG17_PVT_CODE_R2A_SET(val,0x26);
    val = FIELD_CMU_REG17_RESERVED_7_SET(val,0x0); 
    //kc_serdes_wr( KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR, data32);
     apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR,
                val);


    //data32 = kc_serdes_rd( KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR);
     apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR,
                &val);

     val = FIELD_CMU_REG16_PVT_DN_MAN_ENA_SET(val,0x1);
    //kc_serdes_wr( KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR, data32);
      apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR,
                val);


    //data32 = kc_serdes_rd( KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR);
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR,
                &val);

    val = FIELD_CMU_REG16_PVT_DN_MAN_ENA_SET(val,0x0);
    //kc_serdes_wr( KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR, data32);
    apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR,
                val);



// *********************
// UP CALIBRATION CH0
// *********************
    //data32 = kc_serdes_rd( KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR);
 apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR,
                &val);

    val = FIELD_CMU_REG17_PVT_CODE_R2A_SET(val,0x28);
    val = FIELD_CMU_REG17_RESERVED_7_SET(val,0x0); 
    //kc_serdes_wr( KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR, data32);

	apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR,
                val);


    //data32 = kc_serdes_rd( KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR);
      apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR,
                &val);

    val = FIELD_CMU_REG16_PVT_UP_MAN_ENA_SET(val,0x1);
    //kc_serdes_wr( KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR, data32);
    	apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR,
                val);


    //data32 = kc_serdes_rd( KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR);
	 apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR,
                &val);

       val = FIELD_CMU_REG16_PVT_UP_MAN_ENA_SET(val,0x0);
    //kc_serdes_wr( KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR, data32);
		apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR,
                val);

///
// Check for PLL calibration
     //data32 = kc_serdes_rd (KC_CLKMACRO_CMU_REGS_CMU_REG7__ADDR);
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG7__ADDR,
                &val);

     while (FIELD_CMU_REG7_PLL_CALIB_DONE_RD(val) == 0x0) {
	loopcount--;
        for(timeout=0; timeout<0x80000; ++timeout);
        //data32 = kc_serdes_rd (KC_CLKMACRO_CMU_REGS_CMU_REG7__ADDR);
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG7__ADDR,
                &val);

	if (loopcount==0) {
        printk ("[SATA_ERROR] ====>  CLKMACRO PLL CALIB DONE NOT DETECTED ...\n\r");
	break;
        }
      }

// PLL Calibration DONE 
    // data32 = kc_serdes_rd (KC_CLKMACRO_CMU_REGS_CMU_REG7__ADDR);
     apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG7__ADDR,
                &val);

     if (FIELD_CMU_REG7_PLL_CALIB_DONE_RD(val) == 0x1) {
      printk ("kc_macro_calib_ready_check() ====>  CLKMACRO PLL CALIB  Done ...\n\r");
     }
// Check for VCO FAIL 
     if (FIELD_CMU_REG7_VCO_CAL_FAIL_RD(val) == 0x0) {
        printk ("kc_macro_calib_ready_check() ====>  CLKMACRO CALIB Successfull...\n\r");
      }
     else {
    // Assert SDS reset and recall calib function
   //data32 = sm_sata_read (sm_sata_csr_region_base_addr + SM_SATA_ENET_SDS_CSR_REGS_SATA_ENET_CLK_MACRO_REG__ADDR);
    apm_in32(csr_serdes_base + SATA_ENET_CLK_MACRO_REG_ADDR, &val);

//   data32 = FIELD_SATA_ENET_CLK_MACRO_REG_I_RESET_B_SET(data32, 0x0);
//   data32 =   FIELD_SATA_ENET_CLK_MACRO_REG_I_CUSTOMEROV_SET(data32, 0x0);
   //sm_sata_write (sm_sata_csr_region_base_addr + SM_SATA_ENET_SDS_CSR_REGS_SATA_ENET_CLK_MACRO_REG__ADDR, data32);
   apm_out32(csr_serdes_base + SATA_ENET_CLK_MACRO_REG_ADDR, val);


    printk ("[SATA_ERR] kc_macro_calib_ready_check() ====>  CLKMACRO CALIB FAILED due to VCO FAIL...\n\r");
     rc = 1;
     }

return rc;
} // kc_macro_calib_ready_check

void  kc_macro_pdown_force_vco (struct apm88xxxx_sata_context *ctx) {
    unsigned int val;
    void *csr_base = ctx->csr_base;
    void *csr_serdes_base = csr_base + SATA_SERDES_OFFSET;
	
     //data32 = kc_serdes_rd (KC_CLKMACRO_CMU_REGS_CMU_REG0__ADDR);
     apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG0__ADDR ,
                &val);

     val = FIELD_CMU_REG0_PDOWN_SET(val, 1);
    // kc_serdes_wr( KC_CLKMACRO_CMU_REGS_CMU_REG0__ADDR, data32);
   apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG0__ADDR ,
                val);

 	udelay(800);

#if 0
//ADDING PLL RESET
    data32 = kc_serdes_rd (KC_CLKMACRO_CMU_REGS_CMU_REG5__ADDR);
    data32 = FIELD_CMU_REG5_PLL_RESETB_SET(data32,0);
    kc_serdes_wr( KC_CLKMACRO_CMU_REGS_CMU_REG5__ADDR, data32);
 
    delay(800);
    data32 = kc_serdes_rd (KC_CLKMACRO_CMU_REGS_CMU_REG5__ADDR);
    data32 = FIELD_CMU_REG5_PLL_RESETB_SET(data32,1);
    kc_serdes_wr( KC_CLKMACRO_CMU_REGS_CMU_REG5__ADDR, data32);
#endif
     //data32 = kc_serdes_rd (KC_CLKMACRO_CMU_REGS_CMU_REG0__ADDR);
	 apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG0__ADDR ,
                &val);

     val = FIELD_CMU_REG0_PDOWN_SET(val, 0);
     //kc_serdes_wr( KC_CLKMACRO_CMU_REGS_CMU_REG0__ADDR, data32);
	 apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG0__ADDR ,
                val);





     //data32 = kc_serdes_rd (KC_CLKMACRO_CMU_REGS_CMU_REG32__ADDR);
	 apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG32__ADDR ,
                &val);

     val = FIELD_CMU_REG32_FORCE_VCOCAL_START_SET(val, 1);


  //   kc_serdes_wr( KC_CLKMACRO_CMU_REGS_CMU_REG32__ADDR, val);
       apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG32__ADDR ,
                val);

     val =  FIELD_CMU_REG32_FORCE_VCOCAL_START_SET(val, 0);
     //kc_serdes_wr( KC_CLKMACRO_CMU_REGS_CMU_REG32__ADDR, val);
      apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG32__ADDR ,
                val);



}


int kc_sata45_macro_calib_ready_check(struct apm88xxxx_sata_context *ctx) {

    int rc = 0;
    unsigned int data32;
    int timeout;
    void *pcie_base=ctx->pcie_base;
	int loopcount =5;
  // ********************
// TERM CALIBRATION CH0 
// ********************
    data32 = kc_serdes_pcie_rd(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR);
    data32 = FIELD_CMU_REG17_PVT_CODE_R2A_SET(data32,0x0d);
    data32 = FIELD_CMU_REG17_RESERVED_7_SET(data32,0x0); 
    kc_serdes_pcie_wr(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR, data32);

    data32 = kc_serdes_pcie_rd(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR);
    data32 = FIELD_CMU_REG17_PVT_TERM_MAN_ENA_SET(data32,0x1);
    kc_serdes_pcie_wr(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR, data32);
 
    data32 = kc_serdes_pcie_rd(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR);
    data32 = FIELD_CMU_REG17_PVT_TERM_MAN_ENA_SET(data32,0x0);
    kc_serdes_pcie_wr(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR, data32);

// ********************
// *********************
// DOWN CALIBRATION CH0
// *********************
    data32 = kc_serdes_pcie_rd(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR);
    data32 = FIELD_CMU_REG17_PVT_CODE_R2A_SET(data32,0x26);
    data32 = FIELD_CMU_REG17_RESERVED_7_SET(data32,0x0); 
    kc_serdes_pcie_wr(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR, data32);

    data32 = kc_serdes_pcie_rd(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR);
    data32 = FIELD_CMU_REG16_PVT_DN_MAN_ENA_SET(data32,0x1);
    kc_serdes_pcie_wr(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR, data32);

    data32 = kc_serdes_pcie_rd( pcie_base,KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR);
    data32 = FIELD_CMU_REG16_PVT_DN_MAN_ENA_SET(data32,0x0);
    kc_serdes_pcie_wr(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR, data32);

// *********************
// UP CALIBRATION CH0
// *********************
    data32 = kc_serdes_pcie_rd(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR);
    data32 = FIELD_CMU_REG17_PVT_CODE_R2A_SET(data32,0x28);
    data32 = FIELD_CMU_REG17_RESERVED_7_SET(data32,0x0); 
    kc_serdes_pcie_wr(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG17__ADDR, data32);

    data32 = kc_serdes_pcie_rd(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR);
    data32 = FIELD_CMU_REG16_PVT_UP_MAN_ENA_SET(data32,0x1);
    kc_serdes_pcie_wr(pcie_base,  KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR, data32);

    data32 = kc_serdes_pcie_rd(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR);
    data32 = FIELD_CMU_REG16_PVT_UP_MAN_ENA_SET(data32,0x0);
    kc_serdes_pcie_wr(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR, data32);


// Check for PLL calibration
     data32 = kc_serdes_pcie_rd (pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG7__ADDR);
     while (FIELD_CMU_REG7_PLL_CALIB_DONE_RD(data32) == 0x0) {
	loopcount--;
        for(timeout=0; timeout<0x80000; ++timeout);
        data32 = kc_serdes_pcie_rd (pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG7__ADDR);
	if (loopcount==0) {
        printk ("[SATA_ERROR] ====>  CLKMACRO PLL CALIB DONE NOT DETECTED ...\n\r");
	break;
        }
      }

// PLL Calibration DONE 
     data32 = kc_serdes_pcie_rd (pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG7__ADDR);
     if (FIELD_CMU_REG7_PLL_CALIB_DONE_RD(data32) == 0x1) {
      printk ("kc_macro_calib_ready_check() ====>  CLKMACRO PLL CALIB  Done ...\n\r");
     }
// Check for VCO FAIL 
     if (FIELD_CMU_REG7_VCO_CAL_FAIL_RD(data32) == 0x0) {
        printk ("kc_macro_calib_ready_check() ====>  CLKMACRO CALIB Successfull...\n\r");
      }
     else {
#if 0

	     // Assert SDS reset and recall calib function
  // data32 = sm_sata_read (SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR);
   apm_in32((pcie_base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR), &data32);
   data32 = FIELD_PCIE_CLK_MACRO_REG_I_RESET_B_SET(data32, 0x0);
   data32 =   FIELD_PCIE_CLK_MACRO_REG_I_CUSTOMEROV_SET(data32, 0x0);
   //sm_sata_write (SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR, data32);
   apm_out32((pcie_base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR), data32);

 #endif
    printk ("[SATA_ERR] kc_macro_calib_ready_check() ====>  CLKMACRO CALIB FAILED due to VCO FAIL...\n\r");

     rc = 1;
     }


return rc;
} // kc_macro_calib_ready_check



int kc_macro_cfg(struct apm88xxxx_sata_context *ctx )
{
	int rc = 0;
	unsigned int  val;

  int i;   
  void *csr_base = ctx->csr_base;
  void *csr_serdes_base = csr_base + SATA_SERDES_OFFSET;
     int calib_loop_count = 0;

  //data32 = sm_sata_read (sm_sata_csr_region_base_addr + SM_SATA_ENET_SDS_CSR_REGS_SATA_ENET_CLK_MACRO_REG__ADDR);
  apm_in32(csr_serdes_base + SATA_ENET_CLK_MACRO_REG_ADDR, &val);
  //val = FIELD_SATA_ENET_CLK_MACRO_REG_I_RESET_B_SET(val, 0x0);
  val = I_RESET_B_SET(val, 0x0);
  //val = FIELD_SATA_ENET_CLK_MACRO_REG_I_PLL_FBDIV_SET(val, 0x27);
  val=  I_PLL_FBDIV_SET(val, 0x27); 
  //val = FIELD_SATA_ENET_CLK_MACRO_REG_I_CUSTOMEROV_SET(val, 0x0); 
  val=  I_CUSTOMEROV_SET(val, 0x0);
  //sm_sata_write (sm_sata_csr_region_base_addr + SM_SATA_ENET_SDS_CSR_REGS_SATA_ENET_CLK_MACRO_REG__ADDR, data32);

  apm_out32(csr_serdes_base + SATA_ENET_CLK_MACRO_REG_ADDR, val);
//data32 = kc_serdes_rd(  KC_CLKMACRO_CMU_REGS_CMU_REG34__ADDR );
apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG34__ADDR,
                &val);

val = FIELD_CMU_REG34_VCO_CAL_VTH_LO_MAX_SET(val, 0x7);
val = FIELD_CMU_REG34_VCO_CAL_VTH_HI_MAX_SET(val, 0xd);
val = FIELD_CMU_REG34_VCO_CAL_VTH_LO_MIN_SET(val, 0x2);
val = FIELD_CMU_REG34_VCO_CAL_VTH_HI_MIN_SET(val, 0x8);
//kc_serdes_wr(   KC_CLKMACRO_CMU_REGS_CMU_REG34__ADDR , data32);
apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG34__ADDR,
		val);
	
   ///data32 = kc_serdes_rd(KC_CLKMACRO_CMU_REGS_CMU_REG0__ADDR);
apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG0__ADDR,
                &val);

  	val =  FIELD_CMU_REG0_CAL_COUNT_RESOL_SET(val, 0x4);

	//kc_serdes_wr(KC_CLKMACRO_CMU_REGS_CMU_REG0__ADDR, data32);
	apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG0__ADDR,
		val);


//CMU_REG1
	//data32 = kc_serdes_rd(KC_CLKMACRO_CMU_REGS_CMU_REG1__ADDR);
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG1__ADDR,
                &val);



        val=  FIELD_CMU_REG1_PLL_CP_SET(val, 0x1);     
        val =  FIELD_CMU_REG1_PLL_CP_SEL_SET(val, 0x5); //JITU_ANIL 3/23/2013
        val = FIELD_CMU_REG1_PLL_MANUALCAL_SET(val, 0x0);  // JITU_ANIL 3/23/2013 
	//kc_serdes_wr(KC_CLKMACRO_CMU_REGS_CMU_REG1__ADDR, data32);
	apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG1__ADDR,
                val);



//CMU_REG2
	//data32 = kc_serdes_rd(KC_CLKMACRO_CMU_REGS_CMU_REG2__ADDR);
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG2__ADDR,
                &val);

	
  	val =  FIELD_CMU_REG2_PLL_LFRES_SET(val, 0xa);   // JITU_ANIL  03/25/2013 
   	val = FIELD_CMU_REG2_PLL_FBDIV_SET(val, 0x27);      //100Mhz refclk
  	//kc_serdes_wr(KC_CLKMACRO_CMU_REGS_CMU_REG2__ADDR, data32); 
	apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG2__ADDR,
                val);

  

//CMU_REG3
	 //data32 = kc_serdes_rd( KC_CLKMACRO_CMU_REGS_CMU_REG3__ADDR);
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG3__ADDR,
                &val);

 	 val = FIELD_CMU_REG3_VCOVARSEL_SET(val,0x3); //JITU_ANIL 03/23/20113 
	 //ANIL changed this value from 0x15 to 0x16 on 04/15/20113 
	 val = FIELD_CMU_REG3_VCO_MOMSEL_INIT_SET(val,0x10); 
//	 data32 = FIELD_CMU_REG3_VCO_MANMOMSEL_SET(data32,0x10); //JITU_ANIL 03/23/20113 
	 //kc_serdes_wr(KC_CLKMACRO_CMU_REGS_CMU_REG3__ADDR, data32);
	apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG3__ADDR,
                val);
  
//CMU_REG26   Added on 3/23/2013
	//data32 = kc_serdes_rd( KC_CLKMACRO_CMU_REGS_CMU_REG26__ADDR);
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG26__ADDR,
                &val);

        val = FIELD_CMU_REG26_FORCE_PLL_LOCK_SET(val,0x0); //JITU_ANIL 03/23/20113 
	//kc_serdes_wr(KC_CLKMACRO_CMU_REGS_CMU_REG26__ADDR, data32);
	apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG26__ADDR,
                val);


//CMU_REG5   
   	//data32 = kc_serdes_rd(KC_CLKMACRO_CMU_REGS_CMU_REG5__ADDR);
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG5__ADDR,
                &val);


	val = FIELD_CMU_REG5_PLL_LFSMCAP_SET(val,0x3); 
	val = FIELD_CMU_REG5_PLL_LFCAP_SET(val,0x3); 
	val = FIELD_CMU_REG5_PLL_LOCK_RESOLUTION_SET(val,0x7); //JITU_ANIL 03/23/20113 
	//kc_serdes_wr(KC_CLKMACRO_CMU_REGS_CMU_REG5__ADDR, data32);
	apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG5__ADDR,
                val);




//CMU_reg6
	//data32 = kc_serdes_rd(KC_CLKMACRO_CMU_REGS_CMU_REG6__ADDR);
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG6__ADDR,
                &val);


	val = FIELD_CMU_REG6_PLL_VREGTRIM_SET(val,0x0); 
	val = FIELD_CMU_REG6_MAN_PVT_CAL_SET(val,0x1); 
	//kc_serdes_wr(KC_CLKMACRO_CMU_REGS_CMU_REG6__ADDR, data32);
	apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG6__ADDR,
                val);



//CMU_reg16
	//data32 = kc_serdes_rd(KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR);
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR,
                &val);


	val = FIELD_CMU_REG16_CALIBRATION_DONE_OVERRIDE_SET(val,0x1); //JITU_ANIL 03/25/20113 
        val = FIELD_CMU_REG16_BYPASS_PLL_LOCK_SET(val,0x1); //JITU_ANIL 03/23/20113 
	val = FIELD_CMU_REG16_VCOCAL_WAIT_BTW_CODE_SET(val,0x4); //JITU_ANIL 03/23/20113 
	//kc_serdes_wr(KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR, data32);
	apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR,
                val);


//CMU_reg30
	//data32 = kc_serdes_rd(KC_CLKMACRO_CMU_REGS_CMU_REG30__ADDR);
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG30__ADDR,
                &val);


	val = FIELD_CMU_REG30_PCIE_MODE_SET(val,0x0); 
	val = FIELD_CMU_REG30_LOCK_COUNT_SET(val,0x3); //JITU_ANIL 03/23/20113 
	//kc_serdes_wr(KC_CLKMACRO_CMU_REGS_CMU_REG30__ADDR, data32);
	apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG30__ADDR,
                val);

	//kc_serdes_wr(KC_CLKMACRO_CMU_REGS_CMU_REG31__ADDR, 0xF);
	apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG31__ADDR,
                0xF);
	

//CMU_reg32
	//data32 = kc_serdes_rd( KC_CLKMACRO_CMU_REGS_CMU_REG32__ADDR);
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG32__ADDR,
                &val);

	val |= 0x0006 | 0x0180;
	val = FIELD_CMU_REG32_PVT_CAL_WAIT_SEL_SET(val,0x3); //JITU_ANIL 03/23/20113 
	val = FIELD_CMU_REG32_IREF_ADJ_SET(val,0x3);

	//kc_serdes_wr(KC_CLKMACRO_CMU_REGS_CMU_REG32__ADDR, data32);
	apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG32__ADDR,
                val);
	

//CMU_reg34
//	kc_serdes_wr( KC_CLKMACRO_CMU_REGS_CMU_REG34__ADDR, 0x2A2A);
//	kc_serdes_wr( KC_CLKMACRO_CMU_REGS_CMU_REG34__ADDR, 0x8d27);

	apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG34__ADDR,
                0x8d27);


//CMU_reg37
	//data32 = kc_serdes_rd(KC_CLKMACRO_CMU_REGS_CMU_REG37__ADDR);
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG37__ADDR,
                &val);

	//kc_serdes_wr( KC_CLKMACRO_CMU_REGS_CMU_REG37__ADDR, 0xF00F);
	apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_CLKMACRO_CMU_REGS_CMU_REG37__ADDR,
                0xF00F);

 apm_in32(csr_serdes_base + SATA_ENET_CLK_MACRO_REG_ADDR, &val);
  //val = FIELD_SATA_ENET_CLK_MACRO_REG_I_RESET_B_SET(val, 0x0);
  val = I_RESET_B_SET(val, 0x1);
  val=  I_CUSTOMEROV_SET(val, 0x0);
 apm_out32(csr_serdes_base + SATA_ENET_CLK_MACRO_REG_ADDR, val);

#if 0
  data32 = sm_sata_read (sm_sata_csr_region_base_addr + SM_SATA_ENET_SDS_CSR_REGS_SATA_ENET_CLK_MACRO_REG__ADDR);
  data32 = FIELD_SATA_ENET_CLK_MACRO_REG_I_RESET_B_SET(data32, 0x1);
  data32 =   FIELD_SATA_ENET_CLK_MACRO_REG_I_CUSTOMEROV_SET(data32, 0x0);
  sm_sata_write (sm_sata_csr_region_base_addr + SM_SATA_ENET_SDS_CSR_REGS_SATA_ENET_CLK_MACRO_REG__ADDR, data32);
#endif
  mdelay(800);
  

	  rc = 0;
      while (rc || calib_loop_count<5) {
 #if 0
       data32 = sm_sata_read (sm_sata_csr_region_base_addr + SM_SATA_ENET_SDS_CSR_REGS_SATA_ENET_CLK_MACRO_REG__ADDR);
       data32 = FIELD_SATA_ENET_CLK_MACRO_REG_I_RESET_B_SET(data32, 0x1);
       data32 =   FIELD_SATA_ENET_CLK_MACRO_REG_I_CUSTOMEROV_SET(data32, 0x0);
       sm_sata_write (sm_sata_csr_region_base_addr + SM_SATA_ENET_SDS_CSR_REGS_SATA_ENET_CLK_MACRO_REG__ADDR, data32);
      delay(8000);
 #endif
    if(rc) kc_macro_pdown_force_vco(ctx);
	  rc = kc_macro_calib_ready_check(ctx);
	  if (rc==0) break;
	  ++calib_loop_count;
      printk(" sata_sm_sds_config() calib_loop_count=%d rc = %d.. \n\r",calib_loop_count,rc);
      }
      if (calib_loop_count==5) goto  end;
 
  i = 10;
  if (O_PLL_LOCK_RD(val) && ( i > 0)) {
      printk ("  =============>>>> PLL CLKMACRO LOOKED ...\n");
      udelay(100);
  } 
  else { printk ("  =============>>>> PLL CLKMACRO UN-LOOKED ...\n");
  }

   if (O_PLL_READY_RD(val)) {
      printk ("  =============>>>> PLL CLKMACRO READY ...\n");
      

   } 
  else  {  printk ("  =============>>>> PLL CLKMACRO NOT READY ...\n"); }

end:
	return rc;
}

void  kc_sata45_macro_pdown_force_vco (struct apm88xxxx_sata_context *ctx) {
    unsigned int data32;
    void *pcie_base=ctx->pcie_base;
  
     printk (" serdes_pdown_force_vco () \n");
     data32 = kc_serdes_pcie_rd (pcie_base,KC_CLKMACRO_CMU_REGS_CMU_REG0__ADDR);
     data32 = FIELD_CMU_REG0_PDOWN_SET(data32, 1);
     kc_serdes_pcie_wr(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG0__ADDR, data32);

//ADDING PLL RESET
#if 0
    data32 = kc_serdes_pcie_rd (pcie_base,KC_CLKMACRO_CMU_REGS_CMU_REG5__ADDR);
    data32 = FIELD_CMU_REG5_PLL_RESETB_SET(data32,0);
    kc_serdes_pcie_wr( pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG5__ADDR, data32);
 
    udelay(800);
    data32 = kc_serdes_pcie_rd (pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG5__ADDR);
    data32 = FIELD_CMU_REG5_PLL_RESETB_SET(data32,1);
    kc_serdes_pcie_wr( pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG5__ADDR, data32);
#endif    
    udelay(1000);
     data32 = kc_serdes_pcie_rd (pcie_base,KC_CLKMACRO_CMU_REGS_CMU_REG0__ADDR);
     data32 = FIELD_CMU_REG0_PDOWN_SET(data32, 0);
     kc_serdes_pcie_wr( pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG0__ADDR, data32);



     data32 = kc_serdes_pcie_rd (pcie_base,KC_CLKMACRO_CMU_REGS_CMU_REG32__ADDR);
     data32 = FIELD_CMU_REG32_FORCE_VCOCAL_START_SET(data32, 1);
     kc_serdes_pcie_wr(pcie_base,  KC_CLKMACRO_CMU_REGS_CMU_REG32__ADDR, data32);

     data32 =  FIELD_CMU_REG32_FORCE_VCOCAL_START_SET(data32, 0);
     kc_serdes_pcie_wr(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG32__ADDR, data32);

}




void sata45_config_internal_clk (struct apm88xxxx_sata_context * ctx) {
  unsigned int data32,i;
  void *pcie_base=ctx->pcie_base;

  apm_out32((pcie_base + SM_PCIE_CLKRST_CSR_PCIE_CLKEN__ADDR), 0xff);
  
  //data32 = sm_sata_read_mon(SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_CLKRST_CSR_PCIE_CLKEN__ADDR);
  apm_in32((pcie_base + SM_PCIE_CLKRST_CSR_PCIE_CLKEN__ADDR), &data32);
  //printf (" SM_PCIE_CLKRST_CSR_PCIE_CLKEN__ADDR = %X \n", data32);
  for (i=0;i <=5 ; i++) 
  udelay(1000);
   
   //printk("pcie_base 3 0x%llX\n", pcie_base);
 // sm_sata_write(SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_CLKRST_CSR_PCIE_SRST__ADDR, 0x00);
 apm_out32((pcie_base + SM_PCIE_CLKRST_CSR_PCIE_SRST__ADDR), 0x00);
 // data32 = sm_sata_read_mon(SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_CLKRST_CSR_PCIE_SRST__ADDR);
  apm_in32((pcie_base + SM_PCIE_CLKRST_CSR_PCIE_SRST__ADDR),&data32);
   for (i=0;i <=5 ; i++) 
  udelay(1000);

}

void sata45_reset_cmos ( struct apm88xxxx_sata_context *ctx) {
	void *pcie_base = ctx->pcie_base;
//sm_sata_write(SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_CLKRST_CSR_PCIE_CLKEN__ADDR, 0x00);
apm_out32((pcie_base + SM_PCIE_CLKRST_CSR_PCIE_CLKEN__ADDR), 0x00);
 //sm_sata_write(SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_CLKRST_CSR_PCIE_SRST__ADDR, 0xff);
 apm_out32((pcie_base + SM_PCIE_CLKRST_CSR_PCIE_SRST__ADDR), 0xff);

}


int sata45_kc_macro_cfg( struct apm88xxxx_sata_context *ctx)
{
	int rc = 0;
	unsigned int  data32;
  int i;   
   void *pcie_base= ctx->pcie_base;
     int calib_loop_count = 0;
   //printk("pcie_base 0x%llX\n", pcie_base);
   sata45_config_internal_clk (ctx);
  //data32 = sm_sata_read_mon (SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR);
  apm_in32((pcie_base +SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR), &data32 );
  data32 = FIELD_PCIE_CLK_MACRO_REG_I_RESET_B_SET(data32, 0x0);
  data32 = FIELD_PCIE_CLK_MACRO_REG_I_PLL_FBDIV_SET(data32, 0x27);
  data32 = FIELD_PCIE_CLK_MACRO_REG_I_CUSTOMEROV_SET(data32, 0x0); 
  //sm_sata_write_mon (SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR, data32);

  apm_out32((pcie_base +SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR), data32 );
  apm_in32((pcie_base +SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR), &data32 );

data32 = kc_serdes_pcie_rd( pcie_base , KC_CLKMACRO_CMU_REGS_CMU_REG34__ADDR );
data32 = FIELD_CMU_REG34_VCO_CAL_VTH_LO_MAX_SET(data32, 0x7);
data32 = FIELD_CMU_REG34_VCO_CAL_VTH_HI_MAX_SET(data32, 0xd);
data32 = FIELD_CMU_REG34_VCO_CAL_VTH_LO_MIN_SET(data32, 0x2);
data32 = FIELD_CMU_REG34_VCO_CAL_VTH_HI_MIN_SET(data32, 0x8);
kc_serdes_pcie_wr( pcie_base ,  KC_CLKMACRO_CMU_REGS_CMU_REG34__ADDR , data32);
data32 = kc_serdes_pcie_rd( pcie_base , KC_CLKMACRO_CMU_REGS_CMU_REG34__ADDR );


//CMU_REG0
	data32 = kc_serdes_pcie_rd(pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG0__ADDR);

  	data32 =  FIELD_CMU_REG0_CAL_COUNT_RESOL_SET(data32, 0x4);
	kc_serdes_pcie_wr(pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG0__ADDR, data32);

//CMU_REG1
	data32 = kc_serdes_pcie_rd(pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG1__ADDR);
        data32 =  FIELD_CMU_REG1_PLL_CP_SET(data32, 0x1);     
        data32 =  FIELD_CMU_REG1_PLL_CP_SEL_SET(data32, 0x5); //JITU_ANIL 3/23/2013
        data32 = FIELD_CMU_REG1_PLL_MANUALCAL_SET(data32, 0x0);  // JITU_ANIL 3/23/2013 
	kc_serdes_pcie_wr(pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG1__ADDR, data32);


//CMU_REG2
	data32 = kc_serdes_pcie_rd(pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG2__ADDR);
  	data32 =  FIELD_CMU_REG2_PLL_LFRES_SET(data32, 0xa);   // JITU_ANIL  03/25/2013 
   	data32 = FIELD_CMU_REG2_PLL_FBDIV_SET(data32, 0x27);      //100Mhz refclk
  	kc_serdes_pcie_wr(pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG2__ADDR, data32); 
  

//CMU_REG3
	 data32 = kc_serdes_pcie_rd(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG3__ADDR);
 	 data32 = FIELD_CMU_REG3_VCOVARSEL_SET(data32,0x3); //JITU_ANIL 03/23/20113 
	 //ANIL changed this value from 0x15 to 0x16 on 04/15/20113 
	 data32 = FIELD_CMU_REG3_VCO_MOMSEL_INIT_SET(data32,0x10); 
//	 data32 = FIELD_CMU_REG3_VCO_MANMOMSEL_SET(data32,0x10); //JITU_ANIL 03/23/20113 
	 kc_serdes_pcie_wr(pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG3__ADDR, data32);

//CMU_REG26   Added on 3/23/2013
	data32 = kc_serdes_pcie_rd( pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG26__ADDR);
        data32 = FIELD_CMU_REG26_FORCE_PLL_LOCK_SET(data32,0x0); //JITU_ANIL 03/23/20113 
	kc_serdes_pcie_wr(pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG26__ADDR, data32);
//CMU_REG5   
   	data32 = kc_serdes_pcie_rd(pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG5__ADDR);
	data32 = FIELD_CMU_REG5_PLL_LFSMCAP_SET(data32,0x3); 
	data32 = FIELD_CMU_REG5_PLL_LFCAP_SET(data32,0x3); 
	data32 = FIELD_CMU_REG5_PLL_LOCK_RESOLUTION_SET(data32,0x7); //JITU_ANIL 03/23/20113 
	kc_serdes_pcie_wr(pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG5__ADDR, data32);
//CMU_reg6
	data32 = kc_serdes_pcie_rd(pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG6__ADDR);
	data32 = FIELD_CMU_REG6_PLL_VREGTRIM_SET(data32,0x0); 
	data32 = FIELD_CMU_REG6_MAN_PVT_CAL_SET(data32,0x1); 
	kc_serdes_pcie_wr(pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG6__ADDR, data32);
//CMU_reg16
	data32 = kc_serdes_pcie_rd(pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR);
	data32 = FIELD_CMU_REG16_CALIBRATION_DONE_OVERRIDE_SET(data32,0x1); //JITU_ANIL 03/25/20113 
        data32 = FIELD_CMU_REG16_BYPASS_PLL_LOCK_SET(data32,0x1); //JITU_ANIL 03/23/20113 
	data32 = FIELD_CMU_REG16_VCOCAL_WAIT_BTW_CODE_SET(data32,0x4); //JITU_ANIL 03/23/20113 
	kc_serdes_pcie_wr(pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG16__ADDR, data32);
//CMU_reg30
	data32 = kc_serdes_pcie_rd(pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG30__ADDR);
	data32 = FIELD_CMU_REG30_PCIE_MODE_SET(data32,0x0); 
	data32 = FIELD_CMU_REG30_LOCK_COUNT_SET(data32,0x3); //JITU_ANIL 03/23/20113 
	kc_serdes_pcie_wr(pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG30__ADDR, data32);
//CMU_reg31
	kc_serdes_pcie_wr(pcie_base ,KC_CLKMACRO_CMU_REGS_CMU_REG31__ADDR, 0xF);
//CMU_reg32
	data32 = kc_serdes_pcie_rd(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG32__ADDR);
	data32 |= 0x0006 | 0x0180;
	data32 = FIELD_CMU_REG32_PVT_CAL_WAIT_SEL_SET(data32,0x3); //JITU_ANIL 03/23/20113 
	data32 = FIELD_CMU_REG32_IREF_ADJ_SET(data32,0x3); 
	kc_serdes_pcie_wr(pcie_base,KC_CLKMACRO_CMU_REGS_CMU_REG32__ADDR, data32);
//CMU_reg34
//	kc_serdes_pcie_wr( KC_CLKMACRO_CMU_REGS_CMU_REG34__ADDR, 0x2A2A);
	kc_serdes_pcie_wr( pcie_base,KC_CLKMACRO_CMU_REGS_CMU_REG34__ADDR, 0x8d27);
//CMU_reg37
	data32 = kc_serdes_pcie_rd(pcie_base,KC_CLKMACRO_CMU_REGS_CMU_REG37__ADDR);
	kc_serdes_pcie_wr(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG37__ADDR, 0xF00F);
  //data32 = sm_sata_read (SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR);

/*
  apm_in32((pcie_base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR),  &data32 );
  data32 = FIELD_PCIE_CLK_MACRO_REG_I_RESET_B_SET(data32, 0x1);
  data32 =   FIELD_PCIE_CLK_MACRO_REG_I_CUSTOMEROV_SET(data32, 0x0);
  //sm_sata_write (SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR, data32);
  apm_in32((pcie_base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR),  &data32 );
  //apm_out32((pcie_base +  SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR) ,data32);*/
 
  apm_in32((pcie_base +  SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR) , &data32);
       data32 = FIELD_PCIE_CLK_MACRO_REG_I_RESET_B_SET(data32, 0x1);
       data32 =   FIELD_PCIE_CLK_MACRO_REG_I_CUSTOMEROV_SET(data32, 0x0);
       //sm_sata_write (SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR, data32);

       apm_out32((pcie_base +  SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR) ,data32);

for(i=0; i<2;i++)
  mdelay(8);

  

	  rc = 0;
      while ( calib_loop_count<5) {
       //data32 = sm_sata_read (SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR);
#if 0
       apm_in32((pcie_base +  SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR) , &data32);
       data32 = FIELD_PCIE_CLK_MACRO_REG_I_RESET_B_SET(data32, 0x1);
       data32 =   FIELD_PCIE_CLK_MACRO_REG_I_CUSTOMEROV_SET(data32, 0x0);
       //sm_sata_write (SM_ADDR_MAP_PCIE2_CSR_BASE + SM_PCIE_X1_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR, data32);

       apm_out32((pcie_base +  SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR) ,data32);
#endif       
mdelay(8);	
    if(rc) kc_sata45_macro_pdown_force_vco(ctx);
	  rc = kc_sata45_macro_calib_ready_check(ctx);
	  if (rc==0) break;
	  ++calib_loop_count;
      printk(" sata_sm_sds_config() calib_loop_count=%d rc = %d.. \n\r",calib_loop_count,rc);
      }
      if (calib_loop_count==5) goto  end;
 
  i = 10;
       apm_in32((pcie_base +  SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR) , &data32);
  if (FIELD_PCIE_CLK_MACRO_REG_O_PLL_LOCK_RD(data32) && ( i > 0)) {
      printk ("  =============>>>> PLL CLKMACRO LOOKED ...\n");
      udelay(100);
  } 
  else { printk ("  =============>>>> PLL CLKMACRO UN-LOOKED ...\n");
  }

   if (FIELD_PCIE_CLK_MACRO_REG_O_PLL_READY_RD(data32)) {
      printk ("  =============>>>> PLL CLKMACRO READY ...\n");
      

   } 
  else  {  printk ("  =============>>>> PLL CLKMACRO NOT READY ...\n"); }

end:
	return rc;
}



static void apm_sata_set_portphy_cfg(struct apm88xxxx_sata_context *ctx, int channel)
{
	void * mmio = ctx->mmio_base;
        u32 val;
	
	PHYDEBUG("SATA%d.%d port configure mmio 0x%p channel %d",
		ctx->cid, channel, mmio, channel);
        apm_in32(mmio + PORTCFG_ADDR, &val);
        if (channel == 0)
                val = PORTADDR_SET(val, 2);
        else
                val = PORTADDR_SET(val, 3);
        apm_out32_flush(mmio + PORTCFG_ADDR, val);
	/* disable fix rate */
        apm_out32_flush(mmio + PORTPHY1CFG_ADDR, 0x0001fffe);
        apm_out32_flush(mmio + PORTPHY2CFG_ADDR, 0x5018461c);
        apm_out32_flush(mmio + PORTPHY3CFG_ADDR, 0x1c081907);
        apm_out32_flush(mmio + PORTPHY4CFG_ADDR, 0x1c080815);
	apm_in32(mmio + PORTPHY5CFG_ADDR, &val);
	val = FIELD_PORTPHY5CFG_RTCHG_SET(val, 0x300);  // window negotiation 0x800 t0 0x400 
	apm_out32(mmio + PORTPHY5CFG_ADDR, val);
        apm_in32(mmio + PORTAXICFG_ADDR, &val);
	/* enable context management */
        val = FIELD_PORTAXICFG_EN_CONTEXT_SET(val,0x1);
	/* Outstanding */
        val = FIELD_PORTAXICFG_OUTTRANS_SET(val, 0xf);
	apm_out32_flush(mmio + PORTAXICFG_ADDR, val);
}

static void apm_sata_rst_mem_ram(struct apm88xxxx_sata_context *ctx)
{
	void * diagcsr_base = ctx->csr_base + SATA_DIAG_OFFSET;
        int timeout;
        u32 val;

	apm_in32(diagcsr_base + REGSPEC_CFG_MEM_RAM_SHUTDOWN_ADDR, &val);
	if (val == 0) {
		PHYDEBUG("SATA%d already clear memory shutdown", ctx->cid);
		return;
	}

	PHYDEBUG("SATA%d clear memory shutdown", ctx->cid);
	/* SATA controller memory in shutdown. Remove from shutdown. */
        apm_out32_flush(diagcsr_base + REGSPEC_CFG_MEM_RAM_SHUTDOWN_ADDR, 0x00);
	timeout = SATA_RESET_MEM_RAM_TO;
        apm_in32(diagcsr_base + REGSPEC_BLOCK_MEM_RDY_ADDR, &val);
        while (val != 0xFFFFFFFF && timeout-- > 0) {
        	apm_in32(diagcsr_base + REGSPEC_BLOCK_MEM_RDY_ADDR, &val);
		if (val != 0xFFFFFFFF)
			udelay(1);
        }
}

static void apm_sata_clk_rst_pre(struct apm88xxxx_sata_context *ctx)
{
        u32 val;
        void *clkcsr_base = ctx->csr_base + SATA_CLK_OFFSET;

	apm_in32(clkcsr_base + SATACLKENREG_ADDR, &val);
		SATADEBUG("SATA%d controller clock enable", ctx->cid);
		/* disable all reset */
		apm_out32_flush(clkcsr_base + SATASRESETREG_ADDR, 0x00);
		
		/* Enable all resets */
        	apm_out32_flush(clkcsr_base + SATASRESETREG_ADDR, 0xff);
        	
		/* Disable all clks */
        	apm_out32_flush(clkcsr_base + SATACLKENREG_ADDR, 0x00);

        	/* Enable all clks */
	        apm_out32_flush(clkcsr_base + SATACLKENREG_ADDR, 0xf9);  //Note


	        /* Get out of reset for:
		 * 	SDS, CSR
		 *
		 * CORE & MEM are still reset
		 */
        	apm_in32(clkcsr_base + SATASRESETREG_ADDR, &val);
	        if(SATA_MEM_RESET_RD(val)==1){
	        val &= ~(SATA_CSR_RESET_MASK | SATA_SDS_RESET_MASK );
	        val |= SATA_CORE_RESET_MASK | SATA_PCLK_RESET_MASK |
			SATA_PMCLK_RESET_MASK | SATA_MEM_RESET_MASK;
		}
        	apm_out32_flush(clkcsr_base + SATASRESETREG_ADDR, val);
}
static void apm_serdes_reset_rxa_rxd (struct apm88xxxx_sata_context *ctx,
				int channel) {
     u32 val;
	void * csr_base = ctx->csr_base + SATA_SERDES_OFFSET;
    int timeout;
	 apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7__ADDR + channel*0x200,
                &val);
    val =  FIELD_CH0_RXTX_REG7_RESETB_RXD_SET(val,0x0);
    apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7__ADDR + channel*0x200,
                val);

	val =  FIELD_CH0_RXTX_REG7_RESETB_RXA_SET(val,0x0);
     apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7__ADDR + channel*0x200,
                val);
    
    for(timeout=0; timeout<0x800; ++timeout);	

//    printf ("Release RX ANALOG Reset\n\r");
    val =  FIELD_CH0_RXTX_REG7_RESETB_RXA_SET(val,0x1);
    apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7__ADDR + channel*0x200,
                val);

    for(timeout=0; timeout<0x800; ++timeout);	
 //    printf ("Relesae RX DIGITAL Reset\n\r");
	 apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7__ADDR + channel*0x200,
                &val);
    val =  FIELD_CH0_RXTX_REG7_RESETB_RXD_SET(val,0x1);
    apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7__ADDR + channel*0x200,
                val);
    
    for(timeout=0; timeout<0x1000; ++timeout);	

  }
void  sata_sm_deass_pclk_reset(struct apm88xxxx_sata_context *ctx)
{
		u32 val;
        void *clkcsr_base = ctx->csr_base + SATA_CLK_OFFSET;
		apm_in32(clkcsr_base + SATASRESETREG_ADDR, &val);
		val &= ~(SATA_PCLK_RESET_MASK);
		apm_out32(clkcsr_base + SATASRESETREG_ADDR, val);
}
void sata_sm_dis_sds_pmclk_core_reset(struct apm88xxxx_sata_context *ctx)
{
		u32 val;
        void *clkcsr_base = ctx->csr_base + SATA_CLK_OFFSET;
		
	apm_in32(clkcsr_base + SATASRESETREG_ADDR, &val);
	val &= ~(SATA_CORE_RESET_MASK | SATA_PMCLK_RESET_MASK|SATA_SDS_RESET_MASK );
         apm_out32(clkcsr_base + SATASRESETREG_ADDR, val);
	
}
void force_lat_summer_cal (struct apm88xxxx_sata_context *ctx) {

    unsigned int val = 0;
    int         timeout = 0;
  void * csr_base = ctx->csr_base + SATA_SERDES_OFFSET;
  int forcephy=0;
// ***************************
// SUMMER CALIBRATION CH0
// ***************************
     
// SUMMer calib toggle

     apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG127__ADDR ,
                &val);
    val = FIELD_CH0_RXTX_REG127_FORCE_SUM_CAL_START_SET(val,0x1);
    apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG127__ADDR,
                val);
  
    for(timeout=0; timeout<0x40000; ++timeout);
	apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG127__ADDR ,
                &val);
    
    val = FIELD_CH0_RXTX_REG127_FORCE_SUM_CAL_START_SET(val,0x0);
	apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG127__ADDR,
                val);
    for(timeout=0; timeout<0x40000; ++timeout);
   
	apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
               KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG121__ADDR ,
                &val);

   // latch calib toggle
   apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
               KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG127__ADDR ,
                &val);
    val= FIELD_CH0_RXTX_REG127_FORCE_LAT_CAL_START_SET(val,0x1);
    apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG127__ADDR,
                val);
  
    for(timeout=0; timeout<0x40000; ++timeout);

    apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
               KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG127__ADDR ,
                &val);
    val = FIELD_CH0_RXTX_REG127_FORCE_LAT_CAL_START_SET(val,0x0);
     apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG127__ADDR,
                val);
   

// ***************************
// SUMMER CALIBRATION CH1
// ***************************
	apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
               KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG127__ADDR ,
                &val);

    val = FIELD_CH1_RXTX_REG127_FORCE_SUM_CAL_START_SET(val,0x1);
    apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG127__ADDR,
                val);
    
    for(timeout=0; timeout<0x40000; ++timeout);
	
	apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG127__ADDR ,
                &val);
    
    val= FIELD_CH1_RXTX_REG127_FORCE_SUM_CAL_START_SET(val,0x0);
    apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG127__ADDR,
                val);
    for(timeout=0; timeout<0x40000; ++timeout);
	apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG121__ADDR ,
                &val);
    
   
////Latch calib toggle
    apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG127__ADDR ,
                &val);
    val = FIELD_CH1_RXTX_REG127_FORCE_LAT_CAL_START_SET(val,0x1);
    apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG127__ADDR,
                val);
    for(timeout=0; timeout<0x40000; ++timeout);
//
	apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG127__ADDR ,
                &val);
    val = FIELD_CH1_RXTX_REG127_FORCE_LAT_CAL_START_SET(val,0x0);
    apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG127__ADDR,
                val);
 
    for(timeout=0; timeout<0x40000; ++timeout);	//Anil 020813


// Anil request 5-2-2012
// CH 0 

apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG28__ADDR + 0x0,
                0x7);
//Added 18-07-2013

 apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG31__ADDR + 0x0,
                0x2a00);
/*apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG31__ADDR + 0x0,
                0x7e00);*/
// CH 1 
apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG28__ADDR + 0x200,
                0x7);

apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG31__ADDR + 0x200,
                0x2a00);               
/*apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG31__ADDR + 0x200,
                0x7e00);*/ 


//**************
// CH0
//**************
//  removing loopback after calibration cycle
// Added 05/06/2013
		apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG4__ADDR ,
                &val);
	       val = FIELD_CH0_RXTX_REG4_TX_LOOPBACK_BUF_EN_SET(val,0x0);
	     apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
              KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG4__ADDR ,
                val);           
	
// Added on 05/06/2013
		apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7__ADDR  ,
                &val);
	        val = FIELD_CH0_RXTX_REG7_LOOP_BACK_ENA_CTLE_SET(val,0x0);
	      apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
              KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7__ADDR  ,
                val);   
	
//RXTX_REG38
if(forcephy)
{
		 apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
             KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG38__ADDR  ,
                0x2);  
}
else{
		 apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
             KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG38__ADDR  ,
                0x0);   

  }		 
	 

// RXTX_REG39-55
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG39__ADDR  ,
                0xff00); 
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG40__ADDR ,
                0xffff); 
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG41__ADDR ,
                0xffff); 
		
		
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG42__ADDR ,
                0xffff); 
		
		 
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG43__ADDR ,
                0xffff); 
		
		 
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG44__ADDR ,
                0xffff); 
		
		 
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG45__ADDR ,
                0xffff); 
		
		
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG46__ADDR ,
                0xffff); 
		
		
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG47__ADDR ,
                0xfffc); 
		
		 
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG48__ADDR ,
                0x0); 
		
		
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG49__ADDR ,
                0x0); 
		
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG50__ADDR ,
                0x0); 
		
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG51__ADDR ,
                0x0); 
		 
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG52__ADDR ,
                0x0); 
		
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG53__ADDR ,
                0x0); 
		
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG54__ADDR ,
                0x0); 
		
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG55__ADDR ,
                0x0); 
		 

//**************
// CH1
//**************
//  removing loopback after calibration cycle
// Added 05/06/2013
		apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG4__ADDR  ,
                &val);
		val = FIELD_CH1_RXTX_REG4_TX_LOOPBACK_BUF_EN_SET(val,0x0);
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG4__ADDR  ,
                val); 
		
		apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG7__ADDR  ,
                &val);
	     val = FIELD_CH0_RXTX_REG7_LOOP_BACK_ENA_CTLE_SET(val,0x0);
	     
	     apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				 KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG7__ADDR ,
                val);
	if(forcephy)
	{
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG38__ADDR ,
                0x2); 

	}
 	else
	{ 
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG38__ADDR ,
                0x0); 
	}

// RXTX_REG39-55
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG39__ADDR ,
                0xff00); 
 
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG40__ADDR ,
                0xffff); 
		 
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG41__ADDR ,
                0xffff); 
		  
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG42__ADDR ,
                0xffff); 

		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG43__ADDR ,
                0xffff); 
		
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG44__ADDR ,
                0xffff); 

		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG45__ADDR ,
                0xffff); 
	  
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG46__ADDR ,
                0xffff); 
		 
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG47__ADDR ,
                0xfffc); 
		 
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG48__ADDR ,
                0x0); 
	 
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG49__ADDR ,
                0x0); 
	  
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG50__ADDR ,
                0x0); 
		 
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG51__ADDR ,
                0x0); 
	 
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG52__ADDR ,
                0x0); 
	 
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG53__ADDR ,
                0x0); 
		
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG54__ADDR ,
                0x0); 
	 
		apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
				KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG55__ADDR ,
                0x0); 
		 

}
#if 0
static void apm_sata_clk_rst_post(struct apm88xxxx_sata_context *ctx)
{
        u32 val_old;
        u32 val;
        void *clkcsr_base = ctx->csr_base + SATA_CLK_OFFSET;

		SATADEBUG("SATA%d enable SATA core", ctx->cid);
	    apm_in32(clkcsr_base + SATASRESETREG_ADDR, &val);
		val_old = val;
		val &= ~(SATA_CORE_RESET_MASK | SATA_PCLK_RESET_MASK |
		SATA_PMCLK_RESET_MASK | SATA_SDS_RESET_MASK);
		val|=SATA_MEM_RESET_MASK;
		if (val_old != val)
		        apm_out32_flush(clkcsr_base + SATASRESETREG_ADDR, val);
}
#endif
static int apm_serdes_host_sata_select(struct apm88xxxx_sata_context *ctx)
{
		void *muxcsr_base = ctx->csr_base + SATA_ETH_MUX_OFFSET;
        u32 val;

	    PHYDEBUG("SATA%d select SATA MUX", ctx->cid);
        apm_in32(muxcsr_base + SATA_ENET_CONFIG_REG_ADDR, &val);
        val &= ~CFG_SATA_ENET_SELECT_MASK;
        apm_out32_flush(muxcsr_base + SATA_ENET_CONFIG_REG_ADDR, val);
        apm_in32(muxcsr_base + SATA_ENET_CONFIG_REG_ADDR, &val);
        return val & CFG_SATA_ENET_SELECT_MASK ? -1 : 0;
}

static void apm_serdes_validation_CMU_cfg(struct apm88xxxx_sata_context *ctx)
{
	void *csr_base = ctx->csr_base + SATA_SERDES_OFFSET;
        u32 val;

//	 * CMU_reg0[0] = 0      // pciegen3
//	 * CMU_reg0[7:5] = 111	// cal_count_resol
        apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG0__ADDR,
                &val);
        val = FIELD_CMU_REG0_CAL_COUNT_RESOL_SET(val, 0x4);
        apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG0__ADDR,
                val);

//	 * CMU_reg1[13:10] = 0xF = 1111   // pll_cp[3:0]
//	 * CMU_reg1[9:5]   = 0xC = 0 1100 // pll_cp_sel[4:0]
//	 * CMU_reg1[3]     = 0            // pll_manualcal
        apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG1__ADDR,
                &val);
	val= FIELD_CMU_REG1_PLL_CP_SET(val, 0x1);
	val= FIELD_CMU_REG1_PLL_CP_SEL_SET(val, 0x5);
	val = FIELD_CMU_REG1_PLL_MANUALCAL_SET(val, 0x0); //Anil set poly today
	apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG1__ADDR,
                val);

//	 * CMU_reg2[15:14] = 00   pll_refdiv[1:0]
//	 * CMU_reg2[13:5]  = 0x3B = 00011 1011 pll_fbdiv[8:0]
//	 * CMU_reg2[4:1]   = 0x2  = 0010  //pll_lfres[3:0]
        apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG2__ADDR,
                &val);
	val=FIELD_CMU_REG2_PLL_LFRES_SET(val, 0xa);
	val=FIELD_CMU_REG2_PLL_FBDIV_SET(val, FBDIV_VAL); //change  poly 26-04 to 50Mhz
	val=FIELD_CMU_REG2_PLL_REFDIV_SET(val, REFDIV_VAL);//change poly 26-04
	apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG2__ADDR,
                val);

//	 * CMU_reg3[15:10] = 0x1C = 01 1100  vco_manmomsel[5:0]
//	 * CMU_reg3[9:4]   = 0x10 = 01 0000  vco_momsel_init[5:0]
//	 * CMU_reg3[3:0]   = 0x1  = 0001     vcovarsel[3:0]
//	 * CMU_reg3[15:0]  = 0111 0001 0000 0001 = 0x7101
        apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG3__ADDR,
                &val);
	val = FIELD_CMU_REG3_VCOVARSEL_SET(val,0xF);
	val = FIELD_CMU_REG3_VCO_MOMSEL_INIT_SET(val,0x15); //poly new 0x15 to 16 today 
	val = FIELD_CMU_REG3_VCO_MANMOMSEL_SET(val,0x15);//poly new to again today
        apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG3__ADDR,
                val);
//CMU_REG26

	  apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG26__ADDR,
                &val);
	 val = FIELD_CMU_REG26_FORCE_PLL_LOCK_SET(val,0x0);
	 apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG26__ADDR,
                val);

//	 * CMU_reg5[15:14] = 00  pll_lfsmcap[1:0]
//	 * CMU_reg5[13:12] = 00  pll_lfcap[1:0]
	apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG5__ADDR,
                &val);
	val = FIELD_CMU_REG5_PLL_LFSMCAP_SET(val,0x3);
	val = FIELD_CMU_REG5_PLL_LFCAP_SET(val,0x3);
	val = FIELD_CMU_REG5_PLL_LOCK_RESOLUTION_SET(val,0x7);
        apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG5__ADDR,
                val);

//	 * CMU_reg6[10:9] = 00 pll_vregtrim[1:0]
        apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG6__ADDR,
                &val);
	val= FIELD_CMU_REG6_PLL_VREGTRIM_SET(val,0x0);
	val= FIELD_CMU_REG6_MAN_PVT_CAL_SET(val , 0x1); //poly 26-04
        apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG6__ADDR,
                val);

	 //* CMU_reg9[3]   = 1  pll_post_divby2
        apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG9__ADDR,
                &val);
	val= FIELD_CMU_REG9_TX_WORD_MODE_CH1_SET(val,0x3);
	val= FIELD_CMU_REG9_TX_WORD_MODE_CH0_SET(val,0x3);
	val= FIELD_CMU_REG9_PLL_POST_DIVBY2_SET(val,0x1);
        apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG9__ADDR,
                val);


	/*
	 * CMU_reg16[4:2] = 111   vcocal_wait_btw_code[2:0]
	 */
        apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR,
                &val);
	val = FIELD_CMU_REG16_CALIBRATION_DONE_OVERRIDE_SET(val,0x1);
	val = FIELD_CMU_REG16_BYPASS_PLL_LOCK_SET(val,0x1);
	val = FIELD_CMU_REG16_VCOCAL_WAIT_BTW_CODE_SET(val,0x4);
        apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR,
                val);

	/*
	 * CMU_reg30[3] = 0   pciegen3
	 * CMU_reg30[2:1] = 11  lock_count[1:0]
	 */
        apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG30__ADDR,
                &val);
	val = FIELD_CMU_REG30_PCIE_MODE_SET(val,0x0);
	val = FIELD_CMU_REG30_LOCK_COUNT_SET(val,0x3);
        apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG30__ADDR,
                val);

	/*
	 * CMU_reg31[3:0] = 1111 los_override_ch0-ch3
	 */
        apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG31__ADDR,
                0xF);

	/*
	 * CMU_reg32[2:1] = 11  pvt_cal_wait_sel[1:0]
	 * CMU_reg32[8:7] = 11  iref_adj[1:0]
	 */
	apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG32__ADDR,
                &val);
	val |= 0x0006 | 0x0180;
	val = FIELD_CMU_REG32_PVT_CAL_WAIT_SEL_SET(val,0x3);
	val = FIELD_CMU_REG32_IREF_ADJ_SET(val,0x3);
        apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG32__ADDR,
                val);

	/*
	 * CMU_reg34[15:12] = 0010
	 * CMU_reg34[11:8]  = 1010  vco_cal_vth_hi_max[3:0]
	 * CMU_reg34[7:4]   = 0010  vco_cal_vth_lo_min[3:0]
	 * CMU_reg34[3:0]   = 1010  vco_cal_vth_lo_max[3:0]
	 */
        apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG34__ADDR,
                0x8d27); //0x2A2A //anil

	/*
	 * CMU_reg37[15:12] = 1111  CTLE_cal_done_ovr[3:0]
	 * CMU_reg37[3:0]   = 1111  FT_search_done_ovr[3:0]
	 */
        apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG37__ADDR,
                0xF00F);
}

static void apm_serdes_validation_rxtx_cfg(struct apm88xxxx_sata_context *ctx,
					int gen_sel)
{
	void *csr_base = ctx->csr_base + SATA_SERDES_OFFSET;
        u32 val;
        u32 reg;
        int i;
	int channel;
	int forcephy =0;

	for (channel = 0; channel < 2; channel++) {
//Added 18=07-2013
	if(forcephy){
		apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                         KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG38__ADDR + channel*0x200,
                        0x0042); 
		apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                         KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG38__ADDR + channel*0x200,
                        0x0043); 
	}
		
	else {
		 // RXTX REG38      
                 apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                         KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG38__ADDR + channel*0x200,
                        0x0040);      
                        
                  apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                         KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG38__ADDR + channel*0x200,
                        0x0041);
	}  
	        /*
 		 * rxtx_reg147[15:0] =666666    STMC_OVERRIDE[15:0]
		 */
		/* Write to CH0_RXTX_REG147 value 0x6 */
		apm88xxxx_sds_wr_op(csr_base,
			SATA_ENET_SDS_IND_CMD_REG_ADDR,
			SATA_ENET_SDS_IND_WDATA_REG_ADDR,
			KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG147__ADDR + channel*0x200,
			0x6);

                 /*
                 * rxtx_reg0[15:11] = 0x10        CTLE_EQ_HR[4:0]
                 * rxtx_reg0[10:6]  = 0x10        CTLE_EQ_QR[4:0]
                 * rxtx_reg0[5:1]   = 0x10        CTLE_EQ_FR[4:0]
                 */
                apm88xxxx_sds_rd_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG0__ADDR + channel*0x200,
                        &val);
                val = FIELD_CH0_RXTX_REG0_CTLE_EQ_HR_SET(val, 0x10);
		val = FIELD_CH0_RXTX_REG0_CTLE_EQ_QR_SET(val, 0x10);
		val = FIELD_CH0_RXTX_REG0_CTLE_EQ_FR_SET(val, 0x10);
                apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG0__ADDR + channel*0x200,
                        val);

                 /*
                 * rxtx_reg1[15:12] = 0x7   rxacvcm[3:0]
                 * rxtx_reg1[11:7]  = 0x1C  CTLE_EQ[4:0]
                 */
                // 0111_1110
                apm88xxxx_sds_rd_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG1__ADDR + channel*0x200,
                        &val);
             	val= FIELD_CH0_RXTX_REG1_RXACVCM_SET(val, 0x7);
		val= FIELD_CH0_RXTX_REG1_CTLE_EQ_SET(val,CTLE_EQ);  //change 0x9 to 0xd //Anil
                apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG1__ADDR + channel*0x200,
                        val);

                /*
                 * rxtx_reg2[14]  = 0  'Resetb_term'
                 * rxtx_reg2[12]  = 1 (default) Resetb_TXD
                 * rxtx_reg2[11]  = 0 (default) tx_fifo_ena
                 * rxtx_reg2[10]  = 0 (default) tx_inv
                 * rxtx_reg2[8]   = 1   vtt_ena
                 *
                 * Termination to Ground
                 * rxtx_reg2[7:6] = 1  vtt_sel
                 *
                 * rxtx_reg2[5]   = 1  tx_fifo_ena
                 */
                apm88xxxx_sds_rd_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG2__ADDR + channel*0x200,
                        &val);
        	val = FIELD_CH0_RXTX_REG2_VTT_ENA_SET(val, 0x1);
		val = FIELD_CH0_RXTX_REG2_VTT_SEL_SET(val, 0x1);
		val = FIELD_CH0_RXTX_REG2_TX_FIFO_ENA_SET(val, 0x1);
                apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG2__ADDR + channel*0x200,
                        val);

		
                /*
                 * rxtx_reg4[15:14] = 2 for GEN 1 (Quarter rate)   That the controller handling
                 * 					= 1 for GEN 2 (Half rate)
                 * 					= 0 for GEN 3 (Full rate)
                 * rxtx_reg4[13:11] = 3
                 * rxtx_reg4[10:8]  = 4 (default) TX_PRBS_sel[2:0]
                 * rxtx_reg4[6]  	= 0     tx_loopback_buf_en
                 */
                apm88xxxx_sds_rd_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG4__ADDR + channel*0x200,
                        &val);
		//force 6G data rate
		if(forcephy)
		{
		  val=FIELD_CH0_RXTX_REG4_TX_DATA_RATE_SET(val,0);
		}
              	val = FIELD_CH0_RXTX_REG4_TX_WORD_MODE_SET(val,0x3);
              	val = FIELD_CH1_RXTX_REG4_TX_LOOPBACK_BUF_EN_SET(val,0x1);
                apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG4__ADDR + channel*0x200,
                        val);

                /*
                 * rxtx_reg5[15:11] = 6   tx_cn1[4:0]
                 * rxtx_reg5[10:5]  = 4   tx_cp1[5:0]
                 * rxtx_reg5[4:0]   = 0   tx_cn2[4:0]
                 */
                apm88xxxx_sds_rd_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG5__ADDR + channel*0x200,
                        &val);
	        val =FIELD_CH0_RXTX_REG5_TX_CN1_SET(val, 0x0);
		val =FIELD_CH0_RXTX_REG5_TX_CP1_SET(val,0xF); //Anil
		val =FIELD_CH0_RXTX_REG5_TX_CN2_SET(val, 0x0);
                apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG5__ADDR + channel*0x200,
                        val);
                /*
                 * rxtx_reg6[10:7]  = 8   txamp_cntl[3:0]
                 * rxtx_reg6[6]  = 1    txamp_ena
                 * rxtx_reg6[3] = 0
                 * rxtx_reg6[1] = 0
                 * rxtx_reg6[0] = 0   rx_bist_errcnt_rd
                 */
                apm88xxxx_sds_rd_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG6__ADDR + channel*0x200,
                        &val);
            	val  = FIELD_CH0_RXTX_REG6_TXAMP_CNTL_SET(val, 0xf); // per port Txamp oxf to 0x9
		val = FIELD_CH0_RXTX_REG6_TXAMP_ENA_SET(val,0x1);
		val = FIELD_CH0_RXTX_REG6_TX_IDLE_SET (val,0x0);
		val = FIELD_CH0_RXTX_REG6_RX_BIST_RESYNC_SET(val,0x0);
		val = FIELD_CH0_RXTX_REG6_RX_BIST_ERRCNT_RD_SET(val,0x0);
                apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG6__ADDR + channel*0x200,
                        val);
                /*
                 * rxtx_reg7[14]    = 0  Loop_back_ena_ctle
                 * rxtx_reg7[13:11] = 3  RX_word_mode[2:0]
                 *
                 * RX Rate divider select
                 * rxtx_reg7[10:9] 	= 2 for GEN 1 (Quarter rate)
                 * 			= 1 for GEN 2 (Half rate)
                 * 			= 0 for GEN 3 (Full rate)
                 *
                 * rxtx_reg7[8]  	= 1  resetb_rxd
                 * rxtx_reg7[6]  	= 0  st_ena_rx
                 *
                 * RX Word modes select (20bit)
                 * rxtx_reg7[5:3]  	= 4 RX_PRBS_sel[2:0]
                 */
                apm88xxxx_sds_rd_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7__ADDR + channel*0x200,
                        &val);
		//Adding 18=07-2013
		if(forcephy)
		{
		val=FIELD_CH0_RXTX_REG7_RX_DATA_RATE_SET(val, 0);
		}
               	val = FIELD_CH0_RXTX_REG7_BIST_ENA_RX_SET(val,0x0);
               	val = FIELD_CH0_RXTX_REG7_LOOP_BACK_ENA_CTLE_SET(val,0x1);
		val = FIELD_CH0_RXTX_REG7_RX_WORD_MODE_SET(val,0x3);
                apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7__ADDR + channel*0x200,
                        val);

                /*
                 * rxtx_reg8[14] = 1    CDR_Loop_ena
                 * rxtx_reg8[11] = 0    cdr_bypass_rxlos
                 * rxtx_reg8[9]  = 0    SSC_enable
                 * rxtx_reg8[8]  = 1    sd_disable => set to 0 sata working => Tao changed 31/1/2013
                 */
                apm88xxxx_sds_rd_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG8__ADDR + channel*0x200,
                        &val);
                val =FIELD_CH0_RXTX_REG8_CDR_LOOP_ENA_SET(val,0x1);
		val =FIELD_CH0_RXTX_REG8_CDR_BYPASS_RXLOS_SET(val,0x0);
		val =FIELD_CH0_RXTX_REG8_SSC_ENABLE_SET(val, 0x1) ; //change 0x0 to 0x1
		val = FIELD_CH0_RXTX_REG8_SD_DISABLE_SET(val,0x0);
		val = FIELD_CH0_RXTX_REG8_SD_VREF_SET(val, 0x4);
                apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG8__ADDR + channel*0x200,
                        val);

                /*
                 * rxtx_reg11[15:11] = 0    phase_adjust_limit[4:0]
                 */
                apm88xxxx_sds_rd_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG11__ADDR + channel*0x200,
                        &val);
               val = FIELD_CH0_RXTX_REG11_PHASE_ADJUST_LIMIT_SET(val,0x0);
                apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG11__ADDR + channel*0x200,
                        val);
                /*
                 * rxtx_reg12[13] = 1  Latch_off_ena
                 * rxtx_reg12[11] = 0  rx_inv
                 * rxtx_reg12[2]  = 0  sumos_enable
                 * rxtx_reg12[1]  = 0  rx_det_term_enable
                 */
                apm88xxxx_sds_rd_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG12__ADDR + channel*0x200,
                        &val);
                val = FIELD_CH0_RXTX_REG12_LATCH_OFF_ENA_SET(val,0x1);
		val = FIELD_CH0_RXTX_REG12_SUMOS_ENABLE_SET(val,0x0);
		val = FIELD_CH0_RXTX_REG12_RX_DET_TERM_ENABLE_SET(val,0x0);
                apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                         KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG12__ADDR + channel*0x200,
                        val);

                 /*
                 * rxtx_reg26[3] = 1   blwc_ena
                 */
                apm88xxxx_sds_rd_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG26__ADDR + channel*0x200,
                        &val);
              val = FIELD_CH0_RXTX_REG26_PERIOD_ERROR_LATCH_SET(val,0x0);
	      val = FIELD_CH0_RXTX_REG26_BLWC_ENA_SET (val,0x1); //Anil 30 
                apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                         KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG26__ADDR + channel*0x200,
                        val);

                 /*
                 * rxtx_reg28[15:0] = 0xFFFF   DFE_tap_ena[15:0]
                 */
                apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                         KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG28__ADDR + channel*0x200,
                        0x0000); //Anil 
                        
                 apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                         KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG31__ADDR + channel*0x200,
                        0x0000);      
                  
                                  
                   
                  //RXTX REG39-55 
		for (i = 0; i < 17; i++) {
	               apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                         KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG39__ADDR  + channel*0x200 + i*2 ,
                        0x0000);          
					}
 
                /*
                 * Speed Select for Different Data Standards.
                 * rxtx_reg61[13:10]  	= 2 for GEN 1SPD_sel_cdr[3:0]
                 * 			= 4 for GEN 2
                 * 			= 7 for GEN 3
                 * Reset bert logic (bert_resetb)
                 * rxtx_reg61[5] = 0   bert_resetb
                 *
                 * rxtx_reg61[3] = 0
                 *
                 * Pll Select for RX/TX
                 * rxtx_reg61[15] = Don't Care ???rx_hsls_pll_SELECT
                 * rxtx_reg61[14] = Don't Care ???
                 */
                apm88xxxx_sds_rd_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG61__ADDR + channel*0x200,
                        &val);
                
			
           		val = FIELD_CH0_RXTX_REG61_ISCAN_INBERT_SET(val,0x1);
			val = FIELD_CH0_RXTX_REG61_SPD_SEL_CDR_SET(val,SPD_SEL);//poly 26-04 //4 to 5
			//Adding 18-07-2013
			val=FIELD_CH0_RXTX_REG61_LOADFREQ_SHIFT_SET(val,0x0);
 			val = FIELD_CH0_RXTX_REG61_EYE_COUNT_WIDTH_SEL_SET(val,0x0);
                       
                 apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                         KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG61__ADDR + channel*0x200,
                        val);

                /* Added by Anil on 02/17/2013 */
                apm88xxxx_sds_rd_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG62__ADDR + channel*0x200,
                        &val);
          	val = FIELD_CH0_RXTX_REG62_PERIOD_H1_QLATCH_SET(val,0x0);
                apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                         KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG62__ADDR + channel*0x200,
                        val);

                /*
                 * rxtx_reg81-89[15:11] = 0xE
                 * rxtx_reg81-89[10:6] 	= 0xE
                 * rxtx_reg81-89[5:1] 	= 0xE
                 */
                for (i = 0; i < 9; i++) {
                     reg = KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG81__ADDR + channel*0x200 + i*2;
                        apm88xxxx_sds_rd_op(csr_base,
                                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                                reg,
                                &val);
                        val= FIELD_CH0_RXTX_REG89_MU_TH7_SET(val,0xe);
                        val= FIELD_CH0_RXTX_REG89_MU_TH8_SET(val,0xe);
                        val= FIELD_CH0_RXTX_REG89_MU_TH9_SET(val,0xe);
                        apm88xxxx_sds_wr_op(csr_base,
                                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                                reg,
                                val);
                }

                /*
                 * rxtx_reg96-98[15:11] = 0x10
                 * rxtx_reg96-98[10:6]	= 0x10
                 * rxtx_reg96-98[5:1] 	= 0x10
                 */
                for (i = 0; i < 3; i++) {
                        reg = KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG96__ADDR + i*2 + channel*0x200;
                        apm88xxxx_sds_rd_op(csr_base,
                                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                                reg,
                                &val);
                         val = FIELD_CH0_RXTX_REG96_MU_FREQ1_SET(val,0x10);
                        val = FIELD_CH0_RXTX_REG96_MU_FREQ2_SET(val,0x10);
                        val = FIELD_CH0_RXTX_REG96_MU_FREQ3_SET(val,0x10);
                        apm88xxxx_sds_wr_op(csr_base,
                                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                                reg,
                                val);
                }

                /*
                 * rxtx_reg99-101[15:11] = 0x7
                 * rxtx_reg99-101[10:6]	 = 0x7
                 * rxtx_reg99-101[5:1] 	 = 0x7
                 */
                for (i = 0; i < 3; i++) {
                        reg = KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG99__ADDR + i*2 + channel*0x200;
                        apm88xxxx_sds_rd_op(csr_base,
                                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                                reg,
                                &val);
                       	val = FIELD_CH0_RXTX_REG99_MU_PHASE1_SET(val,0x7);
			val = FIELD_CH0_RXTX_REG99_MU_PHASE2_SET(val,0x7);
			val = FIELD_CH0_RXTX_REG99_MU_PHASE3_SET(val,0x7);
                        apm88xxxx_sds_wr_op(csr_base,
                                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                                reg,
                                val);
                }

                /*
                 * rxtx_reg102[6:5] = 2  freqloop_limit[1:0]
                 */
                apm88xxxx_sds_rd_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG102__ADDR + channel*0x200,
                        &val);
			val = FIELD_CH0_RXTX_REG102_FREQLOOP_LIMIT_SET(val,0x0);//change 0x2 to 0x0
                apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG102__ADDR + channel*0x200,
                        val);
                 
                 
                 //change Anil        
                 apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG114__ADDR + channel*0x200,
                        0xffe0);       

                /*
                 * rxtx_reg125[15:9] = 0xA  pq_reg[6:0]
                 * Manual phase program mode
                 * rxtx_reg125[1] = 1  phz_manual
                 */
                apm88xxxx_sds_rd_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG125__ADDR + channel*0x200,
                        &val);
#if 0
                val = FIELD_CH0_RXTX_REG125_PQ_REG_SET(val,0xa); // poly new
		val = FIELD_CH0_RXTX_REG125_PHZ_MANUAL_SET(val,0x1);
#else
                val = FIELD_CH0_RXTX_REG125_SIGN_PQ_SET(val,0x0);  //Anil to 0 30th
               val = FIELD_CH0_RXTX_REG125_PQ_REG_SET(val,PQ_REG); //Anil change to 3 //Anil to 0 30th

		val = FIELD_CH0_RXTX_REG125_PHZ_MANUAL_SET(val,0x1);
#endif
                apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG125__ADDR + channel*0x200,
                        val);

                // * rxtx_reg127[3] = 0 latch_man_cal_ena
                apm88xxxx_sds_rd_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG127__ADDR + channel*0x200,
                        &val);
                val = FIELD_CH0_RXTX_REG127_LATCH_MAN_CAL_ENA_SET(val,0x0);
                apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG127__ADDR + channel*0x200,
                        val);

                //   * rxtx_reg128[3:2] = 3 latch_cal_wait_sel[1:0]
                apm88xxxx_sds_rd_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG128__ADDR + channel*0x200,
                        &val);
               	val = FIELD_CH0_RXTX_REG128_LATCH_CAL_WAIT_SEL_SET(val,0x3); //chnage poly
                apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG128__ADDR + channel*0x200,
                        val);

                /*
                 * rxtx_reg145[15:14] 	= 3   rxdfe_config[1:0]
                 * rxtx_reg145[0] 		= 0
                 */
                apm88xxxx_sds_rd_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG145__ADDR + channel*0x200,
                        &val);
                val = FIELD_CH0_RXTX_REG145_RXDFE_CONFIG_SET(val,0x3);
		val  = FIELD_CH0_RXTX_REG145_TX_IDLE_SATA_SET(val,0x0);
		val  = FIELD_CH0_RXTX_REG145_RXES_ENA_SET(val,0x1); 
		val  = FIELD_CH0_RXTX_REG145_RXVWES_LATENA_SET(val,0x1); 
                apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG145__ADDR + channel*0x200,
                        val);

                /*
                 * rxtx_reg148-151[15:0] = 0xFFFF
                 */
                for (i = 0; i < 4; i++) {
                        reg = KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG148__ADDR + i*2 + channel*0x200;
                        apm88xxxx_sds_wr_op(csr_base,
                                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                                reg,
                                0xFFFF);
                        apm88xxxx_sds_rd_op(csr_base,
                                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                                reg,
                                &val);
                }
	}
}

static int serdes_calib_ready_check(struct apm88xxxx_sata_context  *ctx)
{
	  u32 val;
      int loopcount=5;
      unsigned int mask = 0;
	  void *csr_base = ctx->csr_base;
      void *csr_serdes_base = csr_base + SATA_SERDES_OFFSET;
    
	/* 4. relasase serdes main reset */
	apm_out32_flush(csr_serdes_base + SATA_ENET_SDS_RST_CTL_ADDR,
			0x000000DF);

        /*
        * Anil 021413 start
        * CMU_reg17[15]   = 1               pvt_term_man_ena
        * CMU_reg17[14:8] = 111 1111    pvt_code[6:0]
        * CMU_reg17[7:5]  = 111            Channel select[2:0]
        */
        // TERM CALIBRATION KC_SERDES_CMU_REGS_CMU_REG17__ADDR
        //Poly 26-04 TERM calibration for channel 0
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);
       	 val = FIELD_CMU_REG17_PVT_CODE_R2A_SET(val,0x0d);
   	     val = FIELD_CMU_REG17_RESERVED_7_SET(val,0x0);
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, val);
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);

        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);
        val = FIELD_CMU_REG17_PVT_TERM_MAN_ENA_SET(val,0x1);
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, val);
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);

        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);
        val = FIELD_CMU_REG17_PVT_TERM_MAN_ENA_SET(val,0x0);
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, val);
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);
                
         //Term calibration for channel 1
         
         apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);
       	 val = FIELD_CMU_REG17_PVT_CODE_R2A_SET(val,0x0d);
   	 val = FIELD_CMU_REG17_RESERVED_7_SET(val,0x1);
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, val);
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);

        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);
        val = FIELD_CMU_REG17_PVT_TERM_MAN_ENA_SET(val,0x1);
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, val);
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);

        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);
        val = FIELD_CMU_REG17_PVT_TERM_MAN_ENA_SET(val,0x0);
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, val);
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);   
                
                //end for term calibration both channels    

        // DOWN CALIBRATION for channel zero
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);
        val = FIELD_CMU_REG17_PVT_CODE_R2A_SET(val,0x26);
    	val = FIELD_CMU_REG17_RESERVED_7_SET(val,0x0);
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, val);
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);

        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, &val);
 	val = FIELD_CMU_REG16_PVT_DN_MAN_ENA_SET(val,0x1); //poly set to 0 poly 26-04
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, val);
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, &val);

        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, &val);
	val= FIELD_CMU_REG16_PVT_DN_MAN_ENA_SET(val,0x0); //poly 26-04
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, val);
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, &val);
                
                //Down calibration for channel 1
                  apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);
        val = FIELD_CMU_REG17_PVT_CODE_R2A_SET(val,0x26);
    	val = FIELD_CMU_REG17_RESERVED_7_SET(val,0x1);
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, val);
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);

        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, &val);
 	val = FIELD_CMU_REG16_PVT_DN_MAN_ENA_SET(val,0x1); //poly set to 0 poly 26-04
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, val);
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, &val);

        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, &val);
	val= FIELD_CMU_REG16_PVT_DN_MAN_ENA_SET(val,0x0); //poly 26-04
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, val);
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, &val);
                
                //down calibration done

        // UP CALIBRATION  for channel 0

        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);
        val=FIELD_CMU_REG17_PVT_CODE_R2A_SET(val, 0X28); //poly 26-04
        val=FIELD_CMU_REG17_RESERVED_7_SET(val, 0x0);
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, val);
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);

        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, &val);
        val=FIELD_CMU_REG16_PVT_UP_MAN_ENA_SET(val, 0x1);
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, val);
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, &val);
                
               apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, &val);
        val=FIELD_CMU_REG16_PVT_UP_MAN_ENA_SET(val, 0x0);
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, val);
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, &val);  
                
                
                //up calibration for channel 1
                 apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);
        val=FIELD_CMU_REG17_PVT_CODE_R2A_SET(val, 0X28); //poly 26-04
        val=FIELD_CMU_REG17_RESERVED_7_SET(val, 0x1);
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, val);
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &val);

        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, &val);
        val=FIELD_CMU_REG16_PVT_UP_MAN_ENA_SET(val, 0x1);
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, val);
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, &val);
                
               apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, &val);
        val=FIELD_CMU_REG16_PVT_UP_MAN_ENA_SET(val, 0x0);
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, val);
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR, &val);       
    
        //up calibration end
		mdelay(2000);

        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG7__ADDR, &val);
        PHYDEBUG("SATA%d CMU_REG7 0x%08X", ctx->cid, val);
       
        while(FIELD_CMU_REG7_PLL_CALIB_DONE_RD(val) == 0x0){
			loopcount--;
			udelay(2000);
			apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG7__ADDR, &val);
                if(loopcount==0)
                {
		printk("PLL calibration done not detected\n");
					break;
		}
			
	}
		apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG7__ADDR, &val);

	udelay(1000);
	udelay(1000);	

		
        if (FIELD_CMU_REG7_PLL_CALIB_DONE_RD(val) == 1) {
                PHYDEBUG("SATA%d SERDES PLL calibration done", ctx->cid);
        }
        if (FIELD_CMU_REG7_VCO_CAL_FAIL_RD(val) == 0x0) {
        printk("sata_sm_sds_config() ====>  SERDES CALIB Successfull...\n\r");
		}
		else {
    // Assert SDS reset and recall calib function
		printk("[SATA_ERR] sata_sm_sds_config() ====>  SERDES CALIB FAILED due to VCO FAIL...\n\r");
		return 1;
     }       
        PHYDEBUG("SATA%d Checking TX ready...", ctx->cid);
        /* wait until rx ready for 1 seconds */
        
       //TX ready check 
	
	
        mdelay(10);
	  mask = 0x0300;

    apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG15__ADDR ,&val);
     if ((val & mask) == 0x0300) { 
	 printk("****SERDES TX is ready ******\n");
     }
     else { printk("[SATA_ERROR] : SERDES TX is NOT ready \n");return 1;}
        
	
	return 0;
	}

void serdes_pdown_force_vco( struct apm88xxxx_sata_context *ctx){
	unsigned int val; 
       	void *csr_base = ctx->csr_base;
      	void *csr_serdes_base = csr_base + SATA_SERDES_OFFSET;

	printk("serdes power down VCO \n");
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR ,&val);
	val= FIELD_CMU_REG16_VCOCAL_WAIT_BTW_CODE_SET(val,0x5);
 	 apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG16__ADDR,
                val);

	 apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG0__ADDR ,&val);
	     val=FIELD_CMU_REG0_PDOWN_SET(val,1);
 	 apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG0__ADDR,
                val);

	 udelay(1000);	
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG0__ADDR ,&val);
	    val=FIELD_CMU_REG0_PDOWN_SET(val,0);
	apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG0__ADDR,
                val);
	udelay(1000);
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG32__ADDR ,&val);
	    val=FIELD_CMU_REG32_FORCE_VCOCAL_START_SET(val,1);
	apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG32__ADDR,
                val);
	   val=FIELD_CMU_REG32_FORCE_VCOCAL_START_SET(val,0);
	 apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG32__ADDR,
                val);

}
void sata_tx_ssc_enable(struct apm88xxxx_sata_context *ctx)
{
	void *csr_base = ctx->csr_base;
        void *csr_serdes_base = csr_base + SATA_SERDES_OFFSET;
	unsigned int val;
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG35__ADDR ,&val);
	   val= FIELD_CMU_REG35_PLL_SSC_MOD_SET(val,0x5f);
	 apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG35__ADDR,
                val);

	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG36__ADDR ,&val);

	 val = FIELD_CMU_REG36_PLL_SSC_VSTEP_SET(val,33);  // Gen3 == 33
     	val = FIELD_CMU_REG36_PLL_SSC_EN_SET(val,1);  
     	val = FIELD_CMU_REG36_PLL_SSC_DSMSEL_SET(val,1);
	 apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG36__ADDR,
                val);

	 apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG5__ADDR ,&val);
	 val = FIELD_CMU_REG5_PLL_RESETB_SET(val,0);
	 apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG5__ADDR,
                val);
	 udelay(1000);
	 apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG5__ADDR ,&val);
	 val = FIELD_CMU_REG5_PLL_RESETB_SET(val,1);
	 apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG5__ADDR,
                val);
	apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG32__ADDR ,&val);
	val = FIELD_CMU_REG32_FORCE_VCOCAL_START_SET(val, 1);
	apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG32__ADDR,
                val);
	val =  FIELD_CMU_REG32_FORCE_VCOCAL_START_SET(val, 0);
		apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG32__ADDR,
                val);
}
static int apm_serdes_sata_init(struct apm88xxxx_sata_context *ctx,
				int gen_sel, int clk_type, int rxwclk_inv)
{
        u32 val, ssc_enable=0;
        int calib_loop_count = 0,rc=0;
		void *csr_base = ctx->csr_base;
        void *csr_serdes_base = csr_base + SATA_SERDES_OFFSET;
        void *clkcsr_base = ctx->csr_base + SATA_CLK_OFFSET;
	//Added 18-07-2013
	if((ctx->cid == 2) && ((clk_type == SATA_CLK_INT_DIFF) ||(clk_type == SATA_CLK_INT_SING)))
	{
		sata45_kc_macro_cfg(ctx);
	}

	/* Select SATA mux for SATA port 0 - 3 which shared with SGMII ETH */
        if (ctx->cid < 2) {
                if (apm_serdes_host_sata_select(ctx) != 0) {
                        PHYERROR("SATA%d can not select SATA MUX", ctx->cid);
                        return -1;
                }
        }

        /* Clock reset */
	PHYDEBUG("SATA%d enable clock", ctx->cid);
	apm_sata_clk_rst_pre(ctx);
	//Added 18-07-2013
	
	if((ctx->cid != 2) && ((clk_type == SATA_CLK_INT_DIFF) ||(clk_type == SATA_CLK_INT_SING)))
	{
		kc_macro_cfg(ctx);
	}
	
	apm_out32(csr_serdes_base + SATA_ENET_SDS_RST_CTL_ADDR, 0x00); // Adding poly
	udelay(1000);
        PHYDEBUG("SATA%d reset Serdes", ctx->cid);
	/* 1. Serdes main reset and Controller also under reset */
        apm_out32(csr_serdes_base + SATA_ENET_SDS_RST_CTL_ADDR,
			0x00000020); 

	/* Release all resets except  main reset */
        apm_out32(csr_serdes_base + SATA_ENET_SDS_RST_CTL_ADDR,
                        0x000000DE);

	apm_in32(csr_serdes_base + SATA_ENET_SDS_CTL1_ADDR ,  &val);
	val = CFG_I_SPD_SEL_CDR_OVR1_SET(val , SPD_SEL);//change 4 to 5
	apm_out32(csr_serdes_base + SATA_ENET_SDS_CTL1_ADDR , val);

	PHYDEBUG("SATA%d Setting the customer pin mode", ctx->cid);
        /**
	 * Clear customer pins mode[13:0] = 0
	 * Set customer pins mode[14] = 1
	 */
        apm_in32(csr_serdes_base + SATA_ENET_SDS_CTL0_ADDR, &val);
		val=REGSPEC_CFG_I_CUSTOMER_PIN_MODE0_SET(val ,0x4421);
        apm_out32(csr_serdes_base + SATA_ENET_SDS_CTL0_ADDR, val);
        
        //kc_InitSequence:
//===============
//calib_start_count_stop     : w_State_delay1  = f
//channel_start_count_stop   : w_State_delay2  = 2
//
//kc_InitTXSequence:
//=================
//reset_count_stop           : w_State_delay3 =  2
//ready_count_stop           : w_State_delay9  = 2
//
//kc_InitRXSequence:
//=================
//to_reset_count_stop             : w_State_delay3 = 2
//to_start_ctle_cal_count_stop    : w_State_delay4 = 2
//to_float_tap_src_ena_count_stop : w_State_delay5 = 2
//to_float_tap_src_count_stop     : w_State_delay6 = 3
//to_blwc_ena_count_stop          : w_State_delay7 = 3
//to_ready_count_stop             : w_State_delay8 = 5
//
//4'hf:  // 2.56 ms (assume 10ns clock)
//4'he:  // 1.28 ms
//4'hd:  // 640us
//4'hc:  // 320us
//4'hb:  // 160us
//4'ha:  // 80us
//4'h9:  // 40us
//4'h8:  // 20us
//4'h7:  // 10us
//4'h6:  // 5.12us
//4'h5:  // 2.56us
//4'h4:  // 1.28us
//4'h3:  // 640ns
//4'h2:  // 320ns
//4'h1:  // 160ns
//4'h0:  // 80ns


        /* CMU_REG12 tx ready delay 0x2 */
        apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG12__ADDR,
                &val);
        val = FIELD_CMU_REG12_STATE_DELAY9_SET(val, 0x2);
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG12__ADDR,
                val);
                
         
	
       apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG13__ADDR,
                0xF222);
        apm88xxxx_sds_wr_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG14__ADDR,
                0x2225);       

	if (clk_type == SATA_CLK_EXT_DIFF) {
		apm88xxxx_sds_rd_op(csr_serdes_base,
			SATA_ENET_SDS_IND_CMD_REG_ADDR,
			SATA_ENET_SDS_IND_RDATA_REG_ADDR,
			KC_SERDES_CMU_REGS_CMU_REG0__ADDR,
			&val);
		val =  FIELD_CMU_REG0_PLL_REF_SEL_SET(val, 0x0);
		apm88xxxx_sds_wr_op(csr_serdes_base,
			SATA_ENET_SDS_IND_CMD_REG_ADDR,
			SATA_ENET_SDS_IND_WDATA_REG_ADDR,
			KC_SERDES_CMU_REGS_CMU_REG0__ADDR,
			val);
		apm88xxxx_sds_rd_op(csr_serdes_base,
			SATA_ENET_SDS_IND_CMD_REG_ADDR,
			SATA_ENET_SDS_IND_RDATA_REG_ADDR,
			KC_SERDES_CMU_REGS_CMU_REG1__ADDR,
			&val);
		val =  FIELD_CMU_REG1_REFCLK_CMOS_SEL_SET(val, 0x0);
		apm88xxxx_sds_wr_op(csr_serdes_base,
			SATA_ENET_SDS_IND_CMD_REG_ADDR,
			SATA_ENET_SDS_IND_WDATA_REG_ADDR,
			KC_SERDES_CMU_REGS_CMU_REG1__ADDR,
			val);
		PHYDEBUG("SATA%d Setting REFCLK EXTERNAL DIFF CML0",ctx->cid );
    	}
	else if (clk_type == SATA_CLK_INT_DIFF) {
		apm88xxxx_sds_rd_op(csr_serdes_base,
			SATA_ENET_SDS_IND_CMD_REG_ADDR,
			SATA_ENET_SDS_IND_RDATA_REG_ADDR,
			KC_SERDES_CMU_REGS_CMU_REG0__ADDR,
			&val);
		val =   FIELD_CMU_REG0_PLL_REF_SEL_SET(val, 0x1);
		apm88xxxx_sds_wr_op(csr_serdes_base,
			SATA_ENET_SDS_IND_CMD_REG_ADDR,
			SATA_ENET_SDS_IND_WDATA_REG_ADDR,
			KC_SERDES_CMU_REGS_CMU_REG0__ADDR,
			val);
		apm88xxxx_sds_rd_op(csr_serdes_base,
			SATA_ENET_SDS_IND_CMD_REG_ADDR,
			SATA_ENET_SDS_IND_RDATA_REG_ADDR,
			KC_SERDES_CMU_REGS_CMU_REG1__ADDR,
			&val);
		val =  FIELD_CMU_REG1_REFCLK_CMOS_SEL_SET(val, 0x0);
		apm88xxxx_sds_wr_op(csr_serdes_base,
			SATA_ENET_SDS_IND_CMD_REG_ADDR,
			SATA_ENET_SDS_IND_WDATA_REG_ADDR,
			KC_SERDES_CMU_REGS_CMU_REG1__ADDR,
			val);
	   PHYDEBUG("SATA%d Setting REFCLK INTERNAL DIFF CML1", ctx->cid);
    	} else if (clk_type == SATA_CLK_INT_SING) {
		apm88xxxx_sds_rd_op(csr_serdes_base,
			SATA_ENET_SDS_IND_CMD_REG_ADDR,
			SATA_ENET_SDS_IND_RDATA_REG_ADDR,
			KC_SERDES_CMU_REGS_CMU_REG1__ADDR,
			&val);
		val = FIELD_CMU_REG1_REFCLK_CMOS_SEL_SET(val, 0x1);
		apm88xxxx_sds_wr_op(csr_serdes_base,
			SATA_ENET_SDS_IND_CMD_REG_ADDR,
			SATA_ENET_SDS_IND_WDATA_REG_ADDR,
			KC_SERDES_CMU_REGS_CMU_REG1__ADDR,
			val);
		PHYDEBUG("SATA%d Setting REFCLK INTERNAL CMOS", ctx->cid);
	}
	//Added 18-07-2013
	//SATA4/5 no support for CML1
	if((ctx->cid==2) &&  (clk_type == SATA_CLK_INT_DIFF)){
		apm88xxxx_sds_rd_op(csr_serdes_base,
			SATA_ENET_SDS_IND_CMD_REG_ADDR,
			SATA_ENET_SDS_IND_RDATA_REG_ADDR,
			KC_SERDES_CMU_REGS_CMU_REG1__ADDR,
			&val);
		val = FIELD_CMU_REG1_REFCLK_CMOS_SEL_SET(val, 0x1);
		apm88xxxx_sds_wr_op(csr_serdes_base,
			SATA_ENET_SDS_IND_CMD_REG_ADDR,
			SATA_ENET_SDS_IND_WDATA_REG_ADDR,
			KC_SERDES_CMU_REGS_CMU_REG1__ADDR,
			val);
	}
        /* Setup clock inversion */
	apm88xxxx_sds_rd_op(csr_serdes_base,
		SATA_ENET_SDS_IND_CMD_REG_ADDR,
		SATA_ENET_SDS_IND_RDATA_REG_ADDR,
		KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG13__ADDR, &val);
	val &= ~(rxwclk_inv << 13);
	val |=  (rxwclk_inv << 13);
	apm88xxxx_sds_wr_op(csr_serdes_base,
		SATA_ENET_SDS_IND_CMD_REG_ADDR,
		SATA_ENET_SDS_IND_WDATA_REG_ADDR,
		KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG13__ADDR, val);
	apm88xxxx_sds_rd_op(csr_serdes_base,
		SATA_ENET_SDS_IND_CMD_REG_ADDR,
		SATA_ENET_SDS_IND_RDATA_REG_ADDR,
		KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG13__ADDR, &val);

	apm88xxxx_sds_rd_op(csr_serdes_base,
		SATA_ENET_SDS_IND_CMD_REG_ADDR,
		SATA_ENET_SDS_IND_RDATA_REG_ADDR,
		KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG13__ADDR, &val);
	val &= ~(rxwclk_inv << 13);
	val |=  (rxwclk_inv << 13);
	apm88xxxx_sds_wr_op(csr_serdes_base,
		SATA_ENET_SDS_IND_CMD_REG_ADDR,
		SATA_ENET_SDS_IND_WDATA_REG_ADDR,
		KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG13__ADDR, val);
	apm88xxxx_sds_rd_op(csr_serdes_base,
		SATA_ENET_SDS_IND_CMD_REG_ADDR,
		SATA_ENET_SDS_IND_RDATA_REG_ADDR,
		KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG13__ADDR, &val);

        /* 2. Program serdes registers */
        apm_serdes_validation_CMU_cfg(ctx);
	if(ssc_enable)
	{
		sata_tx_ssc_enable(ctx);
	}
        apm_serdes_validation_rxtx_cfg(ctx, gen_sel);
        /* setup sata gen */
	apm_in32(csr_serdes_base + SATA_ENET_SDS_PCS_CTL0_ADDR, &val);
		val = REGSPEC_CFG_I_RX_WORDMODE0_SET(val,0x3);
		val = REGSPEC_CFG_I_TX_WORDMODE0_SET(val,0x3);
	apm_out32(csr_serdes_base + SATA_ENET_SDS_PCS_CTL0_ADDR, val);
	//calib loop count is reduced to 3 from 5
      while (rc || calib_loop_count<5) {
	  if(rc)serdes_pdown_force_vco(ctx);
	  rc = serdes_calib_ready_check(ctx);
	  if (rc==0) break;
	    ++calib_loop_count;
      }
      if (calib_loop_count==5) 
      {
	   rc=-1;
	  goto  end;
	  }

	 apm_out32_flush(clkcsr_base + SATACLKENREG_ADDR, 0xff); 
	 
	mdelay(3); 
	 sata_sm_dis_sds_pmclk_core_reset(ctx);
	 
	mdelay(3); 
	 
	 sata_sm_deass_pclk_reset(ctx);
	 
	 PHYDEBUG("SATA%d initialized PHY", ctx->cid);
	 return 0;


end :

  return rc;

	
}
#if 0
static void apm_serdes_reset_rxa(struct apm88xxxx_sata_context *ctx,
				int channel)
{
        u32 val;
	void * csr_base = ctx->csr_base + SATA_SERDES_OFFSET;

        /* CH0 RX Reset Analog */
        apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7__ADDR + channel*0x200,
                &val);
        val =  FIELD_CH0_RXTX_REG7_RESETB_RXA_SET(val,0x0);
        apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7__ADDR + channel*0x200,
                val);

        udelay(1000); /* FIXME */

        apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7__ADDR + channel*0x200,
                &val);
        val =  FIELD_CH0_RXTX_REG7_RESETB_RXA_SET(val,0x1);
        apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7__ADDR + channel*0x200,
                val);
}

static void apm_serdes_reset_rxd(struct apm88xxxx_sata_context *ctx,
				int channel){
		u32 val;
	void * csr_base = ctx->csr_base + SATA_SERDES_OFFSET;		
    int timeout;
    apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7__ADDR + channel*0x200,
                &val);
    val =  FIELD_CH0_RXTX_REG7_RESETB_RXD_SET(val,0x0);
    apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7__ADDR + channel*0x200,
                val);
  
    for(timeout=0; timeout<0x800; ++timeout);	
	 apm88xxxx_sds_rd_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
               KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7__ADDR + channel*0x200,
                &val);
    val =  FIELD_CH0_RXTX_REG7_RESETB_RXD_SET(val,0x1);
    apm88xxxx_sds_wr_op(csr_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7__ADDR + channel*0x200,
                val);
    
    for(timeout=0; timeout<0x8000; ++timeout);	

  }


static int apm_serdes_cmu_reset(struct apm88xxxx_sata_context *ctx,
				int channel)
{
	 u32 val;
	void * csr_serdes_base = ctx->csr_base + SATA_SERDES_OFFSET;
	 int rc = 0;
		PHYDEBUG("port_cmu_reset () Check VCO Status\n\r");
     
     apm88xxxx_sds_rd_op(csr_serdes_base,
		SATA_ENET_SDS_IND_CMD_REG_ADDR,
		SATA_ENET_SDS_IND_RDATA_REG_ADDR,
		KC_SERDES_CMU_REGS_CMU_REG7__ADDR, &val);
     if (FIELD_CMU_REG7_VCO_CAL_FAIL_RD(val) == 0x1) { 
        rc = 1;
        PHYDEBUG("port_init_bus () VCO FAILED Resetting CMU \n\r");
        // Asserting CMU reset
        apm88xxxx_sds_rd_op(csr_serdes_base,
		SATA_ENET_SDS_IND_CMD_REG_ADDR,
		SATA_ENET_SDS_IND_RDATA_REG_ADDR,
		KC_SERDES_CMU_REGS_CMU_REG0__ADDR, &val);
		
        val=FIELD_CMU_REG0_RESETB_SET(val,0x0);
        
        apm88xxxx_sds_wr_op(csr_serdes_base,
		SATA_ENET_SDS_IND_CMD_REG_ADDR,
		SATA_ENET_SDS_IND_WDATA_REG_ADDR,
		KC_SERDES_CMU_REGS_CMU_REG0__ADDR, val);
        // Deasserting CMU reset
        
        apm88xxxx_sds_rd_op(csr_serdes_base,
		SATA_ENET_SDS_IND_CMD_REG_ADDR,
		SATA_ENET_SDS_IND_RDATA_REG_ADDR,
		KC_SERDES_CMU_REGS_CMU_REG0__ADDR, &val);
		
        val=FIELD_CMU_REG0_RESETB_SET(val,0x1);
    
        apm88xxxx_sds_wr_op(csr_serdes_base,
		SATA_ENET_SDS_IND_CMD_REG_ADDR,
		SATA_ENET_SDS_IND_WDATA_REG_ADDR,
		KC_SERDES_CMU_REGS_CMU_REG0__ADDR, val);
          mdelay(500);  
      }
      return rc;
	
}

/**
 * Check and clean disparity error
 *
 * Return: 0 	- No disparity error detected
 *         1 	- Disparity error detected and cleaned
 *        -1 	- Invalid port
 */
static int apm_serdes_clean_disparity(struct apm88xxxx_sata_context *ctx, struct ata_port *ap)
{
        u32 val, data32, time ;
	void *mmio = ctx->mmio_base;
        void *csr_base = ctx->csr_base;
        int timeout,rc=0;
	void *port_mmio=ahci_port_base(ap);
	if (csr_base == NULL) {
		SATADEBUG("SATA%d.%d mmio 0x%p invalid",
			ctx->cid, ap->port_no, mmio);
			rc=-1;
			goto end;
		//return -1;
	}
	 apm_in32(port_mmio + PORT_SCR_ERR , &val);
	if(!D0_RD(val)){
		rc=0;
		goto end;
		
	}
        PHYDEBUG("SATA%d.%d check disparity", ctx->cid,ap->port_no);
        /* SERR0_ADDR offset from mmio base, channel mmio will be extracted 100h */
        apm_in32(port_mmio + PORT_SCR_ERR , &val);
	apm_out32(port_mmio + PORT_SCR_ERR , val);
        PHYDEBUG("SATA%d.%d SERR0 0x%08X", ctx->cid, ap->port_no, val);
	mdelay(500); /* FIXME */  // Tao aading 
       apm_in32(port_mmio + PORT_SCR_ERR , &val);
	apm_out32(port_mmio + PORT_SCR_ERR , val);
	// check if Link statemachine is in idle state, if ot wait
      time = 5;
      data32 = readl(mmio + PORTLNKSTAT0_ADDR);   
      printk ("Link State Status before issing RXA ====>  PORTLNKSTAT0 = 0x%x\n\r", data32);
      while (((data32 & 0x3f) != 0x1)  && (time>1)) {
      for(timeout=0; timeout<0x8800; ++timeout); 
      data32 = readl(mmio + PORTLNKSTAT0_ADDR);   
      --time;
      }
      if (time ==0) {rc = 1; goto end;}
        PHYDEBUG("SATA%d.%d SERR0 0x%08X", ctx->cid, ap->port_no, val);
	
	PHYERROR("SATA%d.%d PHY disparity erorr, resetting RXA...",
		ctx->cid,ap->port_no);
	timeout = 5;
	 apm_in32(port_mmio + PORT_SCR_ERR , &val);
        while ((D0_RD(val) || B0_RD(val)) && timeout > 0) {
                apm_serdes_reset_rxa(ctx, ap->port_no);
                timeout--;
                udelay(2000); /* FIXME  */
                data32 = readl(port_mmio + PORT_SCR_STAT);   // port0 SSTS
        if  (((data32 )& 0x00000003) != 0x00000003) {
             data32 = readl(port_mmio + PORT_SCR_STAT);   // port0 SSTS
             printk("[SATA_ERROR] : PORT%d DOWN RESET ANALOG LINK ....  \n\r", ap->port_no);
        }
			udelay(2000);
				apm_in32(port_mmio + PORT_SCR_ERR , &val);
				apm_out32(port_mmio + PORT_SCR_ERR , val);
				
		}
		rc=0;
			apm_in32(port_mmio + PORT_SCR_ERR , &val);
				if(D0_RD(val) || B0_RD(val))
				{
					printk ("port_disparity_clean(loop) ====> Errors Still exist ...QUIT PxSERR = 0x%d\n\r", data32);
					rc=1;
				}
				//Post check of Link Statemachine state
      data32 = readl(mmio + PORTLNKSTAT0_ADDR);   
      printk ("Link State Status when exiting port disparity cleanup loop PORTLNKSTAT0 = 0x%x\n\r", data32);
      if ((data32 & 0x03f) != 0x1) {
      printk ("[SATA_ERROR] Link State Status NOT in IDLE state when exiting port disparity cleanup loop = 0x%x\n\r", data32);
      rc = 1;
      }
        end:
        return rc;
}
#endif
static int apm88xxxx_sata_get_channel(struct ata_host *host,
					struct ata_port *port)
{
        u32 max_ports;
        struct ata_port *cur_port;
        int i;

        max_ports = host->n_ports;

        for (i = 0; i < max_ports; i++) {
                cur_port = host->ports[i];
                if (cur_port == port)
                        return i;
        }

        return -1;
}
#if 0
int apm88xxxx_port_prepare(struct ata_link *link)
{
	struct ata_port *port = link->ap;
        struct ata_host *host = port->host;
	struct apm88xxxx_sata_context *ctx = host->private_data;
	int channel;

	channel = apm88xxxx_sata_get_channel(host, port);
	if (channel < 0 || channel >= MAX_AHCI_CHN_PERCTR)
		return 0;

	return apm_serdes_clean_disparity(ctx,port);
}
#endif
unsigned int apm88xxxx_ahci_read_id(struct ata_device *dev,
					struct ata_taskfile *tf, u16 *id)
{
	unsigned int err_mask;
	struct ata_port *ap=dev->link->ap;
	void *port_mmio=ahci_port_base(ap);
	unsigned int data32=0;
	err_mask =ata_exec_internal(dev, tf, NULL, DMA_FROM_DEVICE,
				     id, sizeof(id[0]) * ATA_ID_WORDS, 0);
	if(err_mask)
		return err_mask;

	
	data32 = readl(port_mmio + PORT_CMD_ISSUE);
	if (data32 == 0x00000000) {     
		writel(0x10, port_mmio + PORT_CMD);
		writel(0x11, port_mmio + PORT_CMD);
	}

	return 0;

}
#if 0
static int apm88xxxx_ahci_hardreset(struct ata_link *link, unsigned int *class,
			  unsigned long deadline)
{
	const unsigned long *timing = sata_ehc_deb_timing(&link->eh_context);
	struct ata_port *ap = link->ap;
	struct ahci_port_priv *pp = ap->private_data;
	u8 *d2h_fis = pp->rx_fis + RX_FIS_D2H_REG;
	struct ata_taskfile tf;
	bool online;
	int rc , rc_rx_ready;
	unsigned int check;
	unsigned int timeout=10;
    struct ata_host *host = ap->host;
	
	struct apm88xxxx_sata_context *ctx = host->private_data;
	u32 val;
	 void *csr_base = ctx->csr_base;
     void *csr_serdes_base = csr_base + SATA_SERDES_OFFSET;
	int channel;
	#define MAX_RETRY_COUNT		3	
	int retry;
	void __iomem *port_mmio;
	int portcmd_saved;


	channel = apm88xxxx_sata_get_channel(host, ap);
	if (channel >= MAX_AHCI_CHN_PERCTR) {
		*class = ATA_DEV_NONE;
                return 0;
	}
	SATADEBUG("SATA%d.%d APM hardreset", ctx->cid, channel);
	
	/* As PORT_CMD_FIS_RX is reset by a hardreset, let save it */
	port_mmio = ahci_port_base(ap);
	portcmd_saved = readl(port_mmio + PORT_CMD);
	SATADEBUG("SATA%d.%d PORT_CMD 0x%08X",
		ctx->cid, channel, portcmd_saved);

	ahci_stop_engine(ap);

	retry = 0;

	/* clear D2H reception area to properly wait for D2H FIS */
	ata_tf_init(link->device, &tf);
	tf.command = 0x80;
	ata_tf_to_fis(&tf, 0, 0, d2h_fis);

	writel(portcmd_saved, port_mmio + PORT_CMD);
	SATADEBUG("SATA%d.%d pre-hardrest PORT_CMD 0x%08X",
		ctx->cid, channel, readl(port_mmio + PORT_CMD));

	rc = sata_link_hardreset(link, timing, deadline, &online,
				 ahci_check_ready);
	// RX ready check 
	
	apm_in32(port_mmio + PORT_SCR_ERR , &val);
	apm_out32(port_mmio + PORT_SCR_ERR , val);
	
	if(rc < 0){
    int mask = 0x0;
     if (ap->port_no == 0) {mask = 0x1;}
     if (ap->port_no == 1) {mask = 0x2;}

     int rxready_loop_count = 0x80000;

     while (rxready_loop_count > 0x1) {
    apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG15__ADDR, &val);
     if ((val & mask) == 0x1) { 
	 printk("SATA%d.%d****SERDES RX0 is ready ******\n", ctx->cid ,ap->port_no);
          for(timeout=0; timeout<0x10000; ++timeout);	
	 break;
     }
     else if ((val & mask) == 0x2) { 
	 printk("SATA%d.%d****SERDES RX1 is ready ******\n", ctx->cid , ap->port_no);
          for(timeout=0; timeout<0x10000; ++timeout);	
	 break;
     }
     else { printk("SATA%d.%d [SATA_ERROR] : SERDES RX is NOT ready \n", ctx->cid , ap->port_no);
	    rc_rx_ready = 1; }
      rxready_loop_count = rxready_loop_count - 0x10000;
     }
    
    timeout=5;
    if(rc_rx_ready ==1){
         while((rc)&& (--timeout > 0) ){
			 check=apm_serdes_cmu_reset(ctx, channel);
			 if(!check){
			 apm_serdes_reset_rxa_rxd(ctx, channel);
			 //apm_serdes_reset_rxa_rxd(ctx, channel);
			}
				rc = sata_link_hardreset(link, timing, deadline, &online,
				 ahci_check_ready);
			}
	}

	// clear all errors
	apm_in32(port_mmio + PORT_SCR_ERR , &val);
	apm_out32(port_mmio + PORT_SCR_ERR , val);
}

	if (online) {
hardreset_retry: 
	val=apm88xxxx_port_prepare(link);
		 if(val==1 && retry <3)
		  {
			retry ++;
		   goto hardreset_retry;
		  }
		  else
		  {
			printk("SATA%d.%d disparity detected more than Max retry \n", ctx->cid , ap->port_no);  
		  }
	}
	SATADEBUG("SATA%d.%d post-hardrest PORT_CMD 0x%08X",
		ctx->cid, channel, readl(port_mmio + PORT_CMD));

	/* As PORT_CMD_FIS_RX is reset by a hardreset, let restore it */
	writel(portcmd_saved, port_mmio + PORT_CMD);
	SATADEBUG("SATA%d.%d restore PORT_CMD 0x%08X",
		ctx->cid, channel, readl(port_mmio + PORT_CMD));

	ahci_start_engine(ap);
	
	if (online)
		*class = ahci_dev_classify(ap);
		
	
	SATADEBUG("SATA%d.%d APM hardreset EXIT rc %d class %u",
		ctx->cid, channel, rc, *class);
	return rc;
}
#endif
//#if 0
static int apm88xxxx_ahci_hardreset(struct ata_link *link, unsigned int *class,
			  unsigned long deadline)
{
	const unsigned long *timing = sata_ehc_deb_timing(&link->eh_context);
	struct ata_port *ap = link->ap;
	struct ahci_port_priv *pp = ap->private_data;
	u8 *d2h_fis = pp->rx_fis + RX_FIS_D2H_REG;
	struct ata_taskfile tf;
	bool online;
	int rc;

    struct ata_host *host = ap->host;

	struct apm88xxxx_sata_context *ctx = host->private_data;
	void *csr_base = ctx->csr_base;
         void *csr_serdes_base = csr_base + SATA_SERDES_OFFSET;
	 int retry=0;
	u32 sstatus , temp;
	#if 0
	    int  rc_rx_ready check, i;
    unsigned int timeout=10;
    #endif
	int channel;
	#define MAX_RETRY_COUNT		3
	
	void __iomem *port_mmio;
	int portcmd_saved;
	u32 portclb_saved;
	u32 portclbhi_saved;
	u32 portrxfis_saved;
	u32 portrxfishi_saved;
	unsigned int val;

	channel = apm88xxxx_sata_get_channel(host, ap);
	if (channel >= MAX_AHCI_CHN_PERCTR) {
		*class = ATA_DEV_NONE;
                return 0;
	}
	SATADEBUG("SATA%d.%d APM hardreset", ctx->cid, channel);

	/* As PORT_CMD_FIS_RX is reset by a hardreset, let save it */
	port_mmio = ahci_port_base(ap);
	portcmd_saved = readl(port_mmio + PORT_CMD);
	portclb_saved=   readl(port_mmio + PORT_LST_ADDR);
	portclbhi_saved=  readl(port_mmio + PORT_LST_ADDR_HI);
	portrxfis_saved=  readl(port_mmio + PORT_FIS_ADDR);
	portrxfishi_saved=readl(port_mmio + PORT_FIS_ADDR_HI);
	SATADEBUG("SATA%d.%d PORT_CMD 0x%08X",
		ctx->cid, channel, portcmd_saved);

	ahci_stop_engine(ap);
	
	
hardreset_retry: 
	/* clear D2H reception area to properly wait for D2H FIS */
	ata_tf_init(link->device, &tf);
	tf.command = 0x80;
	ata_tf_to_fis(&tf, 0, 0, d2h_fis);

	/*writel(portcmd_saved, port_mmio + PORT_CMD);
	SATADEBUG("SATA%d.%d pre-hardrest PORT_CMD 0x%08X",
		ctx->cid, channel, readl(port_mmio + PORT_CMD));*/
		
/*				 int mask = 0x0;
     if (ap->port_no == 0) {mask = 0x1;}
     if (ap->port_no == 1) {mask = 0x2;}

     int rxready_loop_count = 0x80000;

     while (rxready_loop_count > 0x1) {
    apm88xxxx_sds_rd_op(csr_serdes_base,
                SATA_ENET_SDS_IND_CMD_REG_ADDR,
                SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                KC_SERDES_CMU_REGS_CMU_REG15__ADDR, &val);
     if ((val & mask) == 0x1) { 
	 printk("SATA%d.%d****SERDES RX0 is ready ******\n", ctx->cid ,ap->port_no);
          for(timeout=0; timeout<0x10000; ++timeout);	
	 break;
     }
     else if ((val & mask) == 0x2) { 
	 printk("SATA%d.%d****SERDES RX1 is ready ******\n", ctx->cid , ap->port_no);
          for(timeout=0; timeout<0x10000; ++timeout);	
	 break;
     }
     else { printk("SATA%d.%d [SATA_ERROR] : SERDES RX is NOT ready \n", ctx->cid , ap->port_no);
	    rc_rx_ready = 1; }
      rxready_loop_count = rxready_loop_count - 0x10000;
     }
    
    timeout=5;
    if(rc_rx_ready ==1){
         while((rc)&& (--timeout > 0) ){
			 check=apm_serdes_cmu_reset(ctx, channel);
			 if(!check){
			 apm_serdes_reset_rxa_rxd(ctx, channel);
			 //apm_serdes_reset_rxa_rxd(ctx, channel);
				}
				
		}
	}
	*/
	
	rc = sata_link_hardreset(link, timing, deadline, &online,
				 ahci_check_ready);
		// clear all errors
	apm_in32(port_mmio + PORT_SCR_ERR , &val);
	apm_out32(port_mmio + PORT_SCR_ERR , val);

//#if 0
	if (online) {
		sata_scr_read(link, SCR_STATUS, &sstatus);
				temp = (sstatus >> 4) & 0xf;
				
				if(temp == 2) //gen 2
				{
					//should come here if gen2 after retry	
					if(!retry){
					apm_in32(csr_serdes_base + SATA_ENET_SDS_CTL1_ADDR ,  &val);
				//val = CFG_I_SPD_SEL_CDR_OVR1_SET(val , 0x5);//change 4 to 5
				val = CFG_I_SPD_SEL_CDR_OVR1_SET(val , 0x3);//change 4 to 5 for gen 2
					apm_out32(csr_serdes_base + SATA_ENET_SDS_CTL1_ADDR , val);  
				 apm88xxxx_sds_rd_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_RDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG0__ADDR + channel*0x200,
                        &val);
                /*val = FIELD_CH0_RXTX_REG0_CTLE_EQ_HR_SET(val, 0x10);
				val = FIELD_CH0_RXTX_REG0_CTLE_EQ_QR_SET(val, 0x10);
				val = FIELD_CH0_RXTX_REG0_CTLE_EQ_FR_SET(val, 0x10);*/
				val = FIELD_CH0_RXTX_REG0_CTLE_EQ_HR_SET(val, 0x1c); // gen 2
				val = FIELD_CH0_RXTX_REG0_CTLE_EQ_QR_SET(val, 0x1c);
				val = FIELD_CH0_RXTX_REG0_CTLE_EQ_FR_SET(val, 0x1c);
                apm88xxxx_sds_wr_op(csr_base,
                        SATA_ENET_SDS_IND_CMD_REG_ADDR,
                        SATA_ENET_SDS_IND_WDATA_REG_ADDR,
                        KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG0__ADDR + channel*0x200,
                        val);
                       retry ++;
                       goto hardreset_retry;
				  }
				  else
				  {
					  if(retry < 3){//mismatch
					   retry++;
					   goto hardreset_retry;
				  }
				}
                 //apm88xxxx_port_prepare(link) ;
			}
	}
	SATADEBUG("SATA%d.%d post-hardrest PORT_CMD 0x%08X",
		ctx->cid, channel, readl(port_mmio + PORT_CMD));
//#endif		

	/* As PORT_CMD_FIS_RX is reset by a hardreset, let restore it */
	writel(portcmd_saved, port_mmio + PORT_CMD);
	writel(portclb_saved,port_mmio + PORT_LST_ADDR);
	writel(portclbhi_saved ,port_mmio + PORT_LST_ADDR_HI);
	writel(portrxfis_saved ,port_mmio + PORT_FIS_ADDR);
	writel(portrxfishi_saved ,port_mmio + PORT_FIS_ADDR_HI);
	SATADEBUG("SATA%d.%d restore PORT_CMD 0x%08X",
		ctx->cid, channel, readl(port_mmio + PORT_CMD));

	ahci_start_engine(ap);
	//port_mmio = ahci_port_base(ap);
	//portcmd_saved = readl(port_mmio + PORT_CMD);
	apm_sata_rst_mem_ram(ctx);
	if (online)
		*class = ahci_dev_classify(ap);
		/*else if(rc < 0)
	     {
			 while((!online)&& (--timeout > 0) ){
			 check=apm_serdes_cmu_reset(ctx, channel);
			 if(!check){
			 apm_serdes_reset_rxd(ctx, channel);
			 apm_serdes_reset_rxa(ctx, channel);
			}
				rc = sata_link_hardreset(link, timing, deadline, &online,
				 ahci_check_ready);
			}
		}*/	
		//writel(portcmd_saved, port_mmio + PORT_CMD);
	SATADEBUG("SATA%d.%d APM hardreset EXIT rc %d class %u",
		ctx->cid, channel, rc, *class);
	return rc;
}
//#endif
static void apm88xxxx_ahci_host_stop(struct ata_host *host)
{
	struct apm88xxxx_sata_context *hpriv = host->private_data;

	if (!IS_ERR(hpriv->hpriv.clk)) {
		clk_disable_unprepare(hpriv->hpriv.clk);
		clk_put(hpriv->hpriv.clk);
	}
}
#ifdef CONFIG_ARCH_MSLIM
static void apm88xxxx_ahci_sw_activity(struct ata_link *link)
{
	struct ata_port *ap = link->ap;
	struct ahci_port_priv *pp = ap->private_data;
	struct ahci_em_priv *emp = &pp->em_priv[link->pmp];

	if (!(link->flags & ATA_LFLAG_SW_ACTIVITY))
		return;

	emp->activity++;
	if (!timer_pending(&emp->timer))
		mod_timer(&emp->timer, jiffies + msecs_to_jiffies(10));
}

extern void apm_dump_ahci(struct ata_port *ap , unsigned int tag);
static unsigned int apm88xxxx_ahci_qc_issue(struct ata_queued_cmd *qc)
{
	struct ata_port *ap = qc->ap;
	void __iomem *port_mmio = ahci_port_base(ap);
	struct ahci_port_priv *pp = ap->private_data;
	unsigned int i;

	/* Keep track of the currently active link.  It will be used
	 * in completion path to determine whether NCQ phase is in
	 * progress.
	 */
	pp->active_link = qc->dev->link;

	if (qc->tf.protocol == ATA_PROT_NCQ)
		writel(1 << qc->tag, port_mmio + PORT_SCR_ACT);

	if (pp->fbs_enabled && pp->fbs_last_dev != qc->dev->link->pmp) {
		u32 fbs = readl(port_mmio + PORT_FBS);
		fbs &= ~(PORT_FBS_DEV_MASK | PORT_FBS_DEC);
		fbs |= qc->dev->link->pmp << PORT_FBS_DEV_OFFSET;
		writel(fbs, port_mmio + PORT_FBS);
		pp->fbs_last_dev = qc->dev->link->pmp;
	}
	wmb();
	__cpuc_flush_dcache_area((void *)pp->cmd_slot, AHCI_PORT_PRIV_DMA_SZ);
	writel(1 << qc->tag, port_mmio + PORT_CMD_ISSUE);
	apm88xxxx_ahci_sw_activity(qc->dev->link);

	return 0;
}


static void apm88xxxx_ahci_start_fis_rx(struct ata_port *ap)
{
	void __iomem *port_mmio = ahci_port_base(ap);
	struct ahci_host_priv *hpriv = ap->host->private_data;
	struct ahci_port_priv *pp = ap->private_data;
	u32 tmp;
	unsigned long long cmd_slot_dma_addr=mslim_pa_to_iof_axi(pp->cmd_slot_dma);
	unsigned long long rx_fis_dma_addr=mslim_pa_to_iof_axi(pp->rx_fis_dma );
	
	if (hpriv->cap & HOST_CAP_64)
		writel((cmd_slot_dma_addr >> 16) >> 16,
		       port_mmio + PORT_LST_ADDR_HI);
	writel(cmd_slot_dma_addr& 0xffffffff, port_mmio + PORT_LST_ADDR);

	if (hpriv->cap & HOST_CAP_64)
		writel((rx_fis_dma_addr >> 16) >> 16,
		       port_mmio + PORT_FIS_ADDR_HI);
	writel(rx_fis_dma_addr & 0xffffffff, port_mmio + PORT_FIS_ADDR);
	/* enable FIS reception */
	tmp = readl(port_mmio + PORT_CMD);
	tmp |= PORT_CMD_FIS_RX;
	writel(tmp, port_mmio + PORT_CMD);

	/* flush */
	readl(port_mmio + PORT_CMD);
}


static ssize_t apm88xxxx_ahci_transmit_led_message(struct ata_port *ap, u32 state,
					ssize_t size)
{
	struct ahci_host_priv *hpriv = ap->host->private_data;
	struct ahci_port_priv *pp = ap->private_data;
	void __iomem *mmio = hpriv->mmio;
	u32 em_ctl;
	u32 message[] = {0, 0};
	unsigned long flags;
	int pmp;
	struct ahci_em_priv *emp;

	/* get the slot number from the message */
	pmp = (state & EM_MSG_LED_PMP_SLOT) >> 8;
	if (pmp < EM_MAX_SLOTS)
		emp = &pp->em_priv[pmp];
	else
		return -EINVAL;

	spin_lock_irqsave(ap->lock, flags);

	/*
	 * if we are still busy transmitting a previous message,
	 * do not allow
	 */
	em_ctl = readl(mmio + HOST_EM_CTL);
	if (em_ctl & EM_CTL_TM) {
		spin_unlock_irqrestore(ap->lock, flags);
		return -EBUSY;
	}

	if (hpriv->em_msg_type & EM_MSG_TYPE_LED) {
		/*
		 * create message header - this is all zero except for
		 * the message size, which is 4 bytes.
		 */
		message[0] |= (4 << 8);

		/* ignore 0:4 of byte zero, fill in port info yourself */
		message[1] = ((state & ~EM_MSG_LED_HBA_PORT) | ap->port_no);

		/* write message to EM_LOC */
		writel(message[0], mmio + hpriv->em_loc);
		writel(message[1], mmio + hpriv->em_loc+4);

		/*
		 * tell hardware to transmit the message
		 */
		writel(em_ctl | EM_CTL_TM, mmio + HOST_EM_CTL);
	}

	/* save off new led state for port/slot */
	emp->led_state = state;

	spin_unlock_irqrestore(ap->lock, flags);
	return size;
}
static void apm88xxxx_ahci_sw_activity_blink(unsigned long arg)
{
	struct ata_link *link = (struct ata_link *)arg;
	struct ata_port *ap = link->ap;
	struct ahci_port_priv *pp = ap->private_data;
	struct ahci_em_priv *emp = &pp->em_priv[link->pmp];
	unsigned long led_message = emp->led_state;
	u32 activity_led_state;
	unsigned long flags;

	led_message &= EM_MSG_LED_VALUE;
	led_message |= ap->port_no | (link->pmp << 8);

	/* check to see if we've had activity.  If so,
	 * toggle state of LED and reset timer.  If not,
	 * turn LED to desired idle state.
	 */
	spin_lock_irqsave(ap->lock, flags);
	if (emp->saved_activity != emp->activity) {
		emp->saved_activity = emp->activity;
		/* get the current LED state */
		activity_led_state = led_message & EM_MSG_LED_VALUE_ON;

		if (activity_led_state)
			activity_led_state = 0;
		else
			activity_led_state = 1;

		/* clear old state */
		led_message &= ~EM_MSG_LED_VALUE_ACTIVITY;

		/* toggle state */
		led_message |= (activity_led_state << 16);
		mod_timer(&emp->timer, jiffies + msecs_to_jiffies(100));
	} else {
		/* switch to idle */
		led_message &= ~EM_MSG_LED_VALUE_ACTIVITY;
		if (emp->blink_policy == BLINK_OFF)
			led_message |= (1 << 16);
	}
	spin_unlock_irqrestore(ap->lock, flags);
	apm88xxxx_ahci_transmit_led_message(ap, led_message, 4);
}

static void apm88xxxx_ahci_init_sw_activity(struct ata_link *link)
{
	struct ata_port *ap = link->ap;
	struct ahci_port_priv *pp = ap->private_data;
	struct ahci_em_priv *emp = &pp->em_priv[link->pmp];

	/* init activity stats, setup timer */
	emp->saved_activity = emp->activity = 0;
	setup_timer(&emp->timer, apm88xxxx_ahci_sw_activity_blink, (unsigned long)link);

	/* check our blink policy and set flag for link if it's enabled */
	if (emp->blink_policy)
		link->flags |= ATA_LFLAG_SW_ACTIVITY;
}
static void apm88xxxx_ahci_start_port(struct ata_port *ap)
{
	struct ahci_host_priv *hpriv = ap->host->private_data;
	struct ahci_port_priv *pp = ap->private_data;
	struct ata_link *link;
	struct ahci_em_priv *emp;
	ssize_t rc;
	int i;

	/* enable FIS reception */
	apm88xxxx_ahci_start_fis_rx(ap);

	/* enable DMA */
	if (!(hpriv->flags & AHCI_HFLAG_DELAY_ENGINE))
		ahci_start_engine(ap);

	/* turn on LEDs */
	if (ap->flags & ATA_FLAG_EM) {
		ata_for_each_link(link, ap, EDGE) {
			emp = &pp->em_priv[link->pmp];

			/* EM Transmit bit maybe busy during init */
			for (i = 0; i < EM_MAX_RETRY; i++) {
				rc = apm88xxxx_ahci_transmit_led_message(ap,
							       emp->led_state,
							       4);
				if (rc == -EBUSY)
					ata_msleep(ap, 1);
				else
					break;
			}
		}
	}

	if (ap->flags & ATA_FLAG_SW_ACTIVITY)
		ata_for_each_link(link, ap, EDGE)
			apm88xxxx_ahci_init_sw_activity(link);

}
static void apm88xxxx_ahci_power_up(struct ata_port *ap)
{
	struct ahci_host_priv *hpriv = ap->host->private_data;
	void __iomem *port_mmio = ahci_port_base(ap);
	u32 cmd;

	cmd = readl(port_mmio + PORT_CMD) & ~PORT_CMD_ICC_MASK;

	/* spin up device */
	if (hpriv->cap & HOST_CAP_SSS) {
		cmd |= PORT_CMD_SPIN_UP;
		writel(cmd, port_mmio + PORT_CMD);
	}

	/* wake up link */
	writel(cmd | PORT_CMD_ICC_ACTIVE, port_mmio + PORT_CMD);
}
static void apm88xxxx_ahci_enable_fbs(struct ata_port *ap)
{
	struct ahci_port_priv *pp = ap->private_data;
	void __iomem *port_mmio = ahci_port_base(ap);
	u32 fbs;
	int rc;

	if (!pp->fbs_supported)
		return;

	fbs = readl(port_mmio + PORT_FBS);
	if (fbs & PORT_FBS_EN) {
		pp->fbs_enabled = true;
		pp->fbs_last_dev = -1; /* initialization */
		return;
	}

	rc = ahci_stop_engine(ap);
	if (rc)
		return;

	writel(fbs | PORT_FBS_EN, port_mmio + PORT_FBS);
	fbs = readl(port_mmio + PORT_FBS);
	if (fbs & PORT_FBS_EN) {
		dev_info(ap->host->dev, "FBS is enabled\n");
		pp->fbs_enabled = true;
		pp->fbs_last_dev = -1; /* initialization */
	} else
		dev_err(ap->host->dev, "Failed to enable FBS\n");

	ahci_start_engine(ap);
}

static void apm88xxxx_ahci_disable_fbs(struct ata_port *ap)
{
	struct ahci_port_priv *pp = ap->private_data;
	void __iomem *port_mmio = ahci_port_base(ap);
	u32 fbs;
	int rc;

	if (!pp->fbs_supported)
		return;

	fbs = readl(port_mmio + PORT_FBS);
	if ((fbs & PORT_FBS_EN) == 0) {
		pp->fbs_enabled = false;
		return;
	}

	rc = ahci_stop_engine(ap);
	if (rc)
		return;

	writel(fbs & ~PORT_FBS_EN, port_mmio + PORT_FBS);
	fbs = readl(port_mmio + PORT_FBS);
	if (fbs & PORT_FBS_EN)
		dev_err(ap->host->dev, "Failed to disable FBS\n");
	else {
		dev_info(ap->host->dev, "FBS is disabled\n");
		pp->fbs_enabled = false;
	}

	ahci_start_engine(ap);
}

static void apm88xxxx_ahci_pmp_attach(struct ata_port *ap)
{
	void __iomem *port_mmio = ahci_port_base(ap);
	struct ahci_port_priv *pp = ap->private_data;
	u32 cmd;

	cmd = readl(port_mmio + PORT_CMD);
	cmd |= PORT_CMD_PMP;
	writel(cmd, port_mmio + PORT_CMD);

	apm88xxxx_ahci_enable_fbs(ap);

	pp->intr_mask |= PORT_IRQ_BAD_PMP;

	/*
	 * We must not change the port interrupt mask register if the
	 * port is marked frozen, the value in pp->intr_mask will be
	 * restored later when the port is thawed.
	 *
	 * Note that during initialization, the port is marked as
	 * frozen since the irq handler is not yet registered.
	 */
	if (!(ap->pflags & ATA_PFLAG_FROZEN))
		writel(pp->intr_mask, port_mmio + PORT_IRQ_MASK);
}

static void apm88xxxx_ahci_pmp_detach(struct ata_port *ap)
{
	void __iomem *port_mmio = ahci_port_base(ap);
	struct ahci_port_priv *pp = ap->private_data;
	u32 cmd;

	apm88xxxx_ahci_disable_fbs(ap);

	cmd = readl(port_mmio + PORT_CMD);
	cmd &= ~PORT_CMD_PMP;
	writel(cmd, port_mmio + PORT_CMD);

	pp->intr_mask &= ~PORT_IRQ_BAD_PMP;

	/* see comment above in ahci_pmp_attach() */
	if (!(ap->pflags & ATA_PFLAG_FROZEN))
		writel(pp->intr_mask, port_mmio + PORT_IRQ_MASK);
}


int apm88xxxx_ahci_port_resume(struct ata_port *ap)
{
	apm88xxxx_ahci_power_up(ap);
	apm88xxxx_ahci_start_port(ap);

	if (sata_pmp_attached(ap))
		apm88xxxx_ahci_pmp_attach(ap);
	else
		apm88xxxx_ahci_pmp_detach(ap);

	return 0;
}

static int apm88xxxx_ahci_port_start(struct ata_port *ap)
{
	struct ahci_host_priv *hpriv = ap->host->private_data;
	struct device *dev = ap->host->dev;
	struct ahci_port_priv *pp;
	void *mem;
	dma_addr_t mem_dma;
	size_t dma_sz, rx_fis_sz;

	pp = devm_kzalloc(dev, sizeof(*pp), GFP_KERNEL);
	if (!pp) {
		printk("%s: cannot allocate mem for ahci port\n", __func__);
		return -ENOMEM;
	}
	/* check FBS capability */
	if ((hpriv->cap & HOST_CAP_FBS) && sata_pmp_supported(ap)) {
		void __iomem *port_mmio = ahci_port_base(ap);
		u32 cmd = readl(port_mmio + PORT_CMD);
		if (cmd & PORT_CMD_FBSCP)
			pp->fbs_supported = true;
		else if (hpriv->flags & AHCI_HFLAG_YES_FBS) {
			dev_info(dev, "port %d can do FBS, forcing FBSCP\n",
				 ap->port_no);
			pp->fbs_supported = true;
		} else
			dev_warn(dev, "port %d is not capable of FBS\n",
				 ap->port_no);
	}

	if (pp->fbs_supported) {
		dma_sz = AHCI_PORT_PRIV_FBS_DMA_SZ;
		rx_fis_sz = AHCI_RX_FIS_SZ * 16;
	} else {
		dma_sz = AHCI_PORT_PRIV_DMA_SZ;
		rx_fis_sz = AHCI_RX_FIS_SZ;
	}

	mem = dmam_alloc_coherent(dev, dma_sz, &mem_dma, GFP_KERNEL);
	if (!mem) {
		printk("%s: cannot allocate DMA mem size %d for ahci port\n", __func__, dma_sz);
		return -ENOMEM;
	}
	memset(mem, 0, dma_sz);
	
	/*
	 * First item in chunk of DMA memory: 32-slot command table,
	 * 32 bytes each in size
	 */
	pp->cmd_slot = mem;
	pp->cmd_slot_dma = mem_dma;
	mem += AHCI_CMD_SLOT_SZ;
	mem_dma += AHCI_CMD_SLOT_SZ;

	/*
	 * Second item: Received-FIS area
	 */
	pp->rx_fis = mem;
	pp->rx_fis_dma = mem_dma;
	mem += rx_fis_sz;
	mem_dma += rx_fis_sz;

	/*
	 * Third item: data area for storing a single command
	 * and its scatter-gather table
	 */
	pp->cmd_tbl = mem;
	pp->cmd_tbl_dma = mem_dma;
	/*
	 * Save off initial list of interrupts to be enabled.
	 * This could be changed later
	 */
	pp->intr_mask = DEF_PORT_IRQ;

	ap->private_data = pp;

	/* engage engines, captain */
	return apm88xxxx_ahci_port_resume(ap);
}


static unsigned int ahci_sg_dma_address(struct ata_queued_cmd *qc, void *cmd_tbl)
{
	struct scatterlist *sg;
	struct ahci_sg *ahci_sg = cmd_tbl + AHCI_CMD_TBL_HDR_SZ;
	unsigned int si;
	unsigned int i;

	VPRINTK("ENTER\n");
	/*
	 * Next, the S/G list.
	 */
	for_each_sg(qc->sg, sg, qc->n_elem, si) {
		dma_addr_t addr = sg_dma_address(sg);
		unsigned long long dma_addr=mslim_pa_to_iof_axi(addr);
		u32 sg_len = sg_dma_len(sg);
		ahci_sg[si].addr = cpu_to_le32(dma_addr & 0xffffffff);
		ahci_sg[si].addr_hi = cpu_to_le32((dma_addr >> 16) >> 16);			/* print the AXI dump */	
		/*FIX Memory barrier */
		wmb();
		ahci_sg[si].flags_size = cpu_to_le32(sg_len - 1);
		__cpuc_flush_dcache_area((void*)__va(addr),sg_len);
	//__cpuc_flush_dcache_area((void*)&ahci_sg[si], sizeof(ahci_sg[si]));


	}

	return si;
}
void apm88xxxx_ahci_fill_cmd_slot(struct ahci_port_priv *pp, unsigned int tag,
			u32 opts )
{
	dma_addr_t cmd_tbl_dma;
	unsigned int i;

	cmd_tbl_dma = pp->cmd_tbl_dma + tag * AHCI_CMD_TBL_SZ;
	unsigned long long cmd_tbl_dma_addr=mslim_pa_to_iof_axi(cmd_tbl_dma);
	pp->cmd_slot[tag].opts = cpu_to_le32(opts);
	pp->cmd_slot[tag].status = 0;
	pp->cmd_slot[tag].tbl_addr = cpu_to_le32(cmd_tbl_dma_addr & 0xffffffff);
	pp->cmd_slot[tag].tbl_addr_hi = cpu_to_le32((cmd_tbl_dma_addr >> 16) >> 16);
	__cpuc_flush_dcache_area((void *)&pp->cmd_slot[tag], sizeof(pp->cmd_slot[tag]));
}


static void apm88xxxx_ahci_qc_prep(struct ata_queued_cmd *qc)
{
	struct ata_port *ap=qc->ap;
	struct ahci_port_priv *pp = ap->private_data;
	int is_atapi = ata_is_atapi(qc->tf.protocol);
	void *cmd_tbl;
	u32 opts;
	const u32 cmd_fis_len = 5; /* five dwords */
	unsigned int n_elem ;
	/*
	 * Fill in command table information.  First, the header,
	 * a SATA Register - Host to Device command FIS.
	 */
	 
	cmd_tbl = pp->cmd_tbl + qc->tag * AHCI_CMD_TBL_SZ;

	ata_tf_to_fis(&qc->tf, qc->dev->link->pmp, 1, cmd_tbl);
	if (is_atapi) {
		memset(cmd_tbl + AHCI_CMD_TBL_CDB, 0, 32);
		memcpy(cmd_tbl + AHCI_CMD_TBL_CDB, qc->cdb, qc->dev->cdb_len);
	}
	n_elem = 0;
	if (qc->flags & ATA_QCFLAG_DMAMAP)
		n_elem = ahci_sg_dma_address(qc, cmd_tbl);

	 	/*
	 * Fill in command slot information.
	 */
	opts = cmd_fis_len | n_elem << 16 | (qc->dev->link->pmp << 12);
	if (qc->tf.flags & ATA_TFLAG_WRITE)
		opts |= AHCI_CMD_WRITE;
	if (is_atapi)
		opts |= AHCI_CMD_ATAPI | AHCI_CMD_PREFETCH;

	apm88xxxx_ahci_fill_cmd_slot(pp, qc->tag, opts);
}

#endif

static struct ata_port_operations apm88xxxx_ahci_ops = {
	.inherits	= &ahci_ops,
	.hardreset	= apm88xxxx_ahci_hardreset,
	.host_stop	= apm88xxxx_ahci_host_stop,
	.read_id = apm88xxxx_ahci_read_id,
#ifdef CONFIG_ARCH_MSLIM
	.qc_prep =  apm88xxxx_ahci_qc_prep     ,
	.qc_issue = apm88xxxx_ahci_qc_issue  ,
	.port_resume = apm88xxxx_ahci_port_resume,
	.port_start = apm88xxxx_ahci_port_start,
	.pmp_attach = apm88xxxx_ahci_pmp_attach,
	.pmp_detach =apm88xxxx_ahci_pmp_detach,
#endif	
};

static const struct ata_port_info apm88xxxx_ahci_port_info[] = {
	{
		.flags		= AHCI_FLAG_COMMON,
		.pio_mask	= ATA_PIO4,
		.udma_mask	= ATA_UDMA6,
		.port_ops	= &apm88xxxx_ahci_ops,
	},
};

static struct scsi_host_template apm88xxxx_sata_sht = {
	//AHCI_SHT("apm88xxxx-ahci"),
	ATA_NCQ_SHT("apm88xxxx-ahci"),
	.sg_tablesize		= AHCI_MAX_SG,
	.dma_boundary		= AHCI_DMA_BOUNDARY,			
	.shost_attrs		= ahci_shost_attrs,			
	.sdev_attrs		= ahci_sdev_attrs,
	
};

static int apm88xxxx_ahci_init(struct platform_device *op,
		struct resource *serdes_res, struct resource *mmio_res,struct resource *pcie_clk_res,u32 serdes_diff_clk, u32 gen_sel, int cid, int irq)
{
	struct apm88xxxx_sata_context *hpriv;
	struct device *dev = &op->dev;
	struct ata_port_info pi = apm88xxxx_ahci_port_info[0];
	const struct ata_port_info *ppi[] = { &pi, NULL };
	struct ata_host *host;
	int n_ports;
 	int rc = 0;
	int i;
	u32 val;

	hpriv = devm_kzalloc(dev, sizeof(*hpriv), GFP_KERNEL);
	if (!hpriv) {
		dev_err(dev, "can't alloc host context\n");
		return -ENOMEM;
	}
	hpriv->cid = cid;
	hpriv->irq = irq;
#ifndef CONFIG_ARCH_MSLIM
	hpriv->hpriv.clk = clk_get(dev, NULL);
	if (IS_ERR(hpriv->hpriv.clk)) {
		dev_err(dev, "can't get clock\n");
		goto error_clk;
	}
	rc = clk_prepare_enable(hpriv->hpriv.clk);
	if (rc) {
		dev_err(dev, "clock prepare enable failed");
		goto error_enable_clk;
	}
#endif
	hpriv->csr_phys = serdes_res->start;
	hpriv->csr_base = devm_ioremap(dev, serdes_res->start,
					resource_size(serdes_res));
	if (!hpriv->csr_base) {
		dev_err(dev, "can't map PHY CSR resource\n");
		rc  = -ENOMEM;
		goto error;
	}

	hpriv->mmio_phys = mmio_res->start;
	hpriv->mmio_base = devm_ioremap(dev, mmio_res->start,
					resource_size(mmio_res));
	if (!hpriv->mmio_base) {
		dev_err(dev, "can't map MMIO resource\n");
		rc  = -ENOMEM;
		goto error;
	}
	hpriv->hpriv.mmio = hpriv->mmio_base;
	if(cid ==2){
	hpriv->pcie_base=devm_ioremap(dev,pcie_clk_res->start , resource_size(pcie_clk_res));
	//printk("pcie_base after remap 0x%llx\n", hpriv->pcie_base);
	if (!hpriv->pcie_base) {
		dev_err(dev, "can't ma pcie resource\n");
		rc  = -ENOMEM;
		goto error;
	}
	}
	SATADEBUG("SATA%d PHY PAddr 0x%016LX VAddr 0x%p Mmio PAddr 0x%016LX "
		"VAddr 0x%p", cid,
		hpriv->csr_phys, hpriv->csr_base , 
		hpriv->mmio_phys, hpriv->mmio_base);

	rc = apm_serdes_sata_init(hpriv, gen_sel, serdes_diff_clk, 1);
        if (rc != 0) {
                dev_err(dev, "SATA%d PHY initialize failed %d\n", cid, rc);
		rc = -ENODEV;
		goto error;
	}
	force_lat_summer_cal(hpriv);
	apm_serdes_reset_rxa_rxd(hpriv , 0);
	apm_serdes_reset_rxa_rxd(hpriv , 1);
	for (i = 0; i < MAX_AHCI_CHN_PERCTR; i++) {
		apm_sata_set_portphy_cfg(hpriv, i);

        }

	/* Now enable top level interrupt. Otherwise, port interrupt will
           not work. */
	/* AXI disable Mask */
	apm_out32_flush(hpriv->mmio_base + HOST_IRQ_STAT , 0xffffffff);
        apm_out32(hpriv->csr_base + INTSTATUSMASK_ADDR, 0);
	apm_in32(hpriv->csr_base + INTSTATUSMASK_ADDR, &val);
	SATADEBUG("SATA%d top level interrupt mask 0x%X value 0x%08X",
		cid, INTSTATUSMASK_ADDR, val);
	apm_out32_flush(hpriv->csr_base + ERRINTSTATUSMASK_ADDR, 0x0);
	apm_out32_flush(hpriv->csr_base + SATA_SHIM_OFFSET +
			INT_SLV_TMOMASK_ADDR, 0x0);
	/* Enable AXI Interrupt */
	apm_out32(hpriv->csr_base + SLVRDERRATTRIBUTES_ADDR, 0xffffffff);
	apm_out32(hpriv->csr_base + SLVWRERRATTRIBUTES_ADDR, 0xffffffff);
	apm_out32(hpriv->csr_base + MSTRDERRATTRIBUTES_ADDR, 0xffffffff);
	apm_out32(hpriv->csr_base + MSTWRERRATTRIBUTES_ADDR, 0xffffffff);

	/* Enable coherency as Linux uses cache */
#ifndef CONFIG_ARCH_MSLIM	
	apm_in32(hpriv->csr_base + BUSCTLREG_ADDR , &val);
	val= MSTAWAUX_COHERENT_BYPASS_SET(val,0);
	val= MSTARAUX_COHERENT_BYPASS_SET(val,0);
	apm_out32(hpriv->csr_base+ BUSCTLREG_ADDR , val);

        apm_in32(hpriv->csr_base + IOFMSTRWAUX_ADDR, &val);
	val |= (1 << 3);	/* Enable read coherency */
	val |= (1 << 9);	/* Enable write coherency */
        apm_out32_flush(hpriv->csr_base + IOFMSTRWAUX_ADDR, val);
        apm_in32(hpriv->csr_base + IOFMSTRWAUX_ADDR, &val);
	SATADEBUG("SATA%d coherency 0x%X value 0x%08X",
		cid, IOFMSTRWAUX_ADDR, val);
#endif	
	/* Setup AHCI host priv struction */
	ahci_save_initial_config(dev, &hpriv->hpriv, 0, 0);
	/* prepare host */
	if (hpriv->hpriv.cap & HOST_CAP_NCQ)
		pi.flags |= ATA_FLAG_NCQ;
		//pi.flags &=~ATA_FLAG_NCQ;     /// Tao put DMA only.....
	if (hpriv->hpriv.cap & HOST_CAP_PMP)
		pi.flags |= ATA_FLAG_PMP;
		pi.flags&=~ATA_FLAG_PMP;
	//hpriv->hpriv.flags |= AHCI_HFLAG_NO_PMP	; //disable PMP
	ahci_set_em_messages(&hpriv->hpriv, &pi);

	/* CAP.NP sometimes indicate the index of the last enabled
	 * port, at other times, that of the last possible port, so
	 * determining the maximum port number requires looking at
	 * both CAP.NP and port_map.
	 */
	n_ports = max(ahci_nr_ports(hpriv->hpriv.cap),
				fls(hpriv->hpriv.port_map));

	host = ata_host_alloc_pinfo(dev, ppi, n_ports);
	if (!host) {
		dev_err(dev, "can not allocate host pinfo");
		rc = -ENOMEM;
		goto error;
	}

	host->private_data = hpriv;

	if (!(hpriv->hpriv.cap & HOST_CAP_SSS) || ahci_ignore_sss)
		host->flags |= ATA_HOST_PARALLEL_SCAN;
	else
		printk(KERN_INFO
			"ahci: SSS flag set, parallel bus scan disabled\n");

	if (pi.flags & ATA_FLAG_EM)
		ahci_reset_em(host);

	for (i = 0; i < host->n_ports; i++) {
		struct ata_port *ap = host->ports[i];

		ata_port_desc(ap, "mmio %pR", mmio_res);
		ata_port_desc(ap, "port 0x%x", 0x100 + ap->port_no * 0x80);

		/* set enclosure management message type */
		if (ap->flags & ATA_FLAG_EM)
			ap->em_message_type = hpriv->hpriv.em_msg_type;

		/* disabled/not-implemented port */
		if (!(hpriv->hpriv.port_map & (1 << i)))
			ap->ops = &ata_dummy_port_ops;
	}

	rc = ahci_reset_controller(host);
	if (rc)
		goto error;

	ahci_init_controller(host);
	ahci_print_info(host, "APM88xxxx-AHCI");


	/* Take memory ram out of shutdown */
	//apm_sata_rst_mem_ram(hpriv);

	rc = ata_host_activate(host, hpriv->irq, ahci_interrupt, IRQF_SHARED,
			       &apm88xxxx_sata_sht);
	if (rc)
		goto error;

	SATADEBUG("SATA%d PHY initialized", cid);
	return 0;

error:
	if (hpriv->mmio_base) {
		devm_iounmap(dev ,hpriv->mmio_base);
	}
	if(hpriv->csr_base) {
		devm_iounmap(dev ,hpriv->csr_base);	
	}
	
	return rc;
#ifndef CONFIG_ARCH_MSLIM	
error_enable_clk:
	if (!IS_ERR(hpriv->hpriv.clk))
		clk_put(hpriv->hpriv.clk);

error_clk:
	kfree(hpriv);
	return rc;
#endif	
}

static int apm88xxxx_ahci_probe(struct platform_device *op)
{
	struct device_node *np;
	struct resource serdes_res;
	struct resource memio_res;
	struct resource pcie_clk_res;
	struct device *dev = &op->dev;
	const char * clock_name = NULL;
	u32 serdes_diff_clk;
	u32 gen_sel;
        int cid;
	int irq;
	int rc;

	np = op->dev.of_node;
	if (np == NULL) {
		goto setup_acpi;
	}

	/* Check if the entry is disabled */
        if (!of_device_is_available(np))
                return -ENODEV;

	/* Gather resource info */
	rc = of_address_to_resource(np, 0, &memio_res);
	if (rc != 0) {
		dev_err(dev, "no mmio space\n");
		return -EINVAL;
	}
	rc = of_address_to_resource(np, 1, &serdes_res);
	if (rc != 0) {
		dev_err(dev, "no mmio space for PHY\n");
		return -EINVAL;
	}
	irq = irq_of_parse_and_map(np, 0);
	if (irq <= 0) {
		dev_err(dev, "no irq\n");
		return -EINVAL;
	}
	if (of_property_read_u32(np, "gen-sel", &gen_sel) != 0) {
		dev_err(dev, "Can't parse gen-sel from dtb\n");
		return -EINVAL;
	}
	if (of_property_read_u32(np, "serdes-diff-clk",
				&serdes_diff_clk) != 0) {
		dev_err(dev, "Can't parse serdes-diff-clk from dtb\n");
		return -EINVAL;
	}
        if (of_property_read_string(np, "clock-names", &clock_name) != 0) {
		dev_err(dev, "Can't parse clock-names from dtb\n");
		return -EINVAL;
	}
        if (strcmp(clock_name, "eth01clk") == 0)
		cid = 0;
        else if (strcmp(clock_name, "eth23clk") == 0)
                cid = 1;
	else
		cid = 2;
	if(cid == 2)
	{
			rc = of_address_to_resource(np, 2, &pcie_clk_res);
	if (rc != 0) {
		dev_err(dev, "no mmio space for pcie clk\n");
		return -EINVAL;
	}

	}

	/* Require to successfully alloc DMA region */
	op->dev.coherent_dma_mask = DMA_BIT_MASK(64);
        op->dev.dma_mask = &op->dev.coherent_dma_mask;


	/* Now, initialize the driver */
	return apm88xxxx_ahci_init(op, &serdes_res, &memio_res,&pcie_clk_res,
				serdes_diff_clk, gen_sel, cid, irq);

setup_acpi:
	dev_err(dev, "no ACPI configuration support yet!\n");
	return -ENODEV;
}

static int apm88xxxx_ahci_remove(struct platform_device *op)
{
	struct ata_host *host = dev_get_drvdata(&op->dev);
	struct apm88xxxx_sata_context *hpriv = host->private_data;

	SATADEBUG("SATA%d remove", hpriv->cid);
	devm_iounmap(&op->dev, hpriv->csr_base);
	return 0;
}

#ifdef CONFIG_PM_SLEEP_FIXME
static int apm88xxxx_ahci_suspend(struct platform_device *op,
				pm_message_t state)
{
	struct ata_host *host = dev_get_drvdata(&op->dev);
	struct apm88xxxx_sata_context *hpriv = host->private_data;
	void __iomem *mmio = hpriv->mmio_base;
	u32 ctl;
	int rc;

	SATADEBUG("SATA%d suspend", hpriv->cid);

	/*
	 * AHCI spec rev1.1 section 8.3.3:
	 * Software must disable interrupts prior to requesting a
	 * transition of the HBA to D3 state.
	 */
	ctl = readl(mmio + HOST_CTL);
	ctl &= ~HOST_IRQ_EN;
	writel(ctl, mmio + HOST_CTL);
	readl(mmio + HOST_CTL); /* flush */

	rc = ata_host_suspend(host, state);
	if (rc)
		return rc;
#ifdef CONFIG_ARCH_MSLIM
	if (!IS_ERR(hpriv->hpriv.clk))
		clk_disable_unprepare(hpriv->hpriv.clk);
#endif
	return 0;
}

static int apm88xxxx_ahci_resume(struct platform_device *op)
{
	struct ata_host *host = dev_get_drvdata(&op->dev);
	struct apm88xxxx_sata_context *hpriv = host->private_data;
	int rc;

	SATADEBUG("SATA%d resume", hpriv->cid);
#ifndef CONFIG_ARCH_MSLIM
	if (!IS_ERR(hpriv->hpriv.clk)) {
		rc = clk_prepare_enable(hpriv->hpriv.clk);
		if (rc) {
			dev_err(&op->dev, "clock prepare enable failed");
			return rc;
		}
	}
#endif
	if (op->dev.power.power_state.event == PM_EVENT_SUSPEND) {
		rc = ahci_reset_controller(host);
		if (rc)
			goto disable_unprepare_clk;

		ahci_init_controller(host);
	}

	ata_host_resume(host);

	return 0;

disable_unprepare_clk:
#ifndef CONFIG_ARCH_MSLIM
	if (!IS_ERR(hpriv->hpriv.clk))
		clk_disable_unprepare(hpriv->hpriv.clk);
#endif	

	return rc;
}
#endif

#ifdef CONFIG_ARCH_MSLIM
void apm_dump_ahci(struct ata_port *ap, unsigned int tag)
{
	void __iomem *port_mmio = ahci_port_base(ap);
	struct ata_host *host = ap->host;
	struct ahci_host_priv *hpriv = host->private_data;
	void __iomem *mmio = hpriv->mmio;
	struct apm88xxxx_sata_context *hpriv2 = host->private_data;
	void __iomem *csr_base=hpriv2->csr_base;
	unsigned long data32;
	int i;
	
	
	//printk("command 0x%08x PxCI 0x%08x PxSACT 0x%08x \n",command , readl(port_mmio + 0x38), readl(port_mmio + 0x34));
	
	printk(" PxCI 0x%08x PxSACT 0x%08x PxIS 0x%08x PxIE 0x%08x \n", readl(port_mmio + 0x38), readl(port_mmio + 0x34), readl(port_mmio + 0x10), readl(port_mmio + 0x14));
	printk ("SATA_DEBUG : CHECK CLB COMMAND SET UP\n");
	data32 = readl(port_mmio + 0x00  );   // Read CLB
	
        printk (" PxCLB = 0x%08x \n", data32);

	data32=mslim_iof_axi_to_pa(data32);
	data32=data32 + 0x20 *tag;

	//data32=data32 + 0x20 *0x1f;
	for (i =0; i < 4; i++) {
	     dma_addr_t addr_header = (data32 + (i * 4));	
           printk ("address 0x%llx COMMAND_HEADER[%d] = 0x%x \n",mslim_pa_to_iof_axi(addr_header) ,i,readl(__va(data32) +i*4));
         }

	data32 = readl(__va(data32) +2*4);
	data32=mslim_iof_axi_to_pa(data32);
	for (i =0; i < 5; i++) {
	dma_addr_t addr_cfis = (data32 + (i * 4));
           printk ("address 0x%llx CFIS[%d] = 0x%x \n",mslim_pa_to_iof_axi(addr_cfis), i, readl(__va(data32) +i*4));
         }
         
	for (i =0; i < 5; i++) {
		dma_addr_t addr = (data32 + (i * 4) + 0x80);
           printk ("address 0x%llx PRDT[%d] = 0x%x \n",mslim_pa_to_iof_axi(addr) , i, readl(__va(data32) +i*4 + 0x80));
         }

         	data32 = readl(port_mmio + 0x08  );   // Read FB0
	data32=mslim_iof_axi_to_pa(data32);
	for (i =0; i < 5; i++) {
           printk ("D2H[%d] = 0x%x \n", i, readl(__va(data32) +i*4 + 0x40));
	   writel(0x00000000, __va(data32) +i*4 + 0x40);
        }

#if 0
	
	printk("AHCI 0x%08x 0x%08x GHC 0x%08x IS 0x%08x PI 0x%08x \n", 
			readl(mmio + 0x00), readl(mmio + 0x10), readl(mmio + 0x4), readl(mmio + 0x8) , readl(mmio + 0x0c));
    	printk("Port PxIS 0x%08x PxIE 0x%08x PxCI 0x%08x  PxFB0 0x%08x FBU0 0x%08x PxSIG 0x%08x  PxCMD 0x%08x PxSERR 0x%08x \
			PxTFD 0x%08x PxSACT 0x%08x PxCLB0 0x%08x PxSNTF 0x%08x PxFBS 0x%08x \n ", 
		readl(port_mmio + 0x10), readl(port_mmio + 0x14), readl(port_mmio+0x38), readl(port_mmio + 0x8), 
		readl(port_mmio + 0xc), readl(port_mmio + 0x24),readl(port_mmio + 0x18), readl(port_mmio + 0x30), 
		readl(port_mmio + 0x20), readl(port_mmio + 0x34),readl(port_mmio + 0x00), readl(port_mmio + 0x3c),  readl(port_mmio + 0x40) );

#endif 
 #if 0
        printk("ERRINTSTATUS  %x \n", readl(csr_base+0x30));//  writel ( 0x800 ,port_mmio + 0x30 );
	printk ("SLVRDERRATTRIBUTES %x  SLVWRERRATTRIBUTES %x MSTRDERR/ATTRIBUTES %x MSTWRERRATTRIBUTES %x \n",
                 readl(csr_base+0x00), readl(csr_base+0x04), readl(csr_base+0x08), readl(csr_base+0x0c));
        printk ("GLBL_TRANS_ERR %x INT_SLV_TMO %x \n", readl(csr_base+0xd860),  readl(csr_base+0xe00c) );
	writel(0x0, csr_base + 0xd864); // unmasked
	writel(0x0, csr_base + 0xe010); //SM_SLAVE_SHIM_CSR_INT_SLV_TMOMASK
	//port0_link_monitor(ap);

	printk ("SATA_DEBUG : CHECK CLB COMMAND SET UP\n");
	data32 = readl(port_mmio + 0x00  );   // Read CLB
	
        printk (" PxCLB = 0x%08x \n", data32);


	data32=data32 + 0x20 *tag;
	//data32=data32 + 0x20 *0x1f;
	for (i =0; i < 4; i++) {   
           printk ("COMMAND_HEADER[%d] = 0x%x \n", i,readl(__va(data32) +i*4));
         }

	data32 = readl(__va(data32) +2*4);
	for (i =0; i < 5; i++) {
		
           printk ("CFIS[%d] = 0x%x \n", i, readl(__va(data32) +i*4));
         }

	for (i =0; i < 5; i++) {
           printk ("PRDT[%d] = 0x%x \n", i, readl(__va(data32) +i*4 + 0x80));
         }

         	data32 = readl(port_mmio + 0x08  );   // Read FB0
	for (i =0; i < 5; i++) {
           printk ("D2H[%d] = 0x%x \n", i, readl(__va(data32) +i*4 + 0x40));
	   writel(0x00000000, __va(data32) +i*4 + 0x40);
        }
  #endif
}
#endif

static const struct of_device_id apm88xxxx_ahci_of_match[] = {
	{ .compatible = "apm,ceva-ahci", },
	{},
};
MODULE_DEVICE_TABLE(of, apm88xxxx_ahci_of_match);

static struct platform_driver apm88xxxx_ahci_driver = {
	.driver = {
		.name = "APM88xxxx-ahci",
		.owner = THIS_MODULE,
		.of_match_table = apm88xxxx_ahci_of_match,
	},
	.probe = apm88xxxx_ahci_probe,
	.remove = apm88xxxx_ahci_remove,
#ifdef CONFIG_PM_FIXME
	.suspend	= apm88xxxx_ahci_suspend,
	.resume		= apm88xxxx_ahci_resume,
#endif
};
module_platform_driver(apm88xxxx_ahci_driver);

MODULE_DESCRIPTION("APM88xxxx AHCI SATA driver");
MODULE_AUTHOR("Loc Ho <lho@apm.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.01");
