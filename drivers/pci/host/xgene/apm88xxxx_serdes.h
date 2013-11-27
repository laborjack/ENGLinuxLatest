/**
 * APM88XXXX Koolchip Serdes access APIs
 *
 * Copyright (c) 2013, Applied Micro Circuits Corporation
 * Author: Tanmay Inamdar <tinamdar@apm.com>
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
 */

#ifndef __APM88XXXX_SDS_PROGRAM_H__
#define __APM88XXXX_SDS_PROGRAM_H__

void apm88xxxx_sds_wr_op(void *csr_base, uint32_t ind_cmd_reg_addr,
			uint32_t ind_wdata_reg_addr, uint32_t offset,
			uint32_t data);

void apm88xxxx_sds_rd_op(void *csr_base, uint32_t ind_cmd_reg_addr,
			uint32_t ind_rdata_reg_addr, uint32_t offset,
			uint32_t * data);

void apm88xxxx_pcs_wr_op(void *csr_base, uint32_t ind_cmd_reg_addr,
			uint32_t ind_wdata_reg_addr, uint32_t offset,
			uint32_t data);

void apm88xxxx_pcs_rd_op(void *csr_base, uint32_t ind_cmd_reg_addr,
			uint32_t ind_rdata_reg_addr, uint32_t offset,
			uint32_t * data);

void apm88xxxx_cm_wr_op(void *csr_base, uint32_t ind_cmd_reg_addr,
		       uint32_t ind_wdata_reg_addr, uint32_t offset,
		       uint32_t data);

void apm88xxxx_cm_rd_op(void *csr_base, uint32_t ind_cmd_reg_addr,
		       uint32_t ind_rdata_reg_addr, uint32_t offset,
		       uint32_t * data);

void apm88xxxx_sds2_wr_op(void *csr_base, uint32_t ind_cmd_reg_addr,
			 uint32_t ind_wdata_reg_addr, uint32_t offset,
			 uint32_t data);

void apm88xxxx_sds2_rd_op(void *csr_base, uint32_t ind_cmd_reg_addr,
			 uint32_t ind_rdata_reg_addr, uint32_t offset,
			 uint32_t * data);

#endif /* __APM88XXXX_SDS_PROGRAM_H__ */
