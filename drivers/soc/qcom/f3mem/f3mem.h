/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_F3MEM_H
#define _LINUX_F3MEM_H


#define MEMORY_CMA	1
#define MEMORY_NON_CMA	0
/* The last client (MAX_CLIENTS -1) is for supporting the
 * requests coming on the old idl. As the old idl changes
 * supports only single client, binding the last client
 * to honor request coming on old idl changes are
 * sufficient.
 */
#define MAX_CLIENTS 10
#define F3MEM	0
#define CHECK	0
#define FREE	1

struct mem_blocks {
	/* Client Id information */
	uint32_t client_id;
	/* Peripheral associated with client */
	uint32_t peripheral;
	/* Sequence Id */
	uint32_t sequence_id;
	/* CMA or Non-CMA region */
	uint32_t memory_type;
	/* Guaranteed Memory */
	uint32_t guarantee;
	/* Memory alloted or not */
	uint32_t alloted;
	/* Size required for client */
	uint32_t size;
	/* start address of the memory block reserved by server memory
	 * subsystem to client
	*/
	phys_addr_t phy_addr;
	/* Virtual address for the physical address allocated
	*/
	void *virtual_addr;
};

enum {
	F3MEM_APPS,
	F3MEM_MODEM,
	F3MEM_Q6,
	F3MEM_DSPS,
	F3MEM_WCNSS,
	F3MEM_MODEM_Q6_FW,
	F3MEM_RPM,
	NUM_SMEM_SUBSYSTEMS,
};

struct ramdump_segment_local {
	int client_id;
	unsigned long address;
	void *v_address;
	unsigned long size;
};

int f3mem_alloc(struct device *dev,
					unsigned int block_size,
					struct mem_blocks *pblk);
void f3mem_free(unsigned int block_size,
					struct mem_blocks *pblk);
#endif /* _LINUX_F3MEM_H */
