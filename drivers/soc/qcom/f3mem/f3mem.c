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
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/notifier.h>
#include <soc/qcom/subsystem_restart.h>
#include <soc/qcom/subsystem_notif.h>
#include <soc/qcom/ramdump.h>
#include <soc/qcom/msm_qmi_interface.h>
#include <soc/qcom/scm.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <asm/mach/map.h>
#include <mach/msm_iomap.h>
#include "f3mem.h"
#include "kernel_vendor_specific_service_v01.h"

/* Macros */
#define F3MEM_DEV_NAME "f3mem"

#define KERNEL_VS_SERVICE_SVC_ID 0x000000E5
#define KERNEL_VS_SERVICE_INS_ID 1
#define KERNEL_VS_SERVICE_VERS 1

#define MSM_IMEM_BASE              IOMEM(0xFA80E000)       /* 4K */

static struct qmi_handle *f3mem_svc_handle;
static void f3mem_svc_recv_msg(struct work_struct *work);
static DECLARE_DELAYED_WORK(work_recv_msg, f3mem_svc_recv_msg);
static struct workqueue_struct *f3mem_svc_workqueue;

/* Memshare Driver Structure */
struct f3mem_driver {
	struct device *dev;
	struct mutex mem_share;
	struct mutex mem_free;
	struct work_struct f3mem_init_work;
};

/* same as in drivers/base/dma-coherent.c */
struct dma_coherent_mem {
	void		*virt_base;
	dma_addr_t	device_base;
	int		size;
	int		flags;
	unsigned long	*bitmap;
};

static struct f3mem_driver *f3mem_drv;
static void *curr_conn;
static struct mem_blocks memblock[MAX_CLIENTS];
//static int prealloc_client_table[1] = {F3MEM};
static struct msg_desc f3mem_svc_alloc_generic_req_desc = {
	.max_msg_len = MEM_ALLOC_REQ_MAX_MSG_LEN_V01,
	.msg_id = KERNEL_VS_MEM_ALLOC_GENERIC_REQ_MSG_V01,
	.ei_array = kernel_vs_mem_alloc_generic_req_msg_data_v01_ei,
};

static struct msg_desc f3mem_svc_alloc_generic_resp_desc = {
	.max_msg_len = MEM_ALLOC_REQ_MAX_MSG_LEN_V01,
	.msg_id = KERNEL_VS_MEM_ALLOC_GENERIC_RESP_MSG_V01,
	.ei_array = kernel_vs_mem_alloc_generic_resp_msg_data_v01_ei,
};

static struct msg_desc f3mem_svc_free_generic_req_desc = {
	.max_msg_len = MEM_FREE_REQ_MAX_MSG_LEN_V01,
	.msg_id = KERNEL_VS_MEM_FREE_GENERIC_REQ_MSG_V01,
	.ei_array = kernel_vs_mem_free_generic_req_msg_data_v01_ei,
};

static struct msg_desc f3mem_svc_free_generic_resp_desc = {
	.max_msg_len = MEM_FREE_REQ_MAX_MSG_LEN_V01,
	.msg_id = KERNEL_VS_MEM_FREE_GENERIC_RESP_MSG_V01,
	.ei_array = kernel_vs_mem_free_generic_resp_msg_data_v01_ei,
};

static void *f3mem_ramdump_dev;
static struct ramdump_segment *f3mem_ramdump_segments;
static struct ramdump_segment_local *f3mem_ramdump_segments_local;
static uint32_t num_f3mem_areas;

static void *imem_ramdump_dev;
static struct ramdump_segment *imem_ramdump_segment;

struct restart_notifier_block {
	unsigned processor;
	char *name;
	struct notifier_block nb;
};

static void f3mem_print_ramdump_segments(void);
static void f3mem_add_ramdump_segment(struct mem_blocks *memblock, int client_id);
static void f3mem_remove_ramdump_segment(int client_id);

static int f3mem_restart_notifier_cb(struct notifier_block *this,
				unsigned long code,
				void *data);
static int imem_restart_notifier_cb(struct notifier_block *this,
				unsigned long code,
				void *data);

static struct restart_notifier_block f3mem_restart_notifiers[] = {
	{F3MEM_MODEM, "modem", .nb.notifier_call = f3mem_restart_notifier_cb},
};

static struct restart_notifier_block imem_restart_notifiers[] = {
	{F3MEM_MODEM, "modem", .nb.notifier_call = imem_restart_notifier_cb},
};


static int check_client(int client_id, int proc, int request)
{
	int i = 0;
	int found = KERNEL_VS_MEM_CLIENT_INVALID;
	for (i = 0; i < MAX_CLIENTS; i++) {
		if (memblock[i].client_id == client_id &&
			memblock[i].peripheral == proc) {
			found = i;
			break;
		}
	}
	if ((found == KERNEL_VS_MEM_CLIENT_INVALID) && !request) {
		pr_info("No registered client, adding a new client\n");
		/* Add a new client */
		for (i = 0; i < MAX_CLIENTS; i++) {
			if (memblock[i].client_id == KERNEL_VS_MEM_CLIENT_INVALID) {
				memblock[i].client_id = client_id;
				memblock[i].alloted = 0;
				memblock[i].guarantee = 0;
				memblock[i].peripheral = proc;
				found = i;
				break;
			}
		}
	}

	return found;
}

void f3mem_free_client(int id)
{
	memblock[id].size = 0;
	memblock[id].phy_addr = 0;
	memblock[id].virtual_addr = 0;
	memblock[id].alloted = 0;
	memblock[id].client_id = KERNEL_VS_MEM_CLIENT_INVALID;
	memblock[id].guarantee = 0;
	memblock[id].peripheral = -1;
	memblock[id].sequence_id = -1;
	memblock[id].memory_type = MEMORY_CMA;

}

void f3mem_free_mem_clients(int proc)
{
	int i;

	pr_info("f3mem: freeing clients\n");

	for (i = 0; i < MAX_CLIENTS; i++) {
		if (memblock[i].peripheral == proc &&
				!memblock[i].guarantee) {
			pr_info("Freeing memory for client id: %d\n",
					memblock[i].client_id);
			dma_free_coherent(f3mem_drv->dev, memblock[i].size,
				memblock[i].virtual_addr, memblock[i].phy_addr);
			f3mem_free_client(i);
		}
	}
}

void f3mem_fill_alloc_response(struct kernel_vs_mem_alloc_generic_resp_msg_v01 *resp,
						int id, int *flag)
{
	resp->sequence_id_valid = 1;
	resp->sequence_id = memblock[id].sequence_id;
	resp->kernel_vs_mem_alloc_addr_info_valid = 1;
	resp->kernel_vs_mem_alloc_addr_info_len = 1;
	resp->kernel_vs_mem_alloc_addr_info[0].phy_addr = memblock[id].phy_addr;
	resp->kernel_vs_mem_alloc_addr_info[0].num_bytes = memblock[id].size;
	if (!*flag) {
		resp->resp.result = QMI_RESULT_SUCCESS_V01;
		resp->resp.error = QMI_ERR_NONE_V01;
	} else {
		resp->resp.result = QMI_RESULT_FAILURE_V01;
		resp->resp.error = QMI_ERR_NO_MEMORY_V01;
	}

}

void f3mem_initialize_client(void)
{
	int i;

	for (i = 0; i < MAX_CLIENTS; i++) {
		memblock[i].alloted = 0;
		memblock[i].size = 0;
		memblock[i].guarantee = 0;
		memblock[i].phy_addr = 0;
		memblock[i].virtual_addr = 0;
		memblock[i].client_id = KERNEL_VS_MEM_CLIENT_INVALID;
		memblock[i].peripheral = -1;
		memblock[i].sequence_id = -1;
		memblock[i].memory_type = MEMORY_CMA;
	}

}

static int f3mem_handle_alloc_generic_req(void *req_h, void *req)
{
	struct kernel_vs_mem_alloc_generic_req_msg_v01 *alloc_req;
	struct kernel_vs_mem_alloc_generic_resp_msg_v01 *alloc_resp;
	int rc, resp = 0;
	int client_id;

	alloc_req = (struct kernel_vs_mem_alloc_generic_req_msg_v01 *)req;
	pr_info("%s: Received Alloc Request\n", __func__);
	pr_info("%s: req->num_bytes = %d\n", __func__, alloc_req->num_bytes);
	mutex_lock(&f3mem_drv->mem_share);
	alloc_resp = kzalloc(sizeof(struct kernel_vs_mem_alloc_generic_resp_msg_v01),
					GFP_KERNEL);
	if (!alloc_resp) {
		pr_err("In %s, error allocating memory to response structure\n",
						__func__);
		mutex_unlock(&f3mem_drv->mem_share);
		return -ENOMEM;
	}
	alloc_resp->resp.result = QMI_RESULT_FAILURE_V01;
	alloc_resp->resp.error = QMI_ERR_NO_MEMORY_V01;
	pr_info("alloc request client id: %d proc _id: %d\n",
			alloc_req->client_id, alloc_req->proc_id);
	client_id = check_client(alloc_req->client_id, alloc_req->proc_id,
								CHECK);
	if (!memblock[client_id].alloted) {
		rc = f3mem_alloc(f3mem_drv->dev, alloc_req->num_bytes,
					&memblock[client_id]);
		if (rc) {
			pr_err("In %s,Unable to allocate memory for requested client\n",
							__func__);
			resp = 1;
		}
		if (!resp) {
			memblock[client_id].alloted = 1;
			memblock[client_id].size = alloc_req->num_bytes;
			memblock[client_id].peripheral = alloc_req->proc_id;

			f3mem_add_ramdump_segment(&memblock[client_id], client_id);
		}
	}
	memblock[client_id].sequence_id = alloc_req->sequence_id;

	f3mem_fill_alloc_response(alloc_resp, client_id, &resp);

	mutex_unlock(&f3mem_drv->mem_share);
	pr_info("alloc_resp.num_bytes :%d, alloc_resp.handle :%lx, alloc_resp.mem_req_result :%lx\n",
			  alloc_resp->kernel_vs_mem_alloc_addr_info[0].num_bytes,
			  (unsigned long int)
			  alloc_resp->kernel_vs_mem_alloc_addr_info[0].phy_addr,
			  (unsigned long int)alloc_resp->resp.result);
	rc = qmi_send_resp_from_cb(f3mem_svc_handle, curr_conn, req_h,
			&f3mem_svc_alloc_generic_resp_desc, alloc_resp,
			sizeof(alloc_resp));

	if (rc < 0)
		pr_err("In %s, Error sending the alloc request: %d\n",
							__func__, rc);

	return rc;
}

static int f3mem_handle_free_generic_req(void *req_h, void *req)
{
	struct kernel_vs_mem_free_generic_req_msg_v01 *free_req;
	struct kernel_vs_mem_free_generic_resp_msg_v01 free_resp;
	int rc;
	int flag = 0;
	uint32_t client_id;

	free_req = (struct kernel_vs_mem_free_generic_req_msg_v01 *)req;
	pr_info("%s: Received Free Request\n", __func__);
	mutex_lock(&f3mem_drv->mem_free);
	memset(&free_resp, 0, sizeof(struct kernel_vs_mem_free_generic_resp_msg_v01));
	free_resp.resp.error = QMI_ERR_INTERNAL_V01;
	free_resp.resp.result = QMI_RESULT_FAILURE_V01;
	pr_info("Client id: %d proc id: %d\n", free_req->client_id,
				free_req->proc_id);
	client_id = check_client(free_req->client_id, free_req->proc_id, FREE);
	if (client_id == KERNEL_VS_MEM_CLIENT_INVALID) {
		pr_err("In %s, Invalid client request to free memory\n",
					__func__);
		flag = 1;
	} else if (!memblock[client_id].guarantee &&
					memblock[client_id].alloted) {
		pr_info("In %s: pblk->virtual_addr :%lx, pblk->phy_addr: %lx\n,size: %d",
				__func__,
				(unsigned long int)
				memblock[client_id].virtual_addr,
				(unsigned long int)memblock[client_id].phy_addr,
				memblock[client_id].size);
		dma_free_coherent(f3mem_drv->dev, memblock[client_id].size,
			memblock[client_id].virtual_addr,
			memblock[client_id].phy_addr);
		f3mem_free_client(client_id);

		f3mem_remove_ramdump_segment(client_id);
	} else {
		pr_err("In %s, Request came for a guaranteed client cannot free up the memory\n",
						__func__);
	}

	if (flag) {
		free_resp.resp.result = QMI_RESULT_FAILURE_V01;
		free_resp.resp.error = QMI_ERR_INVALID_ID_V01;
	} else {
		free_resp.resp.result = QMI_RESULT_SUCCESS_V01;
		free_resp.resp.error = QMI_ERR_NONE_V01;
	}

	mutex_unlock(&f3mem_drv->mem_free);
	rc = qmi_send_resp_from_cb(f3mem_svc_handle, curr_conn, req_h,
		&f3mem_svc_free_generic_resp_desc, &free_resp,
		sizeof(free_resp));

	if (rc < 0)
		pr_err("In %s, Error sending the free request: %d\n",
					__func__, rc);

	return rc;
}
static int f3mem_svc_connect_cb(struct qmi_handle *handle,
			       void *conn_h)
{
	if (f3mem_svc_handle != handle || !conn_h)
		return -EINVAL;
	if (curr_conn) {
		pr_err("%s: Service is busy\n", __func__);
		return -EBUSY;
	}
	curr_conn = conn_h;
	return 0;
}

static int f3mem_svc_disconnect_cb(struct qmi_handle *handle,
				  void *conn_h)
{
	if (f3mem_svc_handle != handle || curr_conn != conn_h) {
		return -EINVAL;
	}
	curr_conn = NULL;
	return 0;
}

static int f3mem_svc_req_desc_cb(unsigned int msg_id,
				struct msg_desc **req_desc)
{
	int rc;

	pr_info("f3mem: In %s\n", __func__);
	switch (msg_id) {
	case KERNEL_VS_MEM_ALLOC_GENERIC_REQ_MSG_V01:
		*req_desc = &f3mem_svc_alloc_generic_req_desc;
		rc = sizeof(struct kernel_vs_mem_alloc_generic_req_msg_v01);
		break;

	case KERNEL_VS_MEM_FREE_GENERIC_REQ_MSG_V01:
		*req_desc = &f3mem_svc_free_generic_req_desc;
		rc = sizeof(struct kernel_vs_mem_free_generic_req_msg_v01);
		break;

	default:
		rc = -ENOTSUPP;
		break;
	}
	return rc;
}

static int f3mem_svc_req_cb(struct qmi_handle *handle, void *conn_h,
			void *req_h, unsigned int msg_id, void *req)
{
	int rc;

	pr_info("f3mem: In %s\n", __func__);
	if (f3mem_svc_handle != handle || curr_conn != conn_h)
		return -EINVAL;

	switch (msg_id) {
	case KERNEL_VS_MEM_ALLOC_GENERIC_REQ_MSG_V01:
		rc = f3mem_handle_alloc_generic_req(req_h, req);
		break;

	case KERNEL_VS_MEM_FREE_GENERIC_REQ_MSG_V01:
		rc = f3mem_handle_free_generic_req(req_h, req);
		break;

	default:
		rc = -ENOTSUPP;
		break;
	}
	return rc;
}

static void f3mem_svc_recv_msg(struct work_struct *work)
{
	int rc;

	pr_info("f3mem: In %s\n", __func__);
	do {
		pr_info("%s: Notified about a Receive Event\n", __func__);
	} while ((rc = qmi_recv_msg(f3mem_svc_handle)) == 0);

	if (rc != -ENOMSG)
		pr_err("%s: Error receiving message\n", __func__);
}

static void qmi_f3mem_svc_ntfy(struct qmi_handle *handle,
		enum qmi_event_type event, void *priv)
{
	pr_info("f3mem: In %s\n", __func__);
	switch (event) {
	case QMI_RECV_MSG:
		queue_delayed_work(f3mem_svc_workqueue,
				   &work_recv_msg, 0);
		break;
	default:
		break;
	}
}

static struct qmi_svc_ops_options f3mem_svc_ops_options = {
	.version = 1,
	.service_id = KERNEL_VS_SERVICE_SVC_ID,
	.service_vers = KERNEL_VS_SERVICE_VERS,
	.service_ins = KERNEL_VS_SERVICE_INS_ID,
	.connect_cb = f3mem_svc_connect_cb,
	.disconnect_cb = f3mem_svc_disconnect_cb,
	.req_desc_cb = f3mem_svc_req_desc_cb,
	.req_cb = f3mem_svc_req_cb,
};

int f3mem_alloc(struct device *dev,
					unsigned int block_size,
					struct mem_blocks *pblk)
{

	int ret;

	pr_info("%s: f3mem_alloc called", __func__);
	if (!pblk) {
		pr_err("%s: Failed to alloc\n", __func__);
		return -ENOMEM;
	}

	pblk->virtual_addr = dma_alloc_coherent(dev, block_size,
						&pblk->phy_addr, GFP_KERNEL);
	if (pblk->virtual_addr == NULL) {
		pr_err("allocation failed, %d\n", block_size);
		ret = -ENOMEM;
		return ret;
	}
	pr_info("pblk->phy_addr :%lx, pblk->virtual_addr %lx\n",
		  (unsigned long int)pblk->phy_addr,
		  (unsigned long int)pblk->virtual_addr);
	return 0;
}

static void f3mem_init_worker(struct work_struct *work)
{
	int rc;

    pr_info("f3mem_init_worker\n");

	f3mem_svc_workqueue =
		create_singlethread_workqueue("f3mem_svc");
	if (!f3mem_svc_workqueue)
		return;

	f3mem_svc_handle = qmi_handle_create(qmi_f3mem_svc_ntfy, NULL);
	if (!f3mem_svc_handle) {
		pr_err("%s: Creating f3mem_svc qmi handle failed\n",
			__func__);
		destroy_workqueue(f3mem_svc_workqueue);
		return;
	}
	rc = qmi_svc_register(f3mem_svc_handle, &f3mem_svc_ops_options);
	if (rc < 0) {
		pr_err("%s: Registering f3mem svc failed %d\n",
			__func__, rc);
		qmi_handle_destroy(f3mem_svc_handle);
		destroy_workqueue(f3mem_svc_workqueue);
		return;
	}
	pr_info("f3mem: f3mem_init successful\n");
}

static int prealloc_f3mem(struct platform_device *pdev)
{
#if 0
	int rc;
    uint32_t size;

    pr_info("prealloc_f3mem\n");

	rc = of_property_read_u32(pdev->dev.of_node, "qcom,guarantee-size",
						&size);
	if (rc) {
		pr_err("In %s, Error reading size of clients\n",
				__func__);
		return rc;
	}

    pr_info("zhaobin: size 0x%lx\n", (unsigned long int)size);
    memblock[F3MEM].peripheral = KERNEL_VS_MEM_PROC_MPSS_V01;
	memblock[F3MEM].size = size;
	memblock[F3MEM].client_id = prealloc_client_table[F3MEM];
	memblock[F3MEM].guarantee = 1;

	rc = f3mem_alloc(f3mem_drv->dev, memblock[F3MEM].size,
					&memblock[F3MEM]);
    
	if (rc) {
		pr_err("In %s, Unable to allocate memory for guaranteed clients\n",
						__func__);
		return rc;
	}
	memblock[F3MEM].alloted = 1;

    f3mem_add_ramdump_segment(&memblock[F3MEM], F3MEM);
#endif

	return 0;
}

static int f3mem_probe(struct platform_device *pdev)
{
	int rc;
	struct f3mem_driver *drv;
	int i = 0;

	pr_info("f3mem_probe\n");

	drv = devm_kzalloc(&pdev->dev, sizeof(struct f3mem_driver),
							GFP_KERNEL);

	if (!drv) {
		pr_err("Unable to allocate memory to driver\n");
		return -ENOMEM;
	}

	/* Memory allocation has been done successfully */
	mutex_init(&drv->mem_free);
	mutex_init(&drv->mem_share);

	INIT_WORK(&drv->f3mem_init_work, f3mem_init_worker);
	schedule_work(&drv->f3mem_init_work);

	drv->dev = &pdev->dev;
	f3mem_drv = drv;
	platform_set_drvdata(pdev, f3mem_drv);
	f3mem_initialize_client();

	rc = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);

	if (rc) {
		pr_err("In %s, error populating the devices\n", __func__);
		return rc;
	}

	f3mem_ramdump_segments_local = kcalloc(MAX_CLIENTS,
			sizeof(struct ramdump_segment_local), GFP_KERNEL);
	if (!f3mem_ramdump_segments_local) {
		pr_err("%s: ramdump segment kmalloc failed\n", __func__);
		rc = -ENOMEM;
		return rc;
	}
	for (i = 0; i < MAX_CLIENTS; i++) {
		f3mem_ramdump_segments_local[i].client_id = KERNEL_VS_MEM_CLIENT_INVALID;
	}

	rc = prealloc_f3mem(pdev);
	if (rc) {
		pr_err("In %s, error prealloc_f3mem\n", __func__);
		return rc;
	}

	pr_info("In %s, f3mem probe success\n", __func__);
	return 0;
}

static int f3mem_remove(struct platform_device *pdev)
{
	if (!f3mem_drv)
		return 0;

	qmi_svc_unregister(f3mem_svc_handle);
	flush_workqueue(f3mem_svc_workqueue);
	qmi_handle_destroy(f3mem_svc_handle);
	destroy_workqueue(f3mem_svc_workqueue);

	return 0;
}

static struct of_device_id f3mem_match_table[] = {
	{
		.compatible = "qcom,f3mem",
	},
	{}
};

static struct platform_driver f3mem_pdriver = {
	.probe          = f3mem_probe,
	.remove         = f3mem_remove,
	.driver = {
		.name   = F3MEM_DEV_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = f3mem_match_table,
	},
};

module_platform_driver(f3mem_pdriver);

static void f3mem_print_ramdump_segments(void)
{
    int i = 0;

    for (i = 0; i < MAX_CLIENTS; i++) {
		if (f3mem_ramdump_segments_local[i].client_id != KERNEL_VS_MEM_CLIENT_INVALID) {
			pr_info("f3mem_print_ramdump_segments: client_id %d, address 0x%lx, size 0x%lx, virt_addr 0x%lx\n",
					f3mem_ramdump_segments_local[i].client_id,
					(unsigned long int)f3mem_ramdump_segments_local[i].address,
					f3mem_ramdump_segments_local[i].size,
					(unsigned long int)f3mem_ramdump_segments_local[i].v_address);
		}
	}
}

static void f3mem_add_ramdump_segment(struct mem_blocks *memblock, int client_id)
{
	int i = 0;
	int found = KERNEL_VS_MEM_CLIENT_INVALID;

	pr_info("f3mem_add_ramdump_segment\n");
    
	for (i = 0; i < MAX_CLIENTS; i++) {
		if (f3mem_ramdump_segments_local[i].client_id == client_id) {
			found = i;
			break;
		}
	}
	if (found == KERNEL_VS_MEM_CLIENT_INVALID) {
		pr_info("f3mem_add_ramdump_segment: No registered segment, adding a new one\n");
		/* Add a new client */
		for (i = 0; i < MAX_CLIENTS; i++) {
			if (f3mem_ramdump_segments_local[i].client_id == KERNEL_VS_MEM_CLIENT_INVALID) {
				f3mem_ramdump_segments_local[i].client_id = client_id;
				f3mem_ramdump_segments_local[i].address = memblock->phy_addr;
				f3mem_ramdump_segments_local[i].size = memblock->size;
				f3mem_ramdump_segments_local[i].v_address = memblock->virtual_addr;
				break;
			}
		}
	} else {
		pr_info("f3mem_add_ramdump_segment: Segment already added, update\n");
		f3mem_ramdump_segments_local[found].client_id = client_id;
		f3mem_ramdump_segments_local[found].address = memblock->phy_addr;
		f3mem_ramdump_segments_local[found].size = memblock->size;
		f3mem_ramdump_segments_local[found].v_address = memblock->virtual_addr;
	}

	f3mem_print_ramdump_segments();
}

static void f3mem_remove_ramdump_segment(int client_id)
{
	int i = 0;
	int found = KERNEL_VS_MEM_CLIENT_INVALID;

	pr_info("f3mem_remove_ramdump_segment\n");

	for (i = 0; i < MAX_CLIENTS; i++) {
		if (f3mem_ramdump_segments_local[i].client_id == client_id) {
			found = i;
			break;
		}
	}
	if (found == KERNEL_VS_MEM_CLIENT_INVALID) {
		pr_info("f3mem_remove_ramdump_segment: No registered segment\n");
		return;
	} else {
		pr_info("f3mem_remove_ramdump_segment: Segment already added, remove\n");
		f3mem_ramdump_segments_local[found].client_id = KERNEL_VS_MEM_CLIENT_INVALID;
		f3mem_ramdump_segments_local[found].address = 0;
		f3mem_ramdump_segments_local[found].size = 0;
		f3mem_ramdump_segments_local[found].v_address = 0;
	}

	f3mem_print_ramdump_segments();
}

static int imem_restart_notifier_cb(struct notifier_block *this,
				unsigned long code,
				void *data)
{
	struct restart_notifier_block *notifier;
	struct notif_data *notifdata = data;
	int ret;

	pr_info("imem_restart_notifier_cb: code 0x%lx\n", code);

	switch (code) {

	case SUBSYS_AFTER_SHUTDOWN:
		notifier = container_of(this,
					struct restart_notifier_block, nb);
		pr_info("%s: ssrestart for processor %d ('%s')\n",
				__func__, notifier->processor,
				notifier->name);
		break;
	case SUBSYS_RAMDUMP_NOTIFICATION:
		if (!(imem_ramdump_dev && notifdata->enable_ramdump))
			break;
		pr_info("%s: saving ramdump\n", __func__);
        
		/*
		 * XPU protection does not currently allow the
		 * auxiliary memory regions to be dumped.  If this
		 * changes, then num_smem_areas + 1 should be passed
		 * into do_elf_ramdump() to dump all regions.
		 */
		ret = do_ramdump(imem_ramdump_dev,
				imem_ramdump_segment, 1);
		if (ret < 0)
			pr_err("%s: unable to dump smem %d\n", __func__, ret);
		break;
		default:
		break;
	}

	return NOTIFY_DONE;
}

static int f3mem_restart_notifier_cb(struct notifier_block *this,
				unsigned long code,
				void *data)
{
	struct restart_notifier_block *notifier;
	struct notif_data *notifdata = data;
	int ret;
	int i;

	pr_info("f3mem_restart_notifier_cb: code 0x%lx\n", code);

	switch (code) {

	case SUBSYS_AFTER_SHUTDOWN:
		notifier = container_of(this,
					struct restart_notifier_block, nb);
		pr_info("%s: ssrestart for processor %d ('%s')\n",
				__func__, notifier->processor,
				notifier->name);
		break;
	case SUBSYS_RAMDUMP_NOTIFICATION:
		if (!(f3mem_ramdump_dev && notifdata->enable_ramdump))
			break;
		pr_info("%s: saving ramdump\n", __func__);
		kfree(f3mem_ramdump_segments);
		f3mem_ramdump_segments = kcalloc(MAX_CLIENTS,
			sizeof(struct ramdump_segment), GFP_KERNEL);
		if (!f3mem_ramdump_segments) {
	    		pr_err("%s: ramdump segment kmalloc failed\n", __func__);
	    		ret = -ENOMEM;
	    		return ret;
		}
		num_f3mem_areas = 0;
		for (i = 0; i < MAX_CLIENTS; i++) {
			if (f3mem_ramdump_segments_local[i].client_id != KERNEL_VS_MEM_CLIENT_INVALID) {
				f3mem_ramdump_segments[num_f3mem_areas].address = f3mem_ramdump_segments_local[i].address;
				f3mem_ramdump_segments[num_f3mem_areas].v_address = f3mem_ramdump_segments_local[i].v_address;
				f3mem_ramdump_segments[num_f3mem_areas].size = f3mem_ramdump_segments_local[i].size;
				num_f3mem_areas++;
			}
		}
        
		/*
		 * XPU protection does not currently allow the
		 * auxiliary memory regions to be dumped.  If this
		 * changes, then num_smem_areas + 1 should be passed
		 * into do_elf_ramdump() to dump all regions.
		 */
		if (num_f3mem_areas > 0) {
			ret = do_ramdump(f3mem_ramdump_dev,
					f3mem_ramdump_segments, num_f3mem_areas);
			if (ret < 0)
				pr_err("%s: unable to dump smem %d\n", __func__, ret);
		}
		kfree(f3mem_ramdump_segments);
		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

static int __init msm_scan_dt_map_imem_for_ramdump(unsigned long node, const char *uname,
			int depth, void *data)
{
	int i;
	void *handle;
	struct restart_notifier_block *nb;

	const unsigned int *imem_prop;
	int imem_prop_len;
	int ret;
	const char *compat = "qcom,msm-imem";

	ret = of_flat_dt_is_compatible(node, compat);

	if (!ret)
		return 0;

	pr_info("imem_modem_restart_late_init\n");

	imem_prop = of_get_flat_dt_prop(node, "reg",
					&imem_prop_len);

	if (!imem_prop) {
		pr_err("IMEM reg field not found\n");
		return 0;
	}

	if (imem_prop_len != (2*sizeof(u32))) {
		pr_err("IMEM range malformed\n");
		return 0;
	}

	imem_ramdump_dev = create_ramdump_device("imem", NULL);
	if (IS_ERR_OR_NULL(imem_ramdump_dev)) {
		pr_err("%s: Unable to create imem ramdump device.\n",
			__func__);
		imem_ramdump_dev = NULL;
	}

	imem_ramdump_segment = kcalloc(1,
		sizeof(struct ramdump_segment), GFP_KERNEL);
	if (!imem_ramdump_segment) {
		pr_err("%s: ramdump segment kmalloc failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	imem_ramdump_segment->v_address = NULL;
	imem_ramdump_segment->address = be32_to_cpu(imem_prop[0]);
	imem_ramdump_segment->size = be32_to_cpu(imem_prop[1]);
	pr_info("msm_scan_dt_map_imem_for_ramdump: v_address 0x%lx, address 0x%lx, size 0x%lx\n",
			(unsigned long int)imem_ramdump_segment->v_address,
			(unsigned long int)imem_ramdump_segment->address,
			imem_ramdump_segment->size);

	for (i = 0; i < ARRAY_SIZE(imem_restart_notifiers); i++) {
		nb = &imem_restart_notifiers[i];
		handle = subsys_notif_register_notifier(nb->name, &nb->nb);
		pr_info("%s: registering notif for '%s', handle=%p\n",
				__func__, nb->name, handle);
	}

	return 0;
}

static __init int f3mem_modem_restart_late_init(void)
{
	int i;
	void *handle;
	struct restart_notifier_block *nb;

	pr_info("f3mem_modem_restart_late_init\n");

	f3mem_ramdump_dev = create_ramdump_device("f3mem", NULL);
	if (IS_ERR_OR_NULL(f3mem_ramdump_dev)) {
		pr_err("%s: Unable to create f3mem ramdump device.\n",
			__func__);
		f3mem_ramdump_dev = NULL;
	}

	for (i = 0; i < ARRAY_SIZE(f3mem_restart_notifiers); i++) {
		nb = &f3mem_restart_notifiers[i];
		handle = subsys_notif_register_notifier(nb->name, &nb->nb);
		pr_info("%s: registering notif for '%s', handle=%p\n",
				__func__, nb->name, handle);
	}

	of_scan_flat_dt(msm_scan_dt_map_imem_for_ramdump, NULL);

	return 0;
}
late_initcall(f3mem_modem_restart_late_init);

int is_f3mem_range(unsigned long pfn, unsigned long size)
{
	int i = 0;
	u64 from = ((u64)pfn) << PAGE_SHIFT;
	u64 to = from + size;

	pr_info("is_f3mem_range: from 0x%llx, to 0x%llx\n", from, to);

	for (i = 0; i < MAX_CLIENTS; i++) {
		if (f3mem_ramdump_segments_local[i].client_id != KERNEL_VS_MEM_CLIENT_INVALID) {
			if( (from < f3mem_ramdump_segments_local[i].address) || 
				(to > (f3mem_ramdump_segments_local[i].address + f3mem_ramdump_segments_local[i].size)) ) {
				continue;
			}
			else {
				pr_info("is_f3mem_range: belong to f3mem\n");
				return 1;
			}
		}
	}
	return 1;
}
EXPORT_SYMBOL(is_f3mem_range);

MODULE_DESCRIPTION("F3MEM QMI Service Driver");
MODULE_LICENSE("GPL v2");
