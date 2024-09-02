// SPDX-License-Identifier: GPL-2.0
/* PRU Ethernet Driver sysfs file
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com
 *
 */
#include <linux/netdevice.h>
#include <linux/sysfs.h>
#include "prueth.h"

#define nsp_credit_bc_to_emac(attr) \
	container_of(attr, struct prueth_emac, nsp_credit_bc_attr)
#define nsp_credit_mc_to_emac(attr) \
	container_of(attr, struct prueth_emac, nsp_credit_mc_attr)
#define nsp_credit_uc_to_emac(attr) \
	container_of(attr, struct prueth_emac, nsp_credit_uc_attr)
#define prp_emac_mode_to_emac(attr) \
	container_of(attr, struct prueth_emac, prp_emac_mode_attr)

static void update_nsp_credit(u32 val, struct prueth_emac *emac,
			      unsigned int *credit, unsigned int nsp_offset)
{
	void __iomem *dram = emac->prueth->mem[emac->dram].va;

	if (val) {
		*credit = (val << PRUETH_NSP_CREDIT_SHIFT) | PRUETH_NSP_ENABLE;
	} else {
		*credit = PRUETH_NSP_DISABLE;
		writel(*credit, dram + nsp_offset);
	}
}

static ssize_t nsp_credit_bc_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buffer, size_t count)
{
	struct prueth_emac *emac = nsp_credit_bc_to_emac(attr);
	u32 val;

	if (kstrtou32(buffer, 0, &val))
		return -EINVAL;

	update_nsp_credit(val, emac, &emac->nsp_credit_bc,
			  STORM_PREVENTION_OFFSET_BC);

	return count;
}

static ssize_t nsp_credit_bc_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buffer)
{
	struct prueth_emac *emac = nsp_credit_bc_to_emac(attr);

	return snprintf(buffer, PAGE_SIZE, "%u\n",
			emac->nsp_credit_bc >> PRUETH_NSP_CREDIT_SHIFT);
}
DEVICE_ATTR_RW(nsp_credit_bc);

static ssize_t nsp_credit_mc_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buffer, size_t count)
{
	struct prueth_emac *emac = nsp_credit_mc_to_emac(attr);
	u32 val;

	if (kstrtou32(buffer, 0, &val))
		return -EINVAL;

	update_nsp_credit(val, emac, &emac->nsp_credit_mc,
			  STORM_PREVENTION_OFFSET_MC);

	return count;
}

static ssize_t nsp_credit_mc_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buffer)
{
	struct prueth_emac *emac = nsp_credit_mc_to_emac(attr);

	return snprintf(buffer, PAGE_SIZE, "%u\n",
			emac->nsp_credit_mc >> PRUETH_NSP_CREDIT_SHIFT);
}
DEVICE_ATTR_RW(nsp_credit_mc);

static ssize_t nsp_credit_uc_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buffer, size_t count)
{
	struct prueth_emac *emac = nsp_credit_uc_to_emac(attr);
	u32 val;

	if (kstrtou32(buffer, 0, &val))
		return -EINVAL;

	update_nsp_credit(val, emac, &emac->nsp_credit_uc,
			  STORM_PREVENTION_OFFSET_UC);

	return count;
}

static ssize_t nsp_credit_uc_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buffer)
{
	struct prueth_emac *emac = nsp_credit_uc_to_emac(attr);

	return snprintf(buffer, PAGE_SIZE, "%u\n",
			emac->nsp_credit_uc >> PRUETH_NSP_CREDIT_SHIFT);
}
DEVICE_ATTR_RW(nsp_credit_uc);

static ssize_t prp_emac_mode_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buffer, size_t count)
{
	struct prueth_emac *emac = prp_emac_mode_to_emac(attr);
	u32 emac_mode;
	int err;

	err = kstrtou32(buffer, 0, &emac_mode);
	if (err)
		return err;

	if (!PRUETH_HAS_PRP(emac->prueth))
		return -EINVAL;

	if (emac_mode > PRUETH_TX_PRP_EMAC_MODE)
		return -EINVAL;

	emac->prp_emac_mode = emac_mode;

	return count;
}

static ssize_t prp_emac_mode_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buffer)
{
	struct prueth_emac *emac = prp_emac_mode_to_emac(attr);

	return snprintf(buffer, PAGE_SIZE, "%u\n", emac->prp_emac_mode);
}
DEVICE_ATTR_RW(prp_emac_mode);

int prueth_sysfs_init(struct prueth_emac *emac)
{
	int ret;

	emac->nsp_credit_bc_attr = dev_attr_nsp_credit_bc;
	sysfs_attr_init(&emac->nsp_credit_bc_attr.attr);
	ret = device_create_file(&emac->ndev->dev, &emac->nsp_credit_bc_attr);
	if (ret < 0)
		return ret;

	emac->nsp_credit_mc_attr = dev_attr_nsp_credit_mc;
	sysfs_attr_init(&emac->nsp_credit_mc_attr.attr);
	ret = device_create_file(&emac->ndev->dev, &emac->nsp_credit_mc_attr);
	if (ret < 0)
		goto err_mc_fail;

	emac->nsp_credit_uc_attr = dev_attr_nsp_credit_uc;
	sysfs_attr_init(&emac->nsp_credit_uc_attr.attr);
	ret = device_create_file(&emac->ndev->dev, &emac->nsp_credit_uc_attr);
	if (ret < 0)
		goto err_uc_fail;

	emac->prp_emac_mode_attr = dev_attr_prp_emac_mode;
	sysfs_attr_init(&emac->prp_emac_mode_attr.attr);
	ret = device_create_file(&emac->ndev->dev, &emac->prp_emac_mode_attr);
	if (ret < 0)
		goto err_prp_emac_fail;
	
	return 0;

err_prp_emac_fail:
	device_remove_file(&emac->ndev->dev, &emac->nsp_credit_uc_attr);
err_uc_fail:
	device_remove_file(&emac->ndev->dev, &emac->nsp_credit_mc_attr);
err_mc_fail:
	device_remove_file(&emac->ndev->dev, &emac->nsp_credit_bc_attr);
	return ret;
}

void prueth_remove_sysfs_entries(struct prueth_emac *emac)
{
	device_remove_file(&emac->ndev->dev, &emac->nsp_credit_bc_attr);
	device_remove_file(&emac->ndev->dev, &emac->nsp_credit_mc_attr);
	device_remove_file(&emac->ndev->dev, &emac->nsp_credit_uc_attr);
	device_remove_file(&emac->ndev->dev, &emac->prp_emac_mode_attr);
}
