/* SPDX-License-Identifier: GPL-2.0 */
/*
 * prp_netlink.h:
 * This is based on hsr_netlink.h from Arvid Brodin, arvid.brodin@alten.se
 *
 * Copyright (C) 2017-2018 Texas Instruments Incorporated
 *
 * Author(s):
 *	Murali Karicheri <m-karicheri2@ti.com>
 */

#ifndef __PRP_NETLINK_H
#define __PRP_NETLINK_H

#include <linux/if_ether.h>
#include <linux/module.h>
#include <uapi/linux/hsr_prp_netlink.h>

struct hsr_prp_priv;
struct hsr_prp_port;

int __init prp_netlink_init(void);
void __exit prp_netlink_exit(void);

void prp_nl_nodedown(struct hsr_prp_priv *priv, unsigned char addr[ETH_ALEN]);

#endif /* __PRP_NETLINK_H */
