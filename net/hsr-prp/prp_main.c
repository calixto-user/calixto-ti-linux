// SPDX-License-Identifier: GPL-2.0
/*
 * prp_main.c: hsr initialization code. This is based on hsr_main.c
 *
 * Copyright (C) 2017 Texas Instruments Incorporated
 *
 * Author(s):
 *	Murali Karicheri <m-karicheri2@ti.com>
 */
#include <linux/netdevice.h>

#include "hsr_prp_main.h"
#include "prp_netlink.h"

static int __init prp_init(void)
{
	int res;

	res = hsr_prp_register_notifier(PRP);
	if (!res)
		res = prp_netlink_init();

	return res;
}

static void __exit prp_exit(void)
{
	hsr_prp_unregister_notifier(PRP);
	prp_netlink_exit();
}

module_init(prp_init);
MODULE_LICENSE("GPL");
