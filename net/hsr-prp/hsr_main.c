// SPDX-License-Identifier: GPL-2.0
/* Copyright 2011-2014 Autronica Fire and Security AS
 *
 * Author(s):
 *	2011-2014 Arvid Brodin, arvid.brodin@alten.se
 */

#include <linux/netdevice.h>

#include "hsr_netlink.h"
#include "hsr_prp_main.h"

static int __init hsr_init(void)
{
	int res;

	BUILD_BUG_ON(sizeof(struct hsr_tag) != HSR_PRP_HLEN);

	res = hsr_prp_register_notifier(HSR);
	if (!res)
		res = hsr_netlink_init();

	return res;
}

static void __exit hsr_exit(void)
{
	hsr_prp_unregister_notifier(HSR);
	hsr_netlink_exit();
}

module_init(hsr_init);
module_exit(hsr_exit);
MODULE_LICENSE("GPL");
