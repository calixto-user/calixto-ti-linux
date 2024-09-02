// SPDX-License-Identifier: GPL-2.0
/*
 * prp_netlink.c:  Routines for handling Netlink messages for PRP.
 * This is based on hsr_netlink.c from Arvid Brodin, arvid.brodin@alten.se
 *
 * Copyright (C) 2019 Texas Instruments Incorporated
 *
 * Author(s):
 *	Murali Karicheri <m-karicheri2@ti.com>
 */

#include <linux/kernel.h>
#include <net/genetlink.h>
#include <net/rtnetlink.h>

#include "prp_netlink.h"
#include "hsr_prp_device.h"
#include "hsr_prp_framereg.h"
#include "hsr_prp_main.h"
#include "hsr_prp_netlink.h"

static const struct nla_policy prp_policy[IFLA_HSR_PRP_MAX + 1] = {
	[IFLA_HSR_PRP_SLAVE1]		= { .type = NLA_U32 },
	[IFLA_HSR_PRP_SLAVE2]		= { .type = NLA_U32 },
	[IFLA_HSR_PRP_SF_MC_ADDR_LSB]	= { .type = NLA_U8 },
	[IFLA_HSR_PRP_SF_MC_ADDR]	= { .len = ETH_ALEN },
	[IFLA_HSR_PRP_SEQ_NR]		= { .type = NLA_U16 },
};

static int prp_newlink(struct net *src_net, struct net_device *dev,
		       struct nlattr *tb[], struct nlattr *data[],
		       struct netlink_ext_ack *extack)
{
	return hsr_prp_newlink(PRP, src_net, dev, tb, data, extack);
}

static struct rtnl_link_ops prp_link_ops __read_mostly = {
	.kind		= "prp",
	.maxtype	= IFLA_HSR_PRP_MAX,
	.policy		= prp_policy,
	.priv_size	= sizeof(struct hsr_prp_priv),
	.setup		= prp_dev_setup,
	.newlink	= prp_newlink,
	.fill_info	= hsr_prp_fill_info,
};

/* NLA_BINARY missing in libnl; use NLA_UNSPEC in userspace instead. */
static const struct nla_policy prp_genl_policy[HSR_PRP_A_MAX + 1] = {
	[HSR_PRP_A_NODE_ADDR] = { .type = NLA_BINARY, .len = ETH_ALEN },
	[HSR_PRP_A_NODE_ADDR_B] = { .type = NLA_BINARY, .len = ETH_ALEN },
	[HSR_PRP_A_IFINDEX] = { .type = NLA_U32 },
	[HSR_PRP_A_IF1_AGE] = { .type = NLA_U32 },
	[HSR_PRP_A_IF2_AGE] = { .type = NLA_U32 },
	[HSR_PRP_A_IF1_SEQ] = { .type = NLA_U16 },
	[HSR_PRP_A_IF2_SEQ] = { .type = NLA_U16 },
};

static struct genl_family prp_genl_family;

static const struct genl_multicast_group prp_mcgrps[] = {
	{ .name = "prp-network", },
};

/* This is called when we haven't heard from the node with MAC address addr for
 * some time (just before the node is removed from the node table/list).
 */
void prp_nl_nodedown(struct hsr_prp_priv *priv, unsigned char addr[ETH_ALEN])
{
	hsr_prp_nl_nodedown(priv, &prp_genl_family, addr);
}

static int prp_get_node_status(struct sk_buff *skb_in, struct genl_info *info)
{
	return hsr_prp_get_node_status(&prp_genl_family, skb_in, info);
}

static int prp_get_node_list(struct sk_buff *skb_in, struct genl_info *info)
{
	return hsr_prp_get_node_list(&prp_genl_family, skb_in, info);
}

static const struct genl_ops prp_ops[] = {
	{
		.cmd = HSR_PRP_C_GET_NODE_STATUS,
		.flags = 0,
		.policy = prp_genl_policy,
		.doit = prp_get_node_status,
		.dumpit = NULL,
	},
	{
		.cmd = HSR_PRP_C_GET_NODE_LIST,
		.flags = 0,
		.policy = prp_genl_policy,
		.doit = prp_get_node_list,
		.dumpit = NULL,
	},
};

static struct genl_family prp_genl_family __ro_after_init = {
	.hdrsize = 0,
	.name = "PRP",
	.version = 1,
	.maxattr = HSR_PRP_A_MAX,
	.module = THIS_MODULE,
	.ops = prp_ops,
	.n_ops = ARRAY_SIZE(prp_ops),
	.mcgrps = prp_mcgrps,
	.n_mcgrps = ARRAY_SIZE(prp_mcgrps),
};

int __init prp_netlink_init(void)
{
	int rc;

	rc = rtnl_link_register(&prp_link_ops);
	if (rc)
		goto fail_rtnl_link_register;

	rc = genl_register_family(&prp_genl_family);
	if (rc)
		goto fail_genl_register_family;

	return 0;

fail_genl_register_family:
	rtnl_link_unregister(&prp_link_ops);
fail_rtnl_link_register:

	return rc;
}

void __exit prp_netlink_exit(void)
{
	genl_unregister_family(&prp_genl_family);
	rtnl_link_unregister(&prp_link_ops);
}

MODULE_ALIAS_RTNL_LINK("prp");
