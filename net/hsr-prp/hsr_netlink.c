// SPDX-License-Identifier: GPL-2.0
/* Copyright 2011-2014 Autronica Fire and Security AS
 *
 * Author(s):
 *	2011-2014 Arvid Brodin, arvid.brodin@alten.se
 *
 * Routines for handling Netlink messages for HSR.
 */

#include <linux/kernel.h>
#include <net/genetlink.h>
#include <net/rtnetlink.h>

#include "hsr_netlink.h"
#include "hsr_prp_device.h"
#include "hsr_prp_framereg.h"
#include "hsr_prp_main.h"
#include "hsr_prp_netlink.h"

static const struct nla_policy hsr_policy[IFLA_HSR_PRP_MAX + 1] = {
	[IFLA_HSR_PRP_SLAVE1]		= { .type = NLA_U32 },
	[IFLA_HSR_PRP_SLAVE2]		= { .type = NLA_U32 },
	[IFLA_HSR_PRP_SF_MC_ADDR_LSB]	= { .type = NLA_U8 },
	[IFLA_HSR_VERSION]	= { .type = NLA_U8 },
	[IFLA_HSR_PRP_SF_MC_ADDR]	= { .len = ETH_ALEN },
	[IFLA_HSR_PRP_SEQ_NR]		= { .type = NLA_U16 },
	[IFLA_HSR_PRP_SV_VID]		= { .type = NLA_U16 },
	[IFLA_HSR_PRP_SV_PCP]		= { .type = NLA_U8 },
	[IFLA_HSR_PRP_SV_DEI]		= { .type = NLA_U8 },
};

static int hsr_newlink(struct net *src_net, struct net_device *dev,
		       struct nlattr *tb[], struct nlattr *data[],
		       struct netlink_ext_ack *extack)
{
	return hsr_prp_newlink(HSR, src_net, dev, tb, data, extack);
}

static struct rtnl_link_ops hsr_link_ops __read_mostly = {
	.kind		= "hsr",
	.maxtype	= IFLA_HSR_PRP_MAX,
	.policy		= hsr_policy,
	.priv_size	= sizeof(struct hsr_prp_priv),
	.setup		= hsr_dev_setup,
	.newlink	= hsr_newlink,
	.fill_info	= hsr_prp_fill_info,
};

/* attribute policy */
static const struct nla_policy hsr_genl_policy[HSR_PRP_A_MAX + 1] = {
	[HSR_PRP_A_NODE_ADDR] = { .len = ETH_ALEN },
	[HSR_PRP_A_NODE_ADDR_B] = { .len = ETH_ALEN },
	[HSR_PRP_A_IFINDEX] = { .type = NLA_U32 },
	[HSR_PRP_A_IF1_AGE] = { .type = NLA_U32 },
	[HSR_PRP_A_IF2_AGE] = { .type = NLA_U32 },
	[HSR_PRP_A_IF1_SEQ] = { .type = NLA_U16 },
	[HSR_PRP_A_IF2_SEQ] = { .type = NLA_U16 },
};

static struct genl_family hsr_genl_family;

static const struct genl_multicast_group hsr_mcgrps[] = {
	{ .name = "hsr-network", },
};

/* This is called if for some node with MAC address addr, we only get frames
 * over one of the slave interfaces. This would indicate an open network ring
 * (i.e. a link has failed somewhere).
 */
void hsr_prp_nl_ringerror(struct hsr_prp_priv *priv,
			  unsigned char addr[ETH_ALEN],
			  struct hsr_prp_port *port)
{
	struct sk_buff *skb;
	void *msg_head;
	struct hsr_prp_port *master;
	int res;

	skb = genlmsg_new(NLMSG_GOODSIZE, GFP_ATOMIC);
	if (!skb)
		goto fail;

	msg_head = genlmsg_put(skb, 0, 0, &hsr_genl_family, 0,
			       HSR_C_RING_ERROR);
	if (!msg_head)
		goto nla_put_failure;

	res = nla_put(skb, HSR_PRP_A_NODE_ADDR, ETH_ALEN, addr);
	if (res < 0)
		goto nla_put_failure;

	res = nla_put_u32(skb, HSR_PRP_A_IFINDEX, port->dev->ifindex);
	if (res < 0)
		goto nla_put_failure;

	genlmsg_end(skb, msg_head);
	genlmsg_multicast(&hsr_genl_family, skb, 0, 0, GFP_ATOMIC);

	return;

nla_put_failure:
	kfree_skb(skb);

fail:
	rcu_read_lock();
	master = hsr_prp_get_port(priv, HSR_PRP_PT_MASTER);
	netdev_warn(master->dev, "Could not send HSR ring error message\n");
	rcu_read_unlock();
}

/* This is called when we haven't heard from the node with MAC address addr for
 * some time (just before the node is removed from the node table/list).
 */
void hsr_nl_nodedown(struct hsr_prp_priv *priv, unsigned char addr[ETH_ALEN])
{
	hsr_prp_nl_nodedown(priv, &hsr_genl_family, addr);
}

/* HSR_C_GET_NODE_STATUS lets userspace query the internal HSR node table
 * about the status of a specific node in the network, defined by its MAC
 * address.
 *
 * Input: hsr ifindex, node mac address
 * Output: hsr ifindex, node mac address (copied from request),
 *	   age of latest frame from node over slave 1, slave 2 [ms]
 */
static int hsr_get_node_status(struct sk_buff *skb_in, struct genl_info *info)
{
	return hsr_prp_get_node_status(&hsr_genl_family, skb_in, info);
}

/* Get a list of MacAddressA of all nodes known to this node (including self).
 */
static int hsr_get_node_list(struct sk_buff *skb_in, struct genl_info *info)
{
	return hsr_prp_get_node_list(&hsr_genl_family, skb_in, info);
}

static const struct genl_ops hsr_ops[] = {
	{
		.cmd = HSR_PRP_C_GET_NODE_STATUS,
		.flags = 0,
		.policy = hsr_genl_policy,
		.doit = hsr_get_node_status,
		.dumpit = NULL,
	},
	{
		.cmd = HSR_PRP_C_GET_NODE_LIST,
		.flags = 0,
		.policy = hsr_genl_policy,
		.doit = hsr_get_node_list,
		.dumpit = NULL,
	},
};

static struct genl_family hsr_genl_family __ro_after_init = {
	.hdrsize = 0,
	.name = "HSR",
	.version = 1,
	.maxattr = HSR_PRP_A_MAX,
	.module = THIS_MODULE,
	.ops = hsr_ops,
	.n_ops = ARRAY_SIZE(hsr_ops),
	.mcgrps = hsr_mcgrps,
	.n_mcgrps = ARRAY_SIZE(hsr_mcgrps),
};

int __init hsr_netlink_init(void)
{
	int rc;

	rc = rtnl_link_register(&hsr_link_ops);
	if (rc)
		goto fail_rtnl_link_register;

	rc = genl_register_family(&hsr_genl_family);
	if (rc)
		goto fail_genl_register_family;

	return 0;

fail_genl_register_family:
	rtnl_link_unregister(&hsr_link_ops);
fail_rtnl_link_register:

	return rc;
}

void __exit hsr_netlink_exit(void)
{
	genl_unregister_family(&hsr_genl_family);
	rtnl_link_unregister(&hsr_link_ops);
}

MODULE_ALIAS_RTNL_LINK("hsr");
