// SPDX-License-Identifier: GPL-2.0
/* Copyright 2011-2014 Autronica Fire and Security AS
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com
 *
 * Author(s):
 *	2011-2014 Arvid Brodin, arvid.brodin@alten.se
 *
 * This file contains device methods for creating, using and destroying
 * virtual HSR devices.
 */

#include <linux/kernel.h>
#include <net/rtnetlink.h>
#include <net/genetlink.h>

#include "hsr_prp_main.h"
#include "hsr_prp_netlink.h"
#include "hsr_prp_device.h"
#include "hsr_prp_framereg.h"

/* Here, it seems a netdevice has already been allocated for us, and the
 * hsr_prp_dev_setup routine has been executed. Nice!
 */
int hsr_prp_newlink(int proto, struct net *src_net,
		    struct net_device *dev, struct nlattr *tb[],
		    struct nlattr *data[],
		    struct netlink_ext_ack *extack)
{
	struct net_device *link[2];
	unsigned char mc_lsb, version = 0;
	char *sproto = (proto == PRP) ? "PRP" : "HSR";
	unsigned short vid = 0;
	unsigned char pcp = 0, dei = 0;
	bool sv_vlan_tag_needed = false;

	if (!data) {
		netdev_info(dev, "%s: No slave devices specified\n", sproto);
		return -EINVAL;
	}
	if (!data[IFLA_HSR_PRP_SLAVE1]) {
		netdev_info(dev, "%s: Slave1 device not specified\n", sproto);
		return -EINVAL;
	}
	link[0] = __dev_get_by_index(src_net,
				     nla_get_u32(data[IFLA_HSR_PRP_SLAVE1]));
	if (!data[IFLA_HSR_PRP_SLAVE2]) {
		netdev_info(dev, "%s: Slave2 device not specified\n", sproto);
		return -EINVAL;
	}
	link[1] = __dev_get_by_index(src_net,
				     nla_get_u32(data[IFLA_HSR_PRP_SLAVE2]));

	if (!link[0] || !link[1])
		return -ENODEV;
	if (link[0] == link[1])
		return -EINVAL;

	if (!data[IFLA_HSR_PRP_SF_MC_ADDR_LSB])
		mc_lsb = 0;
	else
		mc_lsb = nla_get_u8(data[IFLA_HSR_PRP_SF_MC_ADDR_LSB]);

	if (proto == PRP) {
		version = PRP_V1;
	} else {
		if (!data[IFLA_HSR_VERSION])
			version = 0;
		else
			version = nla_get_u8(data[IFLA_HSR_VERSION]);
	}

	if (data[IFLA_HSR_PRP_SV_VID]) {
		sv_vlan_tag_needed = true;
		vid = nla_get_u16(data[IFLA_HSR_PRP_SV_VID]);
	}

	if (data[IFLA_HSR_PRP_SV_PCP]) {
		sv_vlan_tag_needed = true;
		pcp = nla_get_u8(data[IFLA_HSR_PRP_SV_PCP]);
	}

	if (data[IFLA_HSR_PRP_SV_DEI]) {
		sv_vlan_tag_needed = true;
		dei = nla_get_u8(data[IFLA_HSR_PRP_SV_DEI]);
	}

	if (sv_vlan_tag_needed &&
	    (vid >= (VLAN_N_VID - 1) || dei > 1 || pcp > 7)) {
		netdev_info(dev,
			    "%s: wrong vlan params: vid %d, pcp %d, dei %d\n",
			    sproto, vid, pcp, dei);
		return -EINVAL;
	}

	return hsr_prp_dev_finalize(dev, link, mc_lsb, version,
				    sv_vlan_tag_needed, vid, pcp, dei);
}

int hsr_prp_fill_info(struct sk_buff *skb, const struct net_device *dev)
{
	struct hsr_prp_priv *priv;
	struct hsr_prp_port *port;
	int res;

	priv = netdev_priv(dev);

	res = 0;

	rcu_read_lock();
	port = hsr_prp_get_port(priv, HSR_PRP_PT_SLAVE_A);
	if (port)
		res = nla_put_u32(skb, IFLA_HSR_PRP_SLAVE1, port->dev->ifindex);
	rcu_read_unlock();
	if (res)
		goto nla_put_failure;

	rcu_read_lock();
	port = hsr_prp_get_port(priv, HSR_PRP_PT_SLAVE_B);
	if (port)
		res = nla_put_u32(skb, IFLA_HSR_PRP_SLAVE2, port->dev->ifindex);
	rcu_read_unlock();
	if (res)
		goto nla_put_failure;

	if (nla_put(skb, IFLA_HSR_PRP_SF_MC_ADDR, ETH_ALEN,
		    priv->sup_multicast_addr) ||
	    nla_put_u16(skb, IFLA_HSR_PRP_SEQ_NR, priv->sequence_nr))
		goto nla_put_failure;

	return 0;

nla_put_failure:
	return -EMSGSIZE;
}

/* This is called when we haven't heard from the node with MAC address addr for
 * some time (just before the node is removed from the node table/list).
 */
void hsr_prp_nl_nodedown(struct hsr_prp_priv *priv,
			 struct genl_family *genl_family,
			 unsigned char addr[ETH_ALEN])
{
	struct sk_buff *skb;
	void *msg_head;
	struct hsr_prp_port *master;
	int res;

	skb = genlmsg_new(NLMSG_GOODSIZE, GFP_ATOMIC);
	if (!skb)
		goto fail;

	msg_head = genlmsg_put(skb, 0, 0, genl_family, 0, HSR_PRP_C_NODE_DOWN);
	if (!msg_head)
		goto nla_put_failure;

	res = nla_put(skb, HSR_PRP_A_NODE_ADDR, ETH_ALEN, addr);
	if (res < 0)
		goto nla_put_failure;

	genlmsg_end(skb, msg_head);
	genlmsg_multicast(genl_family, skb, 0, 0, GFP_ATOMIC);

	return;

nla_put_failure:
	kfree_skb(skb);

fail:
	rcu_read_lock();
	master = hsr_prp_get_port(priv, HSR_PRP_PT_MASTER);
	netdev_warn(master->dev, "Could not send PRP node down\n");
	rcu_read_unlock();
}

/* PRP_C_GET_NODE_STATUS lets userspace query the internal PRP node table
 * about the status of a specific node in the network, defined by its MAC
 * address.
 *
 * Input: hsr ifindex, node mac address
 * Output: hsr ifindex, node mac address (copied from request),
 *	   age of latest frame from node over slave 1, slave 2 [ms]
 */
int hsr_prp_get_node_status(struct genl_family *genl_family,
			    struct sk_buff *skb_in,
			    struct genl_info *info)
{
	/* For receiving */
	struct nlattr *na;
	struct net_device *ndev;

	/* For sending */
	struct sk_buff *skb_out;
	void *msg_head;
	struct hsr_prp_priv *priv;
	struct hsr_prp_port *port;
	unsigned char hsr_node_addr_b[ETH_ALEN];
	int hsr_node_if1_age;
	u16 hsr_node_if1_seq;
	int hsr_node_if2_age;
	u16 hsr_node_if2_seq;
	int addr_b_ifindex;
	int res;

	if (!info)
		goto invalid;

	na = info->attrs[HSR_PRP_A_IFINDEX];
	if (!na)
		goto invalid;
	na = info->attrs[HSR_PRP_A_NODE_ADDR];
	if (!na)
		goto invalid;

	ndev = __dev_get_by_index(genl_info_net(info),
				  nla_get_u32(info->attrs[HSR_PRP_A_IFINDEX]));
	if (!ndev)
		goto invalid;
	if (!is_hsr_prp_master(ndev))
		goto invalid;

	/* Send reply */
	skb_out = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
	if (!skb_out) {
		res = -ENOMEM;
		goto fail;
	}

	msg_head = genlmsg_put(skb_out, NETLINK_CB(skb_in).portid,
			       info->snd_seq, genl_family, 0,
			       HSR_PRP_C_SET_NODE_STATUS);
	if (!msg_head) {
		res = -ENOMEM;
		goto nla_put_failure;
	}

	res = nla_put_u32(skb_out, HSR_PRP_A_IFINDEX, ndev->ifindex);
	if (res < 0)
		goto nla_put_failure;

	priv = netdev_priv(ndev);
	res = hsr_prp_get_node_data(priv,
				    (unsigned char *)
				    nla_data(info->attrs[HSR_PRP_A_NODE_ADDR]),
				    hsr_node_addr_b, &addr_b_ifindex,
				    &hsr_node_if1_age, &hsr_node_if1_seq,
				    &hsr_node_if2_age, &hsr_node_if2_seq);
	if (res < 0)
		goto nla_put_failure;

	res = nla_put(skb_out, HSR_PRP_A_NODE_ADDR, ETH_ALEN,
		      nla_data(info->attrs[HSR_PRP_A_NODE_ADDR]));
	if (res < 0)
		goto nla_put_failure;

	if (addr_b_ifindex > -1) {
		res = nla_put(skb_out, HSR_PRP_A_NODE_ADDR_B, ETH_ALEN,
			      hsr_node_addr_b);
		if (res < 0)
			goto nla_put_failure;

		res = nla_put_u32(skb_out, HSR_PRP_A_ADDR_B_IFINDEX,
				  addr_b_ifindex);
		if (res < 0)
			goto nla_put_failure;
	}

	res = nla_put_u32(skb_out, HSR_PRP_A_IF1_AGE, hsr_node_if1_age);
	if (res < 0)
		goto nla_put_failure;
	res = nla_put_u16(skb_out, HSR_PRP_A_IF1_SEQ, hsr_node_if1_seq);
	if (res < 0)
		goto nla_put_failure;
	rcu_read_lock();
	port = hsr_prp_get_port(priv, HSR_PRP_PT_SLAVE_A);
	if (port)
		res = nla_put_u32(skb_out, HSR_PRP_A_IF1_IFINDEX,
				  port->dev->ifindex);
	rcu_read_unlock();
	if (res < 0)
		goto nla_put_failure;

	res = nla_put_u32(skb_out, HSR_PRP_A_IF2_AGE, hsr_node_if2_age);
	if (res < 0)
		goto nla_put_failure;
	res = nla_put_u16(skb_out, HSR_PRP_A_IF2_SEQ, hsr_node_if2_seq);
	if (res < 0)
		goto nla_put_failure;
	rcu_read_lock();
	port = hsr_prp_get_port(priv, HSR_PRP_PT_SLAVE_B);
	if (port)
		res = nla_put_u32(skb_out, HSR_PRP_A_IF2_IFINDEX,
				  port->dev->ifindex);
	rcu_read_unlock();
	if (res < 0)
		goto nla_put_failure;

	genlmsg_end(skb_out, msg_head);
	genlmsg_unicast(genl_info_net(info), skb_out, info->snd_portid);

	return 0;

invalid:
	netlink_ack(skb_in, nlmsg_hdr(skb_in), -EINVAL, NULL);
	return 0;

nla_put_failure:
	kfree_skb(skb_out);
	/* Fall through */

fail:
	return res;
}

/* Get a list of mac_address_a of all nodes known to this node (including self).
 */
int hsr_prp_get_node_list(struct genl_family *genl_family,
			  struct sk_buff *skb_in, struct genl_info *info)
{
	/* For receiving */
	struct nlattr *na;
	struct net_device *ndev;

	/* For sending */
	struct sk_buff *skb_out;
	void *msg_head;
	struct hsr_prp_priv *priv;
	void *pos;
	unsigned char addr[ETH_ALEN];
	int res;

	if (!info)
		goto invalid;

	na = info->attrs[HSR_PRP_A_IFINDEX];
	if (!na)
		goto invalid;

	ndev = __dev_get_by_index(genl_info_net(info),
				  nla_get_u32(info->attrs[HSR_PRP_A_IFINDEX]));
	if (!ndev)
		goto invalid;
	if (!is_hsr_prp_master(ndev))
		goto invalid;

	/* Send reply */
	skb_out = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
	if (!skb_out) {
		res = -ENOMEM;
		goto fail;
	}

	msg_head = genlmsg_put(skb_out, NETLINK_CB(skb_in).portid,
			       info->snd_seq, genl_family, 0,
			       HSR_PRP_C_SET_NODE_LIST);
	if (!msg_head) {
		res = -ENOMEM;
		goto nla_put_failure;
	}

	res = nla_put_u32(skb_out, HSR_PRP_A_IFINDEX, ndev->ifindex);
	if (res < 0)
		goto nla_put_failure;

	priv = netdev_priv(ndev);

	rcu_read_lock();
	pos = hsr_prp_get_next_node(priv, NULL, addr);
	while (pos) {
		if (!hsr_prp_addr_is_self(priv, addr)) {
			res = nla_put(skb_out, HSR_PRP_A_NODE_ADDR,
				      ETH_ALEN, addr);
			if (res < 0) {
				rcu_read_unlock();
				goto nla_put_failure;
			}
		}
		pos = hsr_prp_get_next_node(priv, pos, addr);
	}
	rcu_read_unlock();

	genlmsg_end(skb_out, msg_head);
	genlmsg_unicast(genl_info_net(info), skb_out, info->snd_portid);

	return 0;

invalid:
	netlink_ack(skb_in, nlmsg_hdr(skb_in), -EINVAL, NULL);
	return 0;

nla_put_failure:
	kfree_skb(skb_out);
	/* Fall through */

fail:
	return res;
}
