/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2011-2014 Autronica Fire and Security AS
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com
 *
 * Author(s):
 *	2011-2014 Arvid Brodin, arvid.brodin@alten.se
 *
 * This file contains device methods for creating, using and destroying
 * virtual HSR devices.
 */

#include <uapi/linux/hsr_prp_netlink.h>

int hsr_prp_newlink(int proto, struct net *src_net,
		    struct net_device *dev, struct nlattr *tb[],
		    struct nlattr *data[],
		    struct netlink_ext_ack *extack);
int hsr_prp_fill_info(struct sk_buff *skb, const struct net_device *dev);
void hsr_prp_nl_nodedown(struct hsr_prp_priv *priv,
			 struct genl_family *gen_family,
			 unsigned char addr[ETH_ALEN]);
int hsr_prp_get_node_status(struct genl_family *gen_family,
			    struct sk_buff *skb_in,
			    struct genl_info *info);
int hsr_prp_get_node_list(struct genl_family *gen_family,
			  struct sk_buff *skb_in, struct genl_info *info);
