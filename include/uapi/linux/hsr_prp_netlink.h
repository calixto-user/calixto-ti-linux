/* SPDX-License-Identifier: GPL-2.0 */
/* prp_prp_netlink.h: This is based on hsr_netlink.h from Arvid Brodin,
 * arvid.brodin@alten.se
 *
 * Copyright 2011-2014 Autronica Fire and Security AS
 * Copyright (C) 2019 Texas Instruments Incorporated
 *
 * Author(s):
 *	2011-2014 Arvid Brodin, arvid.brodin@alten.se
 *	2019 Murali Karicheri <m-karicheri2@ti.com>
 */

#ifndef __UAPI_HSR_PRP_NETLINK_H
#define __UAPI_HSR_PRP_NETLINK_H

/* Generic Netlink HSR/PRP family definition
 */

/* attributes */
enum {
	HSR_PRP_A_UNSPEC,
	HSR_PRP_A_NODE_ADDR,
	HSR_PRP_A_IFINDEX,
	HSR_PRP_A_IF1_AGE,
	HSR_PRP_A_IF2_AGE,
	HSR_PRP_A_NODE_ADDR_B,
	HSR_PRP_A_IF1_SEQ,
	HSR_PRP_A_IF2_SEQ,
	HSR_PRP_A_IF1_IFINDEX,
	HSR_PRP_A_IF2_IFINDEX,
	HSR_PRP_A_ADDR_B_IFINDEX,
	__HSR_PRP_A_MAX,
};
#define HSR_PRP_A_MAX (__HSR_PRP_A_MAX - 1)


/* commands */
enum {
	HSR_PRP_C_UNSPEC,
	HSR_C_RING_ERROR, /* only for HSR for now */
	HSR_PRP_C_NODE_DOWN,
	HSR_PRP_C_GET_NODE_STATUS,
	HSR_PRP_C_SET_NODE_STATUS,
	HSR_PRP_C_GET_NODE_LIST,
	HSR_PRP_C_SET_NODE_LIST,
	__HSR_PRP_C_MAX,
};
#define HSR_PRP_C_MAX (__HSR_PRP_C_MAX - 1)

#endif /* __UAPI_HSR_PRP_NETLINK_H */
