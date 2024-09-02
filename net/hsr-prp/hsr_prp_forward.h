/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2011-2014 Autronica Fire and Security AS
 *
 * Author(s):
 *	2011-2014 Arvid Brodin, arvid.brodin@alten.se
 */

#ifndef __HSR_PRP_FORWARD_H
#define __HSR_PRP_FORWARD_H

#include <linux/netdevice.h>
#include "hsr_prp_main.h"

void hsr_prp_forward_skb(struct sk_buff *skb, struct hsr_prp_port *port);

#endif /* __HSR_PRP_FORWARD_H */
