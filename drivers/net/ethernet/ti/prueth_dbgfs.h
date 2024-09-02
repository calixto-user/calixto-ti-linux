/* SPDX-License-Identifier: GPL-2.0 */
/* PRU Ethernet Driver
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com
 *
 */
#if IS_ENABLED(CONFIG_DEBUG_FS)
int prueth_dualemac_debugfs_init(struct prueth_emac *emac);
void prueth_dualemac_debugfs_term(struct prueth_emac *emac);
int prueth_hsr_prp_debugfs_init(struct prueth *prueth);
void prueth_hsr_prp_debugfs_term(struct prueth *prueth);
int prueth_debugfs_init(struct prueth_emac *emac);
void prueth_debugfs_term(struct prueth_emac *emac);
int prueth_sw_debugfs_init(struct prueth *prueth);
void prueth_sw_debugfs_term(struct prueth *prueth);

#else
static inline int prueth_hsr_prp_debugfs_init(struct prueth *prueth)
{
	return 0;
}

static inline void prueth_hsr_prp_debugfs_term(struct prueth *prueth)
{}

static inline int prueth_debugfs_init(struct prueth_emac *emac)
{
	return 0;
}

static inline void prueth_debugfs_term(struct prueth_emac *emac)
{}

static inline int prueth_sw_debugfs_init(struct prueth *prueth)
{
	return 0;
}

static inline void prueth_sw_debugfs_term(struct prueth *prueth)
{}
#endif
