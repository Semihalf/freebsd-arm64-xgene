/* Applied Micro X-Gene SoC Ethernet Driver
 *
 * Copyright (c) 2014, Applied Micro Circuits Corporation
 * Authors: Iyappan Subramanian <isubramanian@apm.com>
 *	    Ravi Patel <rapatel@apm.com>
 *	    Keyur Chudgar <kchudgar@apm.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __XGENE_ENET_MAIN_H__
#define __XGENE_ENET_MAIN_H__

#include "xgene_enet_hw.h"
#include "xgene_enet_xgmac.h"

#define XGENE_DRV_VERSION	"v1.0"
#define XGENE_ENET_MAX_MTU	1536
#define SKB_BUFFER_SIZE		(XGENE_ENET_MAX_MTU - NET_IP_ALIGN)
#define NUM_PKT_BUF	64
#define NUM_BUFPOOL	32
#define START_ETH_BUFNUM	2
#define START_BP_BUFNUM		0x22
#define START_RING_NUM		8

#define PHY_POLL_LINK_ON	(10 * HZ)
#define PHY_POLL_LINK_OFF	(PHY_POLL_LINK_ON / 5)

/* software context of a descriptor ring */
struct xgene_enet_desc_ring {
	device_t ndev;
	u16 id;
	u16 num;
	u16 head;
	u16 tail;
	u16 slots;
	u16 irq;
	u32 size;
	u32 state[NUM_RING_CONFIG];
	bus_space_handle_t cmd_base;
	bus_space_handle_t cmd;
	u16 dst_ring_num;
	u8 nbufpool;

	bus_dma_tag_t buf_dmat;
	union {
		struct xge_buff *buff;
		struct xge_buff *tx_buff; /* Used for Tx dmap */
		struct {
			struct xge_buff *rx_buff;
			struct xge_buff *cp_buff;
		};
	};

	enum xgene_enet_ring_cfgsize cfgsize;
	struct xgene_enet_desc_ring *cp_ring;
	struct xgene_enet_desc_ring *buf_pool;
	union {
		void *desc_addr;
		struct xgene_enet_raw_desc *raw_desc;
		struct xgene_enet_raw_desc16 *raw_desc16;
	};

	bus_dma_tag_t dmat;
	bus_dmamap_t dmap;
	bus_addr_t dma;	/* This is in fact desc_paddr */
};

struct xgene_mac_ops {
	void (*init)(struct xgene_enet_pdata *pdata);
	void (*reset)(struct xgene_enet_pdata *pdata);
	void (*tx_enable)(struct xgene_enet_pdata *pdata);
	void (*rx_enable)(struct xgene_enet_pdata *pdata);
	void (*tx_disable)(struct xgene_enet_pdata *pdata);
	void (*rx_disable)(struct xgene_enet_pdata *pdata);
	void (*set_mac_addr)(struct xgene_enet_pdata *pdata);
	void (*get_mac_addr)(struct xgene_enet_pdata *pdata, u8 *dev_addr);
#if !defined(__FreeBSD__)
	void (*link_state)(struct work_struct *work);
#else
	u32 (*link_state)(struct xgene_enet_pdata *pdata);
#endif
};

struct xgene_port_ops {
	int (*reset)(struct xgene_enet_pdata *pdata);
	void (*cle_bypass)(struct xgene_enet_pdata *pdata,
			   u32 dst_ring_num, u16 bufpool_id);
	void (*shutdown)(struct xgene_enet_pdata *pdata);
};

/* ethernet private data */
struct xgene_enet_pdata {
	device_t ndev;
	uint8_t *hwaddr;
	int phy_speed;
	struct xgene_enet_desc_ring *tx_ring;
	struct xgene_enet_desc_ring *rx_ring;
	u32 rx_buff_cnt;
	u32 tx_qcnt_hi;
	u32 cp_qcnt_hi;
	u32 cp_qcnt_low;
	/* Those will serve as offsets for bus_read/write */
	bus_space_handle_t eth_csr_addr;
	bus_space_handle_t eth_ring_if_addr;
	bus_space_handle_t eth_diag_csr_addr;
	bus_space_handle_t mcx_mac_addr;
	bus_space_handle_t mcx_mac_csr_addr;
	/* Here we store resources instead of pointers to memory */
	struct resource *base_addr;
	struct resource *ring_csr_addr;
	struct resource *ring_cmd_addr;
	enum xgene_enet_rm rm;
	struct xgene_mac_ops *mac_ops;
	struct xgene_port_ops *port_ops;
	u32 port_id;
};

struct xgene_indirect_ctl {
	bus_space_handle_t addr;
	bus_space_handle_t ctl;
	bus_space_handle_t cmd;
	bus_space_handle_t cmd_done;
};

/* Set the specified value into a bit-field defined by its starting position
 * and length within a single u64.
 */
static inline u64 xgene_enet_set_field_value(int pos, int len, u64 val)
{
	return (val & ((1ULL << len) - 1)) << pos;
}

#define SET_VAL(field, val) \
		xgene_enet_set_field_value(field ## _POS, field ## _LEN, val)

#define SET_BIT(field) \
		xgene_enet_set_field_value(field ## _POS, 1, 1)

/* Get the value from a bit-field defined by its starting position
 * and length within the specified u64.
 */
static inline u64 xgene_enet_get_field_value(int pos, int len, u64 src)
{
	return (src >> pos) & ((1ULL << len) - 1);
}

#define GET_VAL(field, src) \
		xgene_enet_get_field_value(field ## _POS, field ## _LEN, src)

#endif /* __XGENE_ENET_MAIN_H__ */
