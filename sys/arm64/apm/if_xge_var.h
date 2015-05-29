/*-
 * Copyright (c) 2015 AppliedMicro Inc
 * All rights reserved.
 *
 * Developed by Semihalf.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
  * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#ifndef __IF_XGE_VAR_H__
#define	__IF_XGE_VAR_H__

#include "xgene_enet_main.h"

DECLARE_CLASS(xge_driver);

#define	ENET_CSR_WRITE32(pdata, off, val)		\
    bus_write_4((pdata)->base_addr, (off), (val))

#define	ENET_CSR_READ32(pdata, off)			\
    bus_read_4((pdata)->base_addr, (off))

#define	RING_CSR_WRITE32(pdata, off, val)		\
    bus_write_4((pdata)->ring_csr_addr, (off), (val))

#define	RING_CSR_READ32(pdata, off)			\
    bus_read_4((pdata)->ring_csr_addr, (off))

#define	RING_CMD_WRITE32(pdata, off, val)		\
    bus_write_4((pdata)->ring_cmd_addr, (off), (val))

#define	RING_CMD_READ32(pdata, off)			\
    bus_read_4((pdata)->ring_cmd_addr, (off))

enum phy_conn_type {
	PHY_CONN_UNKNOWN,
	PHY_CONN_RGMII,
	PHY_CONN_SGMII,
};

#define	PHY_CONN_UNKNOWN_STR	"Unknown"
#define	PHY_CONN_RGMII_STR	"RGMII"

struct xge_buff {
	bus_dma_tag_t		dmat;
	bus_dmamap_t		dmap;
	struct mbuf *		mbuf;
	bus_addr_t		paddr;
};

struct xge_softc {
	device_t		dev;
	device_t		miibus;
	struct mtx		globl_mtx;

	struct ifnet *		ifp;

	int			wd_timeout;
	struct callout		wd_callout;
	enum phy_conn_type	phy_conn_type;
	int			phyaddr;
	uint8_t			hwaddr[ETHER_ADDR_LEN];
	uint32_t		if_flags;

	struct xgene_enet_pdata	pdata;

	struct resource *	enet_csr;
	struct resource *	ring_csr;
	struct resource *	ring_cmd;

	struct resource *	qm_deq_irq;
	void *			qm_deq_irq_ihl;

	struct buf_ring *	rx_mbufs;	/* Buffers received */

	struct buf_ring *	tx_mbufs;	/* Buffers transmitted */
	uint32_t		tx_enq_num;	/* Buffers num enqueued to Tx */
};

int xge_attach(device_t);

int xge_miibus_readreg(device_t dev, int, int);
int xge_miibus_writereg(device_t dev, int, int, int);
void xge_miibus_statchg(device_t dev);

#endif /* !__IF_XGE_VAR_H__ */
