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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/rman.h>
#include <sys/endian.h>
#include <sys/bus.h>
#include <sys/sockio.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>
#include <sys/buf_ring.h>
#include <sys/taskqueue.h>

#include <net/bpf.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_vlan_var.h>

#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <vm/pmap.h>

#include "if_xge_var.h"
#include "miibus_if.h"

/* Extremely verbose output! Use for low level debugging only. */
#define	XGE_DEBUG
#undef XGE_DEBUG

#define	RES_ENET_CSR	0	/* (M)ENET registers */
#define	RES_RING_CSR	1	/* QM(Lite) registers */
#define	RES_RING_CMD	2	/* QM(Lite) fabric area */
#define	RES_DEQ_IRQ	0	/* QM(Lite) dequeue interrupt */

/* Lowest number indicating message error */
#define	XGE_MSG_LERR_FIRST	0x3

#define	XGE_GLOBAL_LOCK_INIT(sc)					\
    mtx_init(&(sc)->globl_mtx, device_get_nameunit((sc)->dev),		\
	MTX_NETWORK_LOCK, MTX_DEF)

#define	XGE_GLOBAL_LOCK_DESTROY(sc)					\
    mtx_destroy(&(sc)->globl_mtx)

#define	XGE_GLOBAL_LOCK(sc)	mtx_lock(&(sc)->globl_mtx)
#define	XGE_GLOBAL_UNLOCK(sc)	mtx_unlock(&(sc)->globl_mtx)

#define	XGE_GLOBAL_LOCK_ASSERT(sc)					\
    mtx_assert(&(sc)->globl_mtx, MA_OWNED)

#define	XGE_SGMAC_LOCK(sc)						\
do {									\
	if ((sc)->phy_conn_type == PHY_CONN_SGMII)			\
		mtx_lock(&sgmac_mtx);					\
} while (0)

#define	XGE_SGMAC_UNLOCK(sc)						\
do {									\
	if ((sc)->phy_conn_type == PHY_CONN_SGMII)			\
		mtx_unlock(&sgmac_mtx);					\
} while (0)

#define	XGE_RING_DESC_ALIGNMENT		256	/* Has to be 256 bytes */
#define	XGE_RING_DESC_NSEGMENTS		1	/* Single segment */

#define	XGE_BUFF_ALIGNMENT		1
#define	XGE_BUFF_NSEGMENTS		1

#define	XGE_WD_TIMEOUT			5	/* Number of HZ ticks */
#define	XGE_TX_DONE_TH			15	/* Free Tx mbufs after this much
						   transmitted */

/* Multiple of Tx slots for Tx SW completion ring */
#define	XGE_TX_MBUFS_MULT		8
/* Multiple of Rx slots for Rx SW completion ring */
#define	XGE_RX_MBUFS_MULT		8

MALLOC_DEFINE(M_XGE, "xge", "X-Gene ENET dynamic memory");

extern int xgene_mii_phy_read(struct xgene_enet_pdata *, uint8_t, uint32_t);
extern int xgene_mii_phy_write(struct xgene_enet_pdata *, uint8_t, uint32_t,
    uint16_t);

static int xge_miibus_readreg(device_t, int, int);
static int xge_miibus_writereg(device_t, int, int, int);
static void xge_miibus_statchg(device_t);

static void xge_gmac_freq_set(struct xgene_enet_pdata *);

static int xge_sgmac_media_change(struct ifnet *);
static void xge_sgmac_media_status(struct ifnet *, struct ifmediareq *);
static int xge_xgmac_media_change(struct ifnet *);
static void xge_xgmac_media_status(struct ifnet *, struct ifmediareq *);

static void xge_mac_get_hwaddr(struct xge_softc *);
static void xge_delete_desc_rings(struct xge_softc *);
static void xge_qm_deq_intr(void *);
static void xge_init(void *);
static int xge_ioctl(struct ifnet *, u_long, caddr_t);
static void xge_txstart(struct ifnet *);
static void xge_txstart_locked(struct xge_softc *);
static void xge_stop_locked(struct xge_softc *);
static void xge_recycle_tx_locked(struct xge_softc *);
static void xge_tx_completion(struct xgene_enet_desc_ring *,
    struct xgene_enet_raw_desc *);
static void xge_do_rx(struct xge_softc *);
static void xge_rx_completion(struct xgene_enet_desc_ring *,
    struct xgene_enet_raw_desc *);
static int xge_gmac_media_change(struct ifnet *);
static void xge_gmac_media_status(struct ifnet *, struct ifmediareq *);
static void xge_tick(void *);
static int xge_init_hw(struct xge_softc *);
static int xge_setup_ops(struct xge_softc *);
static void xge_map_dma_addr(void *, bus_dma_segment_t *, int, int);

static int xge_shutdown(device_t);

static device_method_t xge_methods[] = {
	/* Device interface */
	DEVMETHOD(device_shutdown,	xge_shutdown),

	/* MII interface */
	DEVMETHOD(miibus_readreg,	xge_miibus_readreg),
	DEVMETHOD(miibus_writereg,	xge_miibus_writereg),
	DEVMETHOD(miibus_statchg,	xge_miibus_statchg),

	/* End */
	DEVMETHOD_END
};

DEFINE_CLASS_0(xge, xge_driver, xge_methods, sizeof(struct xge_softc));

DRIVER_MODULE(miibus, xge, miibus_driver, miibus_devclass, 0, 0);
MODULE_DEPEND(xge, ether, 1, 1, 1);
MODULE_DEPEND(xge, miibus, 1, 1, 1);

/******************************************************************************
 ************************* Early system initialization ************************
 ******************************************************************************/
static struct mtx sgmac_mtx;

static void
xge_early_init(void *dummy __unused)
{

	/* Lock to protect shared registers of SGMAC ports */
	mtx_init(&sgmac_mtx, "XGE SGMAC lock", NULL, MTX_DEF);
}
SYSINIT(xge_early_init, SI_SUB_DRIVERS, SI_ORDER_FIRST, xge_early_init, NULL);

/*****************************************************************************
 ***************************** Device methods ********************************
 *****************************************************************************/
int
xge_attach(device_t dev)
{
	struct xge_softc *sc = device_get_softc(dev);
	struct xgene_enet_pdata *pdata = &sc->pdata;
	bus_space_handle_t mac_offset;
	struct ifnet *ifp;
	int err;
	int rid;

	sc->dev = dev;

	/*
	 * Before doing anything check the connection type.
	 * This will allow us to avoid sanity checks later
	 * (if we pass this we can assume that the connection type is OK)
	 */
	switch (sc->phy_conn_type) {
	case PHY_CONN_RGMII:
	case PHY_CONN_SGMII:
	case PHY_CONN_XGMII:
		/* Those are valid types so continue to attach */
		break;
	default:
		device_printf(dev, "Unknown or nnvalid connection type\n");
		return (ENXIO);
	}

	/* Initialize global lock */
	XGE_GLOBAL_LOCK_INIT(sc);
	/* Initialize timeout */
	callout_init_mtx(&sc->timer_callout, &sc->globl_mtx, 0);
	/* Initialize watchdog - inactive */
	sc->wd_timeout = 0;

	/*
	 * Allocate resources
	 */
	/* Ethernet control and status register address space */
	rid = RES_ENET_CSR;
	sc->enet_csr = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	/* Descriptor ring control and status register address space */
	rid = RES_RING_CSR;
	sc->ring_csr = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	/* Descriptor ring command register address space */
	rid = RES_RING_CMD;
	sc->ring_cmd = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE | RF_SHAREABLE);

	if (!sc->enet_csr || !sc->ring_csr || !sc->ring_cmd) {
		device_printf(dev, "Could not allocate resource for %s\n",
		    !sc->enet_csr ? "ENET registers" :
		    !sc->ring_csr ? "QM registers" : "QM fabric area");

		xge_detach(dev);
		return (ENOMEM);
	}

	rid = RES_DEQ_IRQ;
	sc->qm_deq_irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    (RF_ACTIVE | RF_SHAREABLE));
	if (sc->qm_deq_irq == NULL) {
		device_printf(dev,
		    "Cannot allocate QM dequeue interrupt\n");

		xge_detach(dev);
		return (ENXIO);
	}

	/* Setup interrupts */
	err = bus_setup_intr(dev, sc->qm_deq_irq, INTR_TYPE_NET, NULL,
	    xge_qm_deq_intr, sc, &sc->qm_deq_irq_ihl);
	if (err != 0) {
		device_printf(dev,
		    "Cannot set-up QM dequeue interrupt\n");

		xge_detach(dev);
		return (ENXIO);
	}
	/*
	 * Acquire port ID for SGMII ports and set up mac_offset.
	 *
	 * Some of the ENET (SGMAC) registers' regions are shared
	 * among different MAC instances. Each MAC has its registers
	 * placed within a fixed offset from the MAC number 0.
	 * mac_offset is used to take this into account and
	 * HAS TO BE ZEROED for any other port (GMAC/XGMAC).
	 */
	if (sc->phy_conn_type == PHY_CONN_SGMII) {
		if (sc->portid == PORT_ID_INVALID ||
		    sc->portid > PORT_ID_MAX) {
			device_printf(dev, "Invalid port-id specified\n");
			return (ENXIO);
		}
		pdata->port_id = sc->portid;
		mac_offset = (pdata->port_id * MAC_OFFSET);
	} else {
		pdata->port_id = 0;
		mac_offset = 0;
	}

	/*
	 * XXX: For Linux HW layer this was a reference to
	 *      Linux net device. On FreeBSD however we pass
	 *      device_t to extract any necessary context from it.
	 */
	pdata->ndev = dev;

	pdata->hwaddr = sc->hwaddr;
	pdata->rx_buff_cnt = NUM_PKT_BUF;
	pdata->base_addr = sc->enet_csr;
	pdata->ring_csr_addr = sc->ring_csr;
	pdata->ring_cmd_addr = sc->ring_cmd;
	/* Frequently used offsets from base_addr */
	pdata->eth_csr_addr = BLOCK_ETH_CSR_OFFSET;
	pdata->eth_ring_if_addr = BLOCK_ETH_RING_IF_OFFSET;
	pdata->eth_diag_csr_addr = BLOCK_ETH_DIAG_CSR_OFFSET;

	if (sc->phy_conn_type == PHY_CONN_RGMII ||
	    sc->phy_conn_type == PHY_CONN_SGMII) {
		pdata->mcx_mac_addr = BLOCK_ETH_MAC_OFFSET + mac_offset;
		pdata->mcx_mac_csr_addr = BLOCK_ETH_MAC_CSR_OFFSET;
	} else {
		/* No need to add mac_offset here since this is always XGMAC */
		pdata->mcx_mac_addr = BLOCK_AXG_MAC_OFFSET;
		pdata->mcx_mac_csr_addr = BLOCK_AXG_MAC_CSR_OFFSET;
	}

#ifdef XGE_DEBUG
	device_printf(dev,
	    "enet_csr: 0x%016lx, ring_csr: 0x%016lx, ring_cmd: 0x%016lx\n",
	    rman_get_bushandle(sc->enet_csr),
	    rman_get_bushandle(sc->ring_csr),
	    rman_get_bushandle(sc->ring_cmd));
#endif

	/* Setup X-Gene ENET ops */
	err = xge_setup_ops(sc);
	if (err != 0) {
		device_printf(dev,
		    "Failed to set up MAC and PORT low-level callbacks\n");

		xge_detach(dev);
		return (err);
	}

	/* Get initial MAC address */
	xge_mac_get_hwaddr(sc);

	/* Configure ENET hardware */
	err = xge_init_hw(sc);
	if (err != 0) {
		device_printf(dev,
		    "Failed to perform hardware configuration\n");

		xge_detach(dev);
		return (err);
	}

	/* Allocate software completion ring for Rx mbufs. */
	sc->rx_mbufs = buf_ring_alloc(pdata->rx_ring->slots * XGE_RX_MBUFS_MULT,
	    M_XGE, M_WAITOK, &sc->globl_mtx);

	/* Allocate software completion ring for Tx mbufs. */
	sc->tx_mbufs = buf_ring_alloc(pdata->tx_ring->slots * XGE_TX_MBUFS_MULT,
	    M_XGE, M_WAITOK, &sc->globl_mtx);

	/* Allocate and set up the ethernet interface. */
	sc->ifp = ifp = if_alloc(IFT_ETHER);

	ifp->if_softc = sc;
	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX;
	/* TODO: Following features are not supported yet */
#if 0
	ifp->if_flags |= IFF_MULTICAST;
#endif
	ifp->if_capabilities = IFCAP_VLAN_MTU;
	/*
	 * TODO: HW checksum
	 *       Currently disabled.
	 */
	ifp->if_capenable = ifp->if_capabilities;

#ifdef DEVICE_POLLING
	/* XXX: This is not supported so cannot be used at the moment */
#error "DEVICE_POLLING not supported in XGE driver yet"
	/* Advertise that polling is supported */
	ifp->if_capabilities |= IFCAP_POLLING;
#endif
	ifp->if_mtu = ETHERMTU;

	ifp->if_init = xge_init;
	ifp->if_start = xge_txstart;
	ifp->if_ioctl = xge_ioctl;

	KASSERT((pdata->tx_qcnt_hi > 0),
	    ("Invalid number of available Tx descriptors: %d",
	    pdata->tx_qcnt_hi));

	ifp->if_snd.ifq_drv_maxlen = (pdata->tx_qcnt_hi - 1);
	IFQ_SET_MAXLEN(&ifp->if_snd, (pdata->tx_qcnt_hi - 1));
	IFQ_SET_READY(&ifp->if_snd);

	/* Attach PHYs */
	if (sc->phy_conn_type == PHY_CONN_RGMII) {
		err = mii_attach(dev, &sc->miibus, ifp, xge_gmac_media_change,
		    xge_gmac_media_status, BMSR_DEFCAPMASK, sc->phyaddr,
		    MII_OFFSET_ANY, 0);
		if (err != 0) {
			device_printf(dev, "Cannot attach PHY\n");

			xge_detach(dev);
			return (err);
		}
	} else if (sc->phy_conn_type == PHY_CONN_SGMII) {
		ifmedia_init(&sc->ifmedia, IFM_IMASK, xge_sgmac_media_change,
		    xge_sgmac_media_status);

		/*
		 * 10/100/1000 available. Full duplex only.
		 */
		ifmedia_add(&sc->ifmedia, (IFM_ETHER | IFM_10_T | IFM_FDX),
		    0, NULL);
		ifmedia_add(&sc->ifmedia, (IFM_ETHER | IFM_100_TX | IFM_FDX),
		    0, NULL);
		ifmedia_add(&sc->ifmedia, (IFM_ETHER | IFM_1000_T | IFM_FDX),
		    0, NULL);
		ifmedia_add(&sc->ifmedia, (IFM_ETHER | IFM_AUTO | IFM_FDX),
		    0, NULL);

		ifmedia_set(&sc->ifmedia, (IFM_ETHER | IFM_AUTO | IFM_FDX));

	} else if (sc->phy_conn_type == PHY_CONN_XGMII) {
		ifmedia_init(&sc->ifmedia, IFM_IMASK, xge_xgmac_media_change,
		    xge_xgmac_media_status);
		/*
		 * 10G available. Full duplex only.
		 */
		ifmedia_add(&sc->ifmedia, (IFM_ETHER | IFM_10G_LR | IFM_FDX),
		    0, NULL);
		ifmedia_add(&sc->ifmedia, (IFM_ETHER | IFM_10G_SR | IFM_FDX),
		    0, NULL);
		ifmedia_add(&sc->ifmedia, (IFM_ETHER | IFM_AUTO | IFM_FDX),
		    0, NULL);

		ifmedia_set(&sc->ifmedia, (IFM_ETHER | IFM_10G_SR | IFM_FDX));
	}

	ether_ifattach(ifp, sc->hwaddr);

	return (0);
}

int
xge_detach(device_t dev)
{
	struct xge_softc *sc;
	struct xgene_enet_pdata *pdata;
	int err, ret;

	sc = device_get_softc(dev);
	pdata = &sc->pdata;

	ret = 0;
	/* First try to disable interrupts */
	if (sc->qm_deq_irq_ihl != NULL) {
		err = bus_teardown_intr(dev, sc->qm_deq_irq,
		    sc->qm_deq_irq_ihl);
		if (err != 0) {
			device_printf(dev,
			    "Could not teardown QM dequeue interrupt\n");
			/* Save if this is the first error that occurred */
			ret = (ret == 0) ? err : ret;
		}
	}

	/* Disable Rx and Tx */
	if (pdata->mac_ops != NULL) {
		pdata->mac_ops->rx_disable(pdata);
		pdata->mac_ops->tx_disable(pdata);
	}

	/* Delete descriptors rings */
	xge_delete_desc_rings(sc);

	/* Then detach and shut down the networking interface */
	if (device_is_attached(dev)) {
		sc->ifp->if_flags &= ~IFF_UP;
		ether_ifdetach(sc->ifp);
		callout_drain(&sc->timer_callout);
	}

	switch (sc->phy_conn_type) {
	case PHY_CONN_RGMII:
		/* Release MII stuff */
		if (sc->miibus != NULL)
			device_delete_child(dev, sc->miibus);
		bus_generic_detach(dev);
		break;
	case PHY_CONN_SGMII: /* fall through */
	case PHY_CONN_XGMII:
		/* Remove all ifmedia configurations */
		ifmedia_removeall(&sc->ifmedia);
		break;
	default:
		/* Should not be possible to get here so just do nothing. */
		break;
	}

	/* Release interrupt resources */
	if (sc->qm_deq_irq != NULL) {
		err = bus_release_resource(dev, SYS_RES_IRQ, RES_DEQ_IRQ,
		    sc->qm_deq_irq);
		if (err != 0) {
			device_printf(dev,
			    "Could not release QM dequeue interrupt\n");
			/* Save if this is the first error that occurred */
			ret = (ret == 0) ? err : ret;
		}
	}
	/* Release all of the memory resources */
	if (sc->enet_csr != NULL) {
		err = bus_release_resource(dev, SYS_RES_MEMORY, RES_ENET_CSR,
		    sc->enet_csr);
		if (err != 0) {
			device_printf(dev,
			    "Could not release ENET registers resource\n");
			/* Save if this is the first error that occurred */
			ret = (ret == 0) ? err : ret;
		}
	}
	if (sc->ring_csr != NULL) {
		err = bus_release_resource(dev, SYS_RES_MEMORY, RES_RING_CSR,
		    sc->ring_csr);
		if (err != 0) {
			device_printf(dev,
			    "Could not release QM registers resource\n");
			/* Save if this is the first error that occurred */
			ret = (ret == 0) ? err : ret;
		}
	}
	if (sc->ring_cmd != NULL) {
		err = bus_release_resource(dev, SYS_RES_MEMORY, RES_RING_CMD,
		    sc->ring_cmd);
		if (err != 0) {
			device_printf(dev,
			    "Could not release QM fabric area resource\n");
			/* Save if this is the first error that occurred */
			ret = (ret == 0) ? err : ret;
		}
	}

	/* Free software ring buffers if any */
	if (sc->rx_mbufs != NULL)
		buf_ring_free(sc->rx_mbufs, M_XGE);
	if (sc->tx_mbufs != NULL)
		buf_ring_free(sc->tx_mbufs, M_XGE);
	/* Free ifnet structure */
	if (sc->ifp != NULL) {
		if_free(sc->ifp);
		sc->ifp = NULL;
	}

	/* Destroy locks */
	XGE_GLOBAL_LOCK_DESTROY(sc);

	/* Return first error that occured it there was any */
	return (ret);
}

static int
xge_shutdown(device_t dev)
{
	struct xge_softc *sc;

	sc = device_get_softc(dev);

	XGE_GLOBAL_LOCK(sc);
	xge_stop_locked(sc);
	XGE_GLOBAL_UNLOCK(sc);

	return (0);
}

/*****************************************************************************
 ************************ Not implemented routines ***************************
 *****************************************************************************/
static void
xge_set_promiscous(struct xge_softc *sc)
{

	/*
	 * XXX: Since classifier is bypassed all traffic should arrive
	 *       to the interface. Driver is currently uncapable of changing
	 *       this.
	 */
	device_printf(sc->dev, "Changing promiscous mode not implemented\n");
}

static void
xge_set_multicast(struct xge_softc *sc)
{

	device_printf(sc->dev, "Changing multicasting mode not implemented\n");
}

/*****************************************************************************
 ********************** MAC management helper routines ***********************
 *****************************************************************************/

/* Get initial MAC address */
static void
xge_mac_get_hwaddr(struct xge_softc *sc)
{
	struct xgene_enet_pdata *pdata = &sc->pdata;
	uint8_t hwaddr[ETHER_ADDR_LEN];
	uint8_t zeromac[ETHER_ADDR_LEN] = { [0 ... (ETHER_ADDR_LEN - 1)] = 0 };
	uint32_t rnd;

	/* First try to use default address set by FDT/ACPI glue */
	if (memcmp(zeromac, sc->hwaddr, ETHER_ADDR_LEN) != 0) {
		/* Got MAC address from FDT */
		return;
	}
	/*
	 * Now read addess from HW.
	 * It is assumed that if there is a non-zero value this means that
	 * the bootloader did the right thing and we keep that as a default.
	 *
	 * In case of missing default values set 'bsd' + random 24 low-order
	 * bits.
	 */
	pdata->mac_ops->get_mac_addr(pdata, hwaddr);
	if (memcmp(zeromac, hwaddr, ETHER_ADDR_LEN) == 0) {
		/* HW has zeroed MAC address, generate random one */
		rnd = arc4random() & 0x00ffffff;
		hwaddr[0] = 'b';
		hwaddr[1] = 's';
		hwaddr[2] = 'd';
		hwaddr[3] = rnd >> 16;
		hwaddr[4] = rnd >> 8;
		hwaddr[5] = rnd >> 0;
	}
	/* Save current MAC address to context */
	memcpy(sc->hwaddr, hwaddr, ETHER_ADDR_LEN);
}

static __inline void
xge_mac_reset(struct xge_softc *sc)
{
	struct xgene_enet_pdata *pdata = &sc->pdata;

	XGE_SGMAC_LOCK(sc);
	pdata->mac_ops->reset(pdata);
	XGE_SGMAC_UNLOCK(sc);
}

/*****************************************************************************
 ****************** Ring management helper routines **************************
 *****************************************************************************/

static __inline uint16_t
xge_get_ring_id(enum xgene_ring_owner owner, uint8_t bufnum)
{

	return ((owner << 6) | (bufnum & GENMASK(5, 0)));
}

static size_t
xge_get_ring_size(enum xgene_enet_ring_cfgsize cfgsize)
{
	size_t size;

	switch (cfgsize) {
	case RING_CFGSIZE_512B:
		size = 0x200;
		break;
	case RING_CFGSIZE_2KB:
		size = 0x800;
		break;
	case RING_CFGSIZE_16KB:
		size = 0x4000;
		break;
	case RING_CFGSIZE_64KB:
		size = 0x10000;
		break;
	case RING_CFGSIZE_512KB:
		size = 0x80000;
		break;
	default:
		size = 0;
		break;
	}

	return (size);
}

static __inline uint16_t
xge_dst_ring_num(struct xgene_enet_desc_ring *ring)
{
	struct xge_softc *sc;
	struct xgene_enet_pdata *pdata;

	sc = device_get_softc(ring->ndev);
	pdata = &sc->pdata;

	return (((uint16_t)pdata->rm << 10) | ring->num);
}

static uint32_t
xge_ring_len(struct xgene_enet_desc_ring *ring)
{
	struct xge_softc *sc;
	uint32_t ring_state, num_msgs;

	sc = device_get_softc(ring->ndev);

	/* TODO: Find better way to generate offset to the cmd_base[1] */
	ring_state =
	    RING_CMD_READ32(&sc->pdata, ring->cmd_base + sizeof(ring_state));
	num_msgs = ring_state & CREATE_MASK(NUMMSGSINQ_POS, NUMMSGSINQ_LEN);

	return (num_msgs >> NUMMSGSINQ_POS);
}

static struct xgene_enet_desc_ring *
xge_create_ring(struct xge_softc *sc, uint32_t ring_num, enum
    xgene_enet_ring_cfgsize cfgsize, uint32_t ring_id)
{
	struct xgene_enet_pdata *pdata;
	struct xgene_enet_desc_ring *ring;
	size_t size;
	int err;

	pdata = &sc->pdata;

	size = xge_get_ring_size(cfgsize);
	if (size == 0) {
		device_printf(sc->dev,
		    "Unsupported circular buffer size for message queue\n");
		return (NULL);
	}
	ring = malloc(sizeof(*ring), M_XGE, (M_WAITOK | M_ZERO));

	ring->ndev = sc->dev;
	ring->num = ring_num;
	ring->cfgsize = cfgsize;
	ring->id = ring_id;

	err = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),		/* parent */
	    XGE_RING_DESC_ALIGNMENT,		/* alignment */
	    0,					/* boundary */
	    BUS_SPACE_MAXADDR,			/* lowaddr */
	    BUS_SPACE_MAXADDR,			/* highaddr */
	    NULL, NULL,				/* filtfunc, filtfuncarg */
	    size,				/* maxsize */
	    XGE_RING_DESC_NSEGMENTS,		/* nsegments */
	    size,				/* maxsegsize */
	    0,					/* flags */
	    NULL, NULL,				/* lockfunc, lockfuncarg */
	    &ring->dmat);			/* dmat */
	if (err) {
		device_printf(sc->dev,
		    "Failed to allocate busdma tag for descriptors ring\n");
		goto dmatag_fail;
	}

	err = bus_dmamem_alloc(ring->dmat, &ring->desc_addr,
	    (BUS_DMA_NOWAIT | BUS_DMA_ZERO), &ring->dmap);
	if (err) {
		device_printf(sc->dev,
		    "Failed to allocate DMA safe memory for "
		    "descriptors ring\n");
		goto dmamem_fail;
	}

	err = bus_dmamap_load(ring->dmat, ring->dmap, ring->desc_addr, size,
	    xge_map_dma_addr, &ring->dma, BUS_DMA_NOWAIT);
	if (err) {
		device_printf(sc->dev,
		    "Cannot get physical address of descriptors ring\n");
		goto dmamap_fail;
	}

	ring->size = size;
	/* To be used as offsets from ring_cmd_addr resource */
	ring->cmd_base = (ring->num << 6);
	ring->cmd = ring->cmd_base + INC_DEC_CMD_ADDR;

	ring = xgene_enet_setup_ring(ring);

	if (bootverbose) {
		device_printf(sc->dev,
		    "ring info: num: %d, size: %d, id: %d, slots: %d\n",
		    ring->num, ring->size, ring->id, ring->slots);
	}

	return (ring);

dmamap_fail:
	bus_dmamem_free(ring->dmat, ring->desc_addr, ring->dmap);
dmamem_fail:
	bus_dma_tag_destroy(ring->dmat);
	ring->desc_addr = NULL;
dmatag_fail:
	free(ring, M_XGE);

	return (NULL);
}

static void
xge_delete_ring(struct xgene_enet_desc_ring *ring)
{

	xgene_enet_clear_ring(ring);
	bus_dmamem_free(ring->dmat, ring->desc_addr, ring->dmap);
	bus_dma_tag_destroy(ring->dmat);
	free(ring, M_XGE);
}

/*****************************************************************************
 ****************** Buffers and descriptors management ***********************
 *****************************************************************************/

static int
xge_create_dmaps(struct xgene_enet_desc_ring *ring, uint32_t nbuf)
{
	size_t i;
	int err;

	err = bus_dma_tag_create(
	    bus_get_dma_tag(ring->ndev),	/* parent */
	    XGE_BUFF_ALIGNMENT,			/* alignment */
	    0,					/* boundary */
	    BUS_SPACE_MAXADDR,			/* lowaddr */
	    BUS_SPACE_MAXADDR,			/* highaddr */
	    NULL, NULL,				/* filtfunc, filtfuncarg */
	    MCLBYTES,				/* maxsize */
	    XGE_BUFF_NSEGMENTS,			/* nsegments */
	    MCLBYTES,				/* maxsegsize */
	    0,					/* flags */
	    NULL, NULL,				/* lockfunc, lockfuncarg */
	    &ring->buf_dmat);			/* dmat */

	if (err != 0) {
		device_printf(ring->ndev,
		    "Failed to allocate busdma tag for buffers in ring\n");
		goto dmatag_fail;
	}

	for (i = 0; i < nbuf; i++) {
		err = bus_dmamap_create(ring->buf_dmat, 0,
		    &ring->buff[i].dmap);
		if (err != 0) {
			device_printf(ring->ndev,
			    "Failed to create busdma map for buffer\n");
			goto dmamap_fail;
		}
	}

	return (0);

dmamap_fail:
	for (; i > 0; i--)
		bus_dmamap_destroy(ring->buf_dmat, ring->buff[i].dmap);

	bus_dma_tag_destroy(ring->buf_dmat);
dmatag_fail:
	return (err);
}

static int
xge_delete_dmaps(struct xgene_enet_desc_ring *ring)
{
	int err;
	size_t i;

	for (i = 0; ring->buff[i].dmap != NULL; i++) {
		err = bus_dmamap_destroy(ring->buf_dmat, ring->buff[i].dmap);
		if (err != 0) {
			/*
			 * Deletion of DMA tag when DMA map is busy may cause
			 * random panic later.
			 */
			KASSERT(0, ("%s: Could not destroy DMA map for ring",
			    __func__));
			return (err);
		}
	}

	err = bus_dma_tag_destroy(ring->buf_dmat);

	return (err);
}

static int
xge_init_bufpool(struct xgene_enet_desc_ring *buf_pool)
{
	struct xgene_enet_raw_desc16 *raw_desc;
	size_t i;
	int err;

	err = xge_create_dmaps(buf_pool, buf_pool->slots);
	if (err != 0)
		return (err);

	for (i = 0; i < buf_pool->slots; i++) {
		raw_desc = &buf_pool->raw_desc16[i];

		/* Hardware expects descriptor in little endian format */
		/*
		 * XXX: Seems to be not necessary here since we don't
		 *      support BE anyway.
		 */
		raw_desc->m0 =
		    htole64(i | SET_VAL(FPQNUM, buf_pool->dst_ring_num) |
		    SET_VAL(STASH, 3));
	}

	buf_pool->tail = 0;

	return (0);
}

static int
xge_new_rxbuf(bus_dma_tag_t tag, bus_dmamap_t map, struct mbuf **mbufp,
    bus_addr_t *paddr)
{
	struct mbuf *new_mbuf;
	bus_dma_segment_t seg[1];
	int err;
	int nsegs;

	KASSERT(mbufp != NULL, ("NULL mbuf pointer!"));

	new_mbuf = m_getjcl(M_NOWAIT, MT_DATA, M_PKTHDR, MCLBYTES);
	if (new_mbuf == NULL)
		return (ENOBUFS);
	new_mbuf->m_len = new_mbuf->m_pkthdr.len = new_mbuf->m_ext.ext_size;

	if (*mbufp) {
		bus_dmamap_sync(tag, map, BUS_DMASYNC_POSTREAD);
		bus_dmamap_unload(tag, map);
	}

	err = bus_dmamap_load_mbuf_sg(tag, map, new_mbuf, seg, &nsegs,
	    BUS_DMA_NOWAIT);
	if ((nsegs != 1) || (err != 0)) {
		panic("%s: cannot dmamap_load mbuf, nsegs: %d, error: %d",
		    __func__, nsegs, err);
	}

	bus_dmamap_sync(tag, map, BUS_DMASYNC_PREREAD);

	(*mbufp) = new_mbuf;
	(*paddr) = seg->ds_addr;
	return (0);
}

static int
xge_refill_bufpool_locked(struct xgene_enet_desc_ring *buf_pool, uint32_t nbuf)
{
	struct xge_softc *sc;
	struct xgene_enet_raw_desc16 *raw_desc;
	bus_dma_tag_t dmat;
	bus_dmamap_t dmap;
	bus_addr_t *paddr;
	struct mbuf **mbuf;
	uint16_t tail, slots;
	uint16_t bufdatalen;
	size_t i;
	int err;

	sc = device_get_softc(buf_pool->ndev);

	XGE_GLOBAL_LOCK_ASSERT(sc);

	tail = buf_pool->tail;
	slots = buf_pool->slots - 1;

	/*
	 * XXX: Revise that since XGENE_ENET_MAX_MTU == 1536
	 *      and MCLBYTES is 2KB in length
	 */
	bufdatalen = BUF_LEN_CODE_2K | (MCLBYTES & GENMASK(11, 0));

	dmat = buf_pool->buf_dmat;

	for (i = 0; i < nbuf; i++) {
		raw_desc = &buf_pool->raw_desc16[tail];

		dmap = buf_pool->rx_buff[tail].dmap;
		mbuf = &buf_pool->rx_buff[tail].mbuf;
		paddr = &buf_pool->rx_buff[tail].paddr;

		err = xge_new_rxbuf(dmat, dmap, mbuf, paddr);
		if (err != 0) {
			/* XXX: We should handle this somehow different */
			panic("%s: cannot allocate new buffer", __func__);
			return (ENOBUFS);
		}

		raw_desc->m1 = htole64(SET_VAL(DATAADDR, *paddr) |
		    SET_VAL(BUFDATALEN, bufdatalen) | SET_BIT(COHERENT));

		tail = (tail + 1) & slots;
	}
	wmb();

	RING_CMD_WRITE32(&sc->pdata, buf_pool->cmd, nbuf);
	buf_pool->tail = tail;

	return (0);
}

static int
xge_refill_bufpool(struct xgene_enet_desc_ring *buf_pool, uint32_t nbuf)
{
	struct xge_softc *sc;
	int ret;

	sc = device_get_softc(buf_pool->ndev);
	XGE_GLOBAL_LOCK(sc);
	ret = xge_refill_bufpool_locked(buf_pool, nbuf);
	XGE_GLOBAL_UNLOCK(sc);

	return (ret);
}

static int
xge_create_desc_rings(struct xge_softc *sc)
{
	struct xgene_enet_pdata *pdata = &sc->pdata;
	struct xgene_enet_desc_ring *rx_ring, *tx_ring, *cp_ring;
	struct xgene_enet_desc_ring *buf_pool;
	uint16_t ring_id, ring_num;
	uint8_t cpu_bufnum, eth_bufnum, bp_bufnum;
	int err = 0;

	cpu_bufnum = pdata->cpu_bufnum;
	eth_bufnum = pdata->eth_bufnum;
	bp_bufnum = pdata->bp_bufnum;
	ring_num = pdata->ring_num;

	buf_pool = NULL;

	/* Allocate Rx descriptor ring (work queue) */
	ring_id = xge_get_ring_id(RING_OWNER_CPU, cpu_bufnum);
	cpu_bufnum++;

	rx_ring = xge_create_ring(sc, ring_num, RING_CFGSIZE_16KB,
	    ring_id);
	ring_num++;

	if (rx_ring == NULL) {
		device_printf(sc->dev, "Cannot allocate Rx descriptor ring\n");
		err = ENOMEM;
		goto error;
	}
	pdata->rx_ring = rx_ring;

	/* Allocate Rx buffers pool (free queue) */
	ring_id = xge_get_ring_id(RING_OWNER_ETH0, bp_bufnum);
	bp_bufnum++;
	buf_pool = xge_create_ring(sc, ring_num, RING_CFGSIZE_2KB,
	    ring_id);
	ring_num++;

	if (buf_pool == NULL) {
		device_printf(sc->dev, "Cannot allocate buffer pool\n");
		err = ENOMEM;
		goto error;
	}

	rx_ring->nbufpool = NUM_BUFPOOL;
	rx_ring->buf_pool = buf_pool;

	buf_pool->rx_buff = malloc((buf_pool->slots * sizeof(struct xge_buff)),
	    M_XGE, (M_WAITOK | M_ZERO));

	buf_pool->dst_ring_num = xge_dst_ring_num(buf_pool);

	/* Allocate Tx descriptor ring (work queue) */
	ring_id = xge_get_ring_id(RING_OWNER_ETH0, eth_bufnum);
	eth_bufnum++;
	tx_ring = xge_create_ring(sc, ring_num, RING_CFGSIZE_16KB,
	    ring_id);
	ring_num++;

	if (tx_ring == NULL) {
		device_printf(sc->dev, "Cannot allocate Tx descriptor ring\n");
		err = ENOMEM;
		goto error;
	}

	tx_ring->tx_buff = malloc((tx_ring->slots * sizeof(struct xge_buff)),
	    M_XGE, (M_WAITOK | M_ZERO));

	/* Allocate DMA maps for Tx buffers. */
	err = xge_create_dmaps(tx_ring, tx_ring->slots);
	if (err != 0) {
		device_printf(sc->dev, "Cannot create Tx DMA maps\n");
		goto error;
	}

	tx_ring->tail = 0;
	pdata->tx_ring = tx_ring;

	cp_ring = pdata->rx_ring;
	cp_ring->cp_buff = malloc((tx_ring->slots * sizeof(struct xge_buff)),
	    M_XGE, (M_WAITOK | M_ZERO));

	pdata->tx_ring->cp_ring = cp_ring;
	pdata->tx_ring->dst_ring_num = xge_dst_ring_num(cp_ring);

	pdata->tx_qcnt_hi = pdata->tx_ring->slots / 2;
	pdata->cp_qcnt_hi = pdata->rx_ring->slots / 2;
	pdata->cp_qcnt_low = pdata->cp_qcnt_hi / 2;

	sc->tx_enq_num = 0;

	return (0);

error:
	xge_delete_desc_rings(sc);
	return (err);
}

static void
xge_delete_desc_rings(struct xge_softc *sc)
{
	struct xgene_enet_pdata *pdata;
	struct xgene_enet_desc_ring *buf_pool;

	pdata = &sc->pdata;

	if (pdata->tx_ring != NULL) {
		if (xge_delete_dmaps(pdata->tx_ring) != 0) {
			device_printf(sc->dev,
			    "Could not delete DMA maps for Tx ring (busy)\n");
		}
		xge_delete_ring(pdata->tx_ring);
		pdata->tx_ring = NULL;
	}

	if (pdata->rx_ring != NULL) {
		buf_pool = pdata->rx_ring->buf_pool;
#if 0
		/* XXX: Not yet */
		xge_delete_bufpool(buf_pool);
#endif
		if (xge_delete_dmaps(buf_pool) != 0) {
			device_printf(sc->dev,
			    "Could not delete DMA maps for bufpool (busy)\n");
		}
		xge_delete_ring(buf_pool);
		xge_delete_ring(pdata->rx_ring);
		pdata->rx_ring = NULL;
	}
}

static boolean_t
is_rx_desc(struct xgene_enet_raw_desc *raw_desc)
{

	/*
	 * Rx descriptor will have its FPQNUM field
	 * (free pool queue number) set.
	 */
	return GET_VAL(FPQNUM, le64toh(raw_desc->m0)) ? TRUE : FALSE;
}

/*
 * Process Rx/completion ring.
 * Processes at most nbuf slots if nbuf > 0 or all occupied if nbuf < 0
 */
static int
xge_process_ring(struct xgene_enet_desc_ring *ring, int nbuf)
{
	struct xge_softc *sc;
	struct xgene_enet_raw_desc *raw_desc;
	uint16_t head;
	uint16_t slots;
	int count;

	sc = device_get_softc(ring->ndev);

	slots = ring->slots - 1;
	/* Head should point to the first occupied descriptor */
	head = ring->head;

	for (count = 0; nbuf != 0; nbuf--) {
		raw_desc = &ring->raw_desc[head];
		/* Synchronize descriptor */
		bus_dmamap_sync(ring->dmat, ring->dmap, BUS_DMASYNC_POSTREAD);
		/* Is this descriptor marked occupied? */
		if (xgene_enet_is_desc_slot_empty(raw_desc))
			break;

		/*
		 * Both Rx and Tx-completion messages land in rx_ring.
		 * Due to that we need to check and react properly according to
		 * the descriptor type.
		 */
		if (is_rx_desc(raw_desc))
			xge_rx_completion(ring, raw_desc);
		else
			xge_tx_completion(ring, raw_desc);

		xgene_enet_mark_desc_slot_empty(raw_desc);
		bus_dmamap_sync(ring->dmat, ring->dmap, BUS_DMASYNC_PREWRITE);

		head = (head + 1) & slots;
		count++;
	}

	if (count > 0) {
		/*
		 * Decrement number of busy
		 * descriptors in the corresponding ring.
		 */
		RING_CMD_WRITE32(&sc->pdata, ring->cmd, -count);
		ring->head = head;

		/*
		 * TODO: Restart transmission if stopped due to lack
		 *       of the Tx buffers.
		 */
	}

	return (count);
}

static void
xge_qm_deq_intr_locked(struct xge_softc *sc, int count)
{
	struct xgene_enet_pdata *pdata = &sc->pdata;

	XGE_GLOBAL_LOCK_ASSERT(sc);

	xge_process_ring(pdata->rx_ring, count);
}

static void
xge_qm_deq_intr(void *arg)
{
	struct xge_softc *sc = arg;

	XGE_GLOBAL_LOCK(sc);
	xge_qm_deq_intr_locked(sc, -1);
	/* Recycle Tx-ed mbufs */
	if (buf_ring_count(sc->tx_mbufs) > XGE_TX_DONE_TH)
		xge_recycle_tx_locked(sc);
	XGE_GLOBAL_UNLOCK(sc);
	xge_do_rx(sc);
}

/*****************************************************************************
 *************************** IFNET callbacks *********************************
 *****************************************************************************/
static void
xge_init_locked(struct xge_softc *sc)
{
	struct xgene_enet_pdata *pdata;
	struct xgene_mac_ops *mac_ops;
	struct ifnet *ifp;

	XGE_GLOBAL_LOCK_ASSERT(sc);

	pdata = &sc->pdata;
	mac_ops = pdata->mac_ops;
	ifp = sc->ifp;

	if (ifp->if_drv_flags & IFF_DRV_RUNNING)
		return;

	/* Enable Tx and Rx */
	mac_ops->tx_enable(pdata);
	mac_ops->rx_enable(pdata);

	/* Activate network interface */
	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

	/* Schedule timer callout */
	callout_reset(&sc->timer_callout, hz, xge_tick, sc);
}

static void
xge_init(void *if_softc)
{
	struct xge_softc *sc = if_softc;

	XGE_GLOBAL_LOCK(sc);
	xge_init_locked(sc);
	XGE_GLOBAL_UNLOCK(sc);
}

static int
xge_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct xge_softc *sc;
	struct mii_data *mii_sc;
	struct ifreq *ifr;
	uint32_t flags;
	int mask, err;

	sc = ifp->if_softc;
	ifr = (struct ifreq *)data;

	err = 0;
	switch (cmd) {
	case SIOCSIFFLAGS:
		XGE_GLOBAL_LOCK(sc);
		if (ifp->if_flags & IFF_UP) {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
				flags = ifp->if_flags ^ sc->if_flags;
				if ((sc->if_flags & ifp->if_flags) &
				    IFF_PROMISC) {
					/* Change promiscous mode */
					xge_set_promiscous(sc);
				}

				if ((sc->if_flags ^ ifp->if_flags) &
				    IFF_ALLMULTI) {
					/* Change multicasting settings */
					xge_set_multicast(sc);
				}
			} else
				xge_init_locked(sc);

		} else if (ifp->if_drv_flags & IFF_DRV_RUNNING)
				xge_stop_locked(sc);

		sc->if_flags = ifp->if_flags;
		XGE_GLOBAL_UNLOCK(sc);
		break;

	case SIOCADDMULTI:
	case SIOCDELMULTI:
		if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
			XGE_GLOBAL_LOCK(sc);
			xge_set_multicast(sc);
			XGE_GLOBAL_UNLOCK(sc);
		}
		break;

	case SIOCSIFMEDIA:
	case SIOCGIFMEDIA:
		if (sc->miibus != NULL) {
			mii_sc = device_get_softc(sc->miibus);
			err = ifmedia_ioctl(ifp, ifr, &mii_sc->mii_media, cmd);
		} else
			err = ifmedia_ioctl(ifp, ifr, &sc->ifmedia, cmd);

		break;

	case SIOCSIFCAP:
		mask = ifp->if_capenable ^ ifr->ifr_reqcap;
		if (mask & IFCAP_VLAN_MTU) {
			/* No work to do except acknowledge the change took. */
			ifp->if_capenable ^= IFCAP_VLAN_MTU;
		}
		break;

	default:
		err = ether_ioctl(ifp, cmd, data);
		break;
	}

	return (err);
}

static void
xge_txstart(struct ifnet *ifp)
{
	struct xge_softc *sc = ifp->if_softc;

	XGE_GLOBAL_LOCK(sc);
	xge_txstart_locked(sc);
	XGE_GLOBAL_UNLOCK(sc);
}

/*
 * Shut down the interface.
 * Not really an ifnet callback but fits here
 */
static void
xge_stop_locked(struct xge_softc *sc)
{
	struct xgene_enet_pdata *pdata;
	struct xgene_mac_ops *mac_ops;
	struct ifnet *ifp;

	XGE_GLOBAL_LOCK_ASSERT(sc);

	ifp = sc->ifp;
	pdata = &sc->pdata;
	mac_ops = pdata->mac_ops;

	/* Stop tick engine */
	callout_stop(&sc->timer_callout);
	/* Clean watchdog */
	sc->wd_timeout = 0;

	/* Disable interface */
	ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | IFF_DRV_OACTIVE);

	/* TODO: Disable interrupts? */

	/* Process ring */
	xge_process_ring(sc->pdata.rx_ring, -1);
	sc->tx_enq_num = 0;

	/* Disable Rx and Tx */
	mac_ops->tx_disable(pdata);
	mac_ops->rx_disable(pdata);
}

/*****************************************************************************
 ******************************** Tx path ************************************
 *****************************************************************************/
static uint64_t
xge_work_msg(struct mbuf *mb)
{
	struct ether_header *eth;
	struct ip *ip;
	struct tcphdr *tcp;
	uint8_t l3hlen, l4hlen;
	uint8_t csum_enable, proto, ethhdr;
	uint64_t hopinfo;

	l4hlen = proto = csum_enable = 0;

	eth = (struct ether_header *)mb->m_data;
	ip = (struct ip *)(mb->m_data + sizeof(struct ether_header));
	tcp = (struct tcphdr *)(ip + (ip->ip_hl << 2));

	if (ip->ip_p == IPPROTO_TCP) {
		l4hlen = tcp->th_off * 4;
		proto = TSO_IPPROTO_TCP;
		csum_enable = 1;
	} else if (ip->ip_p == IPPROTO_UDP) {
		l4hlen = UDP_HDR_SIZE;
		csum_enable = 1;
	}

	l3hlen = ip->ip_hl; /* IP header length in words (4 bytes) */
	ethhdr = ETHER_HDR_LEN;
	if (eth->ether_type == htons(ETHERTYPE_VLAN))
		ethhdr += ETHER_VLAN_ENCAP_LEN;

	/* XXX: Disable HW checksum until we support it */
	csum_enable = 0;

	hopinfo = SET_VAL(TCPHDR, l4hlen) |
		  SET_VAL(IPHDR, l3hlen) |
		  SET_VAL(ETHHDR, ethhdr) |
		  SET_VAL(EC, csum_enable) |
		  SET_VAL(IS, proto) |
		  SET_BIT(IC) |
		  SET_BIT(TYPE_ETH_WORK_MESSAGE);

	return (hopinfo);
}

static void
xge_setup_tx_desc(struct xgene_enet_desc_ring *tx_ring, struct mbuf *mb)
{
	struct xgene_enet_raw_desc *raw_desc;
	bus_dma_tag_t dmat;
	bus_dmamap_t dmap;
	bus_dma_segment_t seg[1];
	uint64_t hopinfo;
	int nsegs;
	uint16_t tail;
	int err;

	tail = tx_ring->tail;

	dmat = tx_ring->buf_dmat;
	/* Fetch unused dmap */
	dmap = tx_ring->tx_buff[tail].dmap;
	/* Create mapping in DMA memory */
	err = bus_dmamap_load_mbuf_sg(dmat, dmap, mb, seg, &nsegs,
	    BUS_DMA_NOWAIT);

	if ((nsegs != 1) || (err != 0)) {
		panic("%s: cannot dmamap_load mbuf, nsegs: %d, error: %d",
		    __func__, nsegs, err);
	}

	raw_desc = &tx_ring->raw_desc[tail];
	memset(raw_desc, 0, sizeof(struct xgene_enet_raw_desc));

	/* Hardware expects descriptor in little endian format */
	raw_desc->m0 = htole64(tail);
	raw_desc->m1 = htole64(
	    SET_VAL(DATAADDR, seg[0].ds_addr) |
	    SET_VAL(BUFDATALEN, seg[0].ds_len) |
	    SET_BIT(COHERENT));
	hopinfo = xge_work_msg(mb);
	raw_desc->m3 = htole64(SET_VAL(HENQNUM, tx_ring->dst_ring_num) |
	    hopinfo);
	tx_ring->cp_ring->cp_buff[tail].mbuf = mb;
	tx_ring->cp_ring->cp_buff[tail].dmap = dmap;
	tx_ring->cp_ring->cp_buff[tail].dmat = dmat;
	wmb();

#ifdef XGE_DEBUG
	size_t i;

	printf("%s: m_data phys: 0x%016lx, ds_addr: 0x%016lx\n", __func__,
	    vtophys(mb->m_data), seg[0].ds_addr);
	printf("%s: m_data contents:\n", __func__);
	for (i = 0; i < mb->m_len; i++) {
		printf("%02x ", mb->m_data[i]);
		if ((i % 8) == 0)
			printf(" ");
		if ((i % 16) == 0)
			printf("\n");
	}
#endif
}

static int
xge_encap(struct xge_softc *sc, struct mbuf *mb)
{
	struct xgene_enet_desc_ring *tx_ring;
	struct xgene_enet_desc_ring *cp_ring;
	uint32_t tx_level, cq_level;

	tx_ring = sc->pdata.tx_ring;
	cp_ring = tx_ring->cp_ring;

	tx_level = xge_ring_len(tx_ring);
	cq_level = xge_ring_len(cp_ring);

	if (tx_level > sc->pdata.tx_qcnt_hi ||
	    cq_level > sc->pdata.cp_qcnt_hi)
		return (ENOBUFS);

	xge_setup_tx_desc(tx_ring, mb);

	RING_CMD_WRITE32(&sc->pdata, tx_ring->cmd, 1);

#ifdef XGE_DEBUG
	struct xgene_enet_raw_desc *raw_desc;

	tx_level = xge_ring_len(tx_ring);
	cq_level = xge_ring_len(cp_ring);
	raw_desc = &tx_ring->raw_desc[tx_ring->tail];

	printf("\n%s: tx_level: %d, cq_level: %d\n", __func__,
	    tx_level, cq_level);
	printf("%s: raw_desc[0]: 0x%016lx\n", __func__, raw_desc->m0);
	printf("%s: raw_desc[1]: 0x%016lx\n", __func__, raw_desc->m1);
	printf("%s: raw_desc[2]: 0x%016lx\n", __func__, raw_desc->m2);
	printf("%s: raw_desc[3]: 0x%016lx\n", __func__, raw_desc->m3);
#endif

	tx_ring->tail = (tx_ring->tail + 1) & (tx_ring->slots - 1);
	sc->tx_enq_num++;

	return (0);
}

static void
xge_txstart_locked(struct xge_softc *sc)
{
	struct mbuf *m0, *mtmp;
	struct ifnet *ifp;
	int csum_flags;
	unsigned int queued;

	XGE_GLOBAL_LOCK_ASSERT(sc);

	ifp = sc->ifp;
	queued = 0;

	if ((ifp->if_drv_flags &
	    (IFF_DRV_RUNNING | IFF_DRV_OACTIVE)) != IFF_DRV_RUNNING)
		return;

	while (!IFQ_DRV_IS_EMPTY(&ifp->if_snd)) {
		/*
		 * Dequeue packets to the driver managed queue and
		 * return first mbuf here.
		 */
		IFQ_DRV_DEQUEUE(&ifp->if_snd, m0);
		if (m0 == NULL)
			break;

		csum_flags = m0->m_pkthdr.csum_flags;
		if (csum_flags != 0) {
			/* TODO: Use offloading features here */
		}

		mtmp = m_defrag(m0, M_NOWAIT);
		if (mtmp != NULL)
			m0 = mtmp;

		if (xge_encap(sc, m0) != 0) {
			IFQ_DRV_PREPEND(&ifp->if_snd, m0);
			ifp->if_drv_flags |= IFF_DRV_OACTIVE;
			break;
		}

		queued++;
		BPF_MTAP(ifp, m0);
	}

	if (queued) {
		if (sc->tx_enq_num != 0) {
			/* Set watchdog timeout (in hz ticks) */
			sc->wd_timeout = XGE_WD_TIMEOUT;
		}
	}
}

static void
xge_recycle_tx_locked(struct xge_softc *sc)
{
	struct mbuf *mb;

	XGE_GLOBAL_LOCK_ASSERT(sc);

	mb = buf_ring_dequeue_sc(sc->tx_mbufs);
	while (mb != NULL) {
		m_free(mb);
		sc->tx_enq_num--;
		/*
		 * If everything is transmitted then disable watchdog and break.
		 * Keep in mind that xge_txstart() needs to be protected by
		 * the same lock as this function.
		 */
		if (sc->tx_enq_num == 0) {
			sc->wd_timeout = 0;
			break;
		}
		mb = buf_ring_dequeue_sc(sc->tx_mbufs);
	}
}

static void
xge_tx_completion(struct xgene_enet_desc_ring *cp_ring,
    struct xgene_enet_raw_desc *raw_desc)
{
	struct xge_softc *sc;
	bus_dma_tag_t dmat;
	bus_dmamap_t dmap;
	struct mbuf *mb;
	uint16_t desc_idx;
	uint8_t status;

	sc = device_get_softc(cp_ring->ndev);

	/*
	 * Get USERINFO field where 'tail'
	 * number of the Tx descriptor was written.
	 */
	desc_idx = GET_VAL(USERINFO, le64toh(raw_desc->m0));

	/* Unmap mbuf */
	mb = cp_ring->cp_buff[desc_idx].mbuf;
	dmap = cp_ring->cp_buff[desc_idx].dmap;
	dmat = cp_ring->cp_buff[desc_idx].dmat;
	bus_dmamap_unload(dmat, dmap);
	/*
	 * Enqueue already transmitted mbuf to the
	 * software completion queue for further processing.
	 */
	if (__predict_false(buf_ring_enqueue(sc->tx_mbufs, mb) != 0)) {
		/*
		 * Cannot postpone this action for later. Free the mbuf now.
		 * Disable watchdog if everything is transmitted.
		 */
		m_free(mb);
		sc->tx_enq_num--;
		if (sc->tx_enq_num == 0)
			sc->wd_timeout = 0;
	}

	status = GET_VAL(LERR, le64toh(raw_desc->m0));
	if (status >= XGE_MSG_LERR_FIRST) {
		/*
		 * TODO: Parse errors properly.
		 *       For now just increase packet counters.
		 */
#if 0
		xgene_enet_parse_error(cp_ring, pdata, status);
#endif
		if_inc_counter(sc->ifp, IFCOUNTER_OERRORS, 1);
		return;
	}

	if_inc_counter(sc->ifp, IFCOUNTER_OPACKETS, 1);
}

/*****************************************************************************
 ********************************** Rx path **********************************
 *****************************************************************************/
static void
xge_do_rx(struct xge_softc *sc)
{
	struct ifnet *ifp;
	struct mbuf *mb;

	ifp = sc->ifp;

	mb = buf_ring_dequeue_mc(sc->rx_mbufs);
	while (mb != NULL) {
		mb->m_pkthdr.rcvif = ifp;

		m_fixhdr(mb);
		m_adj(mb, -ETHER_CRC_LEN);

		(*ifp->if_input)(ifp, mb);
		mb = buf_ring_dequeue_mc(sc->rx_mbufs);
	}
}

static void
xge_rx_completion(struct xgene_enet_desc_ring *rx_ring,
    struct xgene_enet_raw_desc *raw_desc)
{
	struct xge_softc *sc;
	struct ifnet *ifp;
	struct xgene_enet_desc_ring *buf_pool;
	bus_dma_tag_t dmat;
	bus_dmamap_t dmap;
	struct mbuf *mb;
	uint16_t desc_idx;
	uint8_t status;

	sc = device_get_softc(rx_ring->ndev);
	ifp = sc->ifp;

	/* Get descriptor bufpool index */
	desc_idx = GET_VAL(USERINFO, le64toh(raw_desc->m0));

	/* Sync Rx buffer */
	buf_pool = rx_ring->buf_pool;
	dmap = buf_pool->rx_buff[desc_idx].dmap;
	dmat = buf_pool->rx_buff[desc_idx].dmat;
	bus_dmamap_sync(dmat, dmap, BUS_DMASYNC_POSTREAD);
	bus_dmamap_unload(dmat, dmap);

	mb = buf_pool->rx_buff[desc_idx].mbuf;
	if (mb == NULL)
		panic("%s: Rx descriptor bound to NULL mbuf", __func__);

	/* Set the proper amount of data in this mbuf */
	mb->m_len = GET_VAL(BUFDATALEN, le64toh(raw_desc->m1));
	/*
	 * Enqueue received mbuf to the
	 * software completion queue for further processing.
	 */
	if (__predict_false(buf_ring_enqueue(sc->rx_mbufs, mb) != 0)) {
		/*
		 * Not enough Rx slots in software ring.
		 * Discard this packet.
		 */
		if_inc_counter(ifp, IFCOUNTER_IQDROPS, 1);
		goto out;
	}

	status = GET_VAL(LERR, le64toh(raw_desc->m0));
	if (status >= XGE_MSG_LERR_FIRST) {
		/*
		 * TODO: Parse errors properly.
		 *       For now just increase packet counters.
		 */
#if 0
		xgene_enet_parse_error(rx_ring, pdata, status);
#endif
		if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);
		goto out;
	}

	if_inc_counter(ifp, IFCOUNTER_IPACKETS, 1);

out:
	if (--rx_ring->nbufpool == 0) {
		xge_refill_bufpool_locked(buf_pool, NUM_BUFPOOL);
		rx_ring->nbufpool = NUM_BUFPOOL;
	}
}

/*****************************************************************************
 ****************************** MII interface ********************************
 *****************************************************************************/
static int
xge_miibus_readreg(device_t dev, int phy, int reg)
{
	struct xge_softc *sc = device_get_softc(dev);
	struct xgene_enet_pdata *pdata = &sc->pdata;
	int ret;

	ret = xgene_mii_phy_read(pdata, phy, reg);
	return ((ret < 0) ? 0 : ret);
}

static int
xge_miibus_writereg(device_t dev, int phy, int reg, int val)
{
	struct xge_softc *sc = device_get_softc(dev);
	struct xgene_enet_pdata *pdata = &sc->pdata;
	int ret;

	ret = xgene_mii_phy_write(pdata, phy, reg, val);

	return ((ret < 0) ? -ret : 0);
}

static void
xge_miibus_statchg(device_t dev)
{
	struct xge_softc *sc;
	struct mii_data *mii;
	struct xgene_enet_pdata *pdata;
	struct xgene_mac_ops *mac_ops;

	sc = device_get_softc(dev);
	pdata = &sc->pdata;
	mac_ops = pdata->mac_ops;

	/*
	 * This routine is called as a result of
	 * mii_mediachg() so lock should be acquired
	 */
	XGE_GLOBAL_LOCK_ASSERT(sc);

	mii = device_get_softc(sc->miibus);

	if ((mii->mii_media_status & IFM_ACTIVE) != 0)
		sc->link_is_up = TRUE;
	else {
		sc->link_is_up = FALSE;
		mac_ops->rx_disable(pdata);
		mac_ops->tx_disable(pdata);
		return;
	}

	/* If we got here it means that link is up */
	switch (IFM_SUBTYPE(mii->mii_media_active)) {
	case IFM_1000_T:
		pdata->phy_speed = SPEED_1000;
		break;
	case IFM_100_TX:
		pdata->phy_speed = SPEED_100;
		break;
	case IFM_10_T:
		pdata->phy_speed = SPEED_10;
		break;
	default:
		pdata->phy_speed = SPEED_UNKNOWN;
		break;
	}

	if (pdata->phy_speed != SPEED_UNKNOWN) {
		xge_gmac_freq_set(pdata);

		mac_ops->init(pdata);
		mac_ops->rx_enable(pdata);
		mac_ops->tx_enable(pdata);
	}
}

static int
xge_gmac_media_change(struct ifnet *ifp)
{
	struct xge_softc *sc;
	struct mii_data *mii_sc;
	int err;

	sc = ifp->if_softc;
	mii_sc = device_get_softc(sc->miibus);

	XGE_GLOBAL_LOCK(sc);
	err = mii_mediachg(mii_sc);
	XGE_GLOBAL_UNLOCK(sc);

	return (err);
}

static void
xge_gmac_media_status(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct xge_softc *sc;
	struct mii_data *mii_sc;

	sc = ifp->if_softc;
	mii_sc = device_get_softc(sc->miibus);

	XGE_GLOBAL_LOCK(sc);
	mii_pollstat(mii_sc);
	ifmr->ifm_active = mii_sc->mii_media_active;
	ifmr->ifm_status = mii_sc->mii_media_status;
	XGE_GLOBAL_UNLOCK(sc);
}

static void
xge_gmac_freq_set(struct xgene_enet_pdata *pdata)
{
	struct xge_softc *sc;

	sc = device_get_softc(pdata->ndev);

	switch (pdata->phy_speed) {
	case SPEED_1000:
		bus_space_write_4(sc->ethclk_bst, sc->ethclk_bsh,
		    SCU_ETHDIV_ADDR, 0x8);
		break;
	case SPEED_100:
		bus_space_write_4(sc->ethclk_bst, sc->ethclk_bsh,
		    SCU_ETHDIV_ADDR, 0x28);
		break;
	case SPEED_10:
		bus_space_write_4(sc->ethclk_bst, sc->ethclk_bsh,
		    SCU_ETHDIV_ADDR, 0x190);
		break;
	default:
		/* Should not happen but... */
		device_printf(pdata->ndev,
		    "Trying to set unsupported speed on RGMII port: %d\n",
		    pdata->phy_speed);
		break;
	}

}
/*****************************************************************************
 ****************************** Ifmedia **************************************
 *****************************************************************************/

/*
 * Same routine used by both sgmac and xgmac
 */
static boolean_t
xge_sx_gmac_update_status_locked(struct ifnet *ifp)
{
	struct xge_softc *sc;
	struct xgene_enet_pdata *pdata;
	struct xgene_mac_ops *mac_ops;
	boolean_t link_was_up, link_is_up;

	sc = ifp->if_softc;
	pdata = &sc->pdata;
	mac_ops = pdata->mac_ops;

	XGE_GLOBAL_LOCK_ASSERT(sc);
	link_was_up = sc->link_is_up;
	link_is_up = !!mac_ops->link_state(pdata);

	if (link_is_up) {
		if (!link_was_up) {
			if_link_state_change(ifp, LINK_STATE_UP);
			XGE_SGMAC_LOCK(sc);
			mac_ops->init(pdata);
			XGE_SGMAC_UNLOCK(sc);

			mac_ops->rx_enable(pdata);
			mac_ops->tx_enable(pdata);
		}

	} else {
		if (link_was_up) {
			if_link_state_change(ifp, LINK_STATE_DOWN);
			mac_ops->rx_disable(pdata);
			mac_ops->tx_disable(pdata);
		}
	}

	sc->link_is_up = link_is_up;

	return (link_is_up);
}

static void
xge_sgmac_media_status(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct xge_softc *sc;
	struct xgene_enet_pdata *pdata;
	sc = ifp->if_softc;
	boolean_t link_is_up;

	sc = ifp->if_softc;
	pdata = &sc->pdata;

	XGE_GLOBAL_LOCK(sc);
	link_is_up = xge_sx_gmac_update_status_locked(ifp);

	ifmr->ifm_status = IFM_AVALID;
	ifmr->ifm_active = IFM_ETHER;

	if (!link_is_up) {
		XGE_GLOBAL_UNLOCK(sc);
		return;
	}

	/* Device attached to working network */
	ifmr->ifm_status |= IFM_ACTIVE;

	switch (pdata->phy_speed) {
	case SPEED_10:
		ifmr->ifm_active |= IFM_10_T;
		break;
	case SPEED_100:
		ifmr->ifm_active |= IFM_100_TX;
		break;
	case SPEED_1000:
		ifmr->ifm_active |= IFM_1000_T;
		break;
	default:
		ifmr->ifm_active |= IFM_AUTO;
		break;
	}

	/* Always full duplex */
	ifmr->ifm_active |= IFM_FDX;

	XGE_GLOBAL_UNLOCK(sc);
}

static int
xge_sgmac_media_change(struct ifnet *ifp __unused)
{

	/* XXX: We can't really change anything until HW layer allows that */
	return (0);
}

static void
xge_xgmac_media_status(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct xge_softc *sc;
	sc = ifp->if_softc;
	boolean_t link_is_up;

	sc = ifp->if_softc;

	XGE_GLOBAL_LOCK(sc);
	link_is_up = xge_sx_gmac_update_status_locked(ifp);

	ifmr->ifm_status = IFM_AVALID;
	ifmr->ifm_active = IFM_ETHER;

	if (link_is_up) {
		/* Device attached to working network */
		ifmr->ifm_status |= IFM_ACTIVE;
	}
	XGE_GLOBAL_UNLOCK(sc);
}

static int
xge_xgmac_media_change(struct ifnet *ifp __unused)
{

	/* Not needed since we operate in 10G mode only */
	return (0);
}

/*****************************************************************************
 ****************************** Watchdog *************************************
 *****************************************************************************/
static void
xge_watchdog(struct xge_softc *sc)
{
	struct xgene_enet_pdata *pdata;

	XGE_GLOBAL_LOCK_ASSERT(sc);

	if ((sc->wd_timeout == 0) || (--sc->wd_timeout > 0))
		return;

	pdata = &sc->pdata;
	xge_process_ring(sc->pdata.rx_ring, -1);
	xge_recycle_tx_locked(sc);
	if (sc->tx_enq_num == 0)
		return;

	if_printf(sc->ifp, "watchdog timeout\n");

	/* Restart controller */
	xge_stop_locked(sc);
	xge_mac_reset(sc);
	xge_init_locked(sc);
}

static void
xge_tick(void *arg)
{
	struct xge_softc *sc;
	struct mii_data *mii_sc;
	struct ifnet *ifp;
	boolean_t link_was_up;

	sc = arg;
	ifp = sc->ifp;

	XGE_GLOBAL_LOCK_ASSERT(sc);
	/* Disable watchdog or reset port */
	xge_watchdog(sc);

	link_was_up = sc->link_is_up;

	switch (sc->phy_conn_type) {
	case PHY_CONN_RGMII:
		mii_sc = device_get_softc(sc->miibus);
		mii_tick(mii_sc);
		break;
	case PHY_CONN_SGMII: /* fall through */
	case PHY_CONN_XGMII:
		xge_sx_gmac_update_status_locked(ifp);
		break;
	default:
		panic("%s: callout from unsupported port type", __func__);
	}

	if (!link_was_up && sc->link_is_up && !IFQ_DRV_IS_EMPTY(&ifp->if_snd)) {
		/* Start transmission here since the link went up */
		xge_txstart_locked(sc);
	}

	/* Schedule another check one second from now. */
	callout_reset(&sc->timer_callout, hz, xge_tick, sc);
}

/*****************************************************************************
 *************************** Low-level configuration *************************
 *****************************************************************************/
static int
xge_init_hw(struct xge_softc *sc)
{
	struct xgene_enet_pdata *pdata;
	struct xgene_enet_desc_ring *buf_pool;
	uint16_t dst_ring_num;
	int err;

	pdata = &sc->pdata;
	/* Port reset */
	err = pdata->port_ops->reset(pdata);
	if (err != 0)
		return (err);

	/* Allocate DMA ring buffers */
	err = xge_create_desc_rings(sc);
	if (err != 0)
		return (err);

	/* Set-up buffer pool */
	buf_pool = pdata->rx_ring->buf_pool;
	err = xge_init_bufpool(buf_pool);
	if (err != 0) {
		device_printf(sc->dev,
		    "Failed to initialize Rx buffers pool\n");
		goto error;
	}
	err = xge_refill_bufpool(buf_pool, pdata->rx_buff_cnt);
	if (err != 0) {
		device_printf(sc->dev, "Failed to refill Rx buffers pool\n");
		goto error;
	}

	dst_ring_num = xge_dst_ring_num(pdata->rx_ring);
	pdata->port_ops->cle_bypass(pdata, dst_ring_num, buf_pool->id);

	return (0);
error:
	xge_delete_desc_rings(sc);
	return (err);
}

static int
xge_setup_ops(struct xge_softc *sc)
{
	struct xgene_enet_pdata *pdata = &sc->pdata;

	switch (sc->phy_conn_type) {
	case PHY_CONN_RGMII:
		pdata->mac_ops = &xgene_gmac_ops;
		pdata->port_ops = &xgene_gport_ops;
		pdata->rm = RM3;
		break;
	case PHY_CONN_SGMII:
		pdata->mac_ops = &xgene_sgmac_ops;
		pdata->port_ops = &xgene_sgport_ops;
		pdata->rm = RM1;
		break;
	case PHY_CONN_XGMII:
		pdata->mac_ops = &xgene_xgmac_ops;
		pdata->port_ops = &xgene_xgport_ops;
		pdata->rm = RM0;
		break;
	default:
		device_printf(sc->dev, "Unsupported connection type");
		return (ENXIO);
	}

	switch (pdata->port_id) {
	case 0:
		pdata->cpu_bufnum = START_CPU_BUFNUM_0;
		pdata->eth_bufnum = START_ETH_BUFNUM_0;
		pdata->bp_bufnum = START_BP_BUFNUM_0;
		pdata->ring_num = START_RING_NUM_0;
		break;
	case 1:
		pdata->cpu_bufnum = START_CPU_BUFNUM_1;
		pdata->eth_bufnum = START_ETH_BUFNUM_1;
		pdata->bp_bufnum = START_BP_BUFNUM_1;
		pdata->ring_num = START_RING_NUM_1;
		break;
	default:
		device_printf(sc->dev, "Unsupported %s port ID: %d\n",
		    (sc->phy_conn_type == PHY_CONN_RGMII) ? "RGMII" :
		    (sc->phy_conn_type == PHY_CONN_SGMII) ? "SGMII" : "XGMII",
		    pdata->port_id);
		return (ENXIO);
	}

	return (0);
}

/*****************************************************************************
 ******************* Various (unaffiliated) routines *************************
 *****************************************************************************/

/* Callback for bus_dmamap_load() */
static void
xge_map_dma_addr(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	bus_addr_t *paddr;

	KASSERT(nseg == 1, ("wrong number of segments, should be 1"));
	paddr = arg;
	*paddr = segs->ds_addr;
}
