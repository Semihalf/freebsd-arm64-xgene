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

#include <machine/bus.h>
#include <machine/fdt.h>
#include <machine/resource.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "if_xge_var.h"

#define	XGE_DEBUG
#undef	XGE_DEBUG

static void xge_fdt_get_macaddr(device_t);
static int xge_fdt_map_clocks(device_t);
static void xge_fdt_unmap_clocks(device_t);

static int xge_fdt_probe(device_t);
static int xge_fdt_attach(device_t);
static int xge_fdt_detach(device_t);

static device_method_t xge_fdt_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		xge_fdt_probe),
	DEVMETHOD(device_attach,	xge_fdt_attach),
	DEVMETHOD(device_detach,	xge_fdt_detach),

	/* End */
	DEVMETHOD_END
};

DEFINE_CLASS_1(xge, xge_fdt_driver, xge_fdt_methods, sizeof(struct xge_softc),
    xge_driver);

static devclass_t xge_fdt_devclass;

DRIVER_MODULE(xge, simplebus, xge_fdt_driver, xge_fdt_devclass, 0, 0);

static int
xge_fdt_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "apm,xgene-enet") &&
	    !ofw_bus_is_compatible(dev, "apm,xgene1-sgenet") &&
	    !ofw_bus_is_compatible(dev, "apm,xgene1-xgenet"))
		return (ENXIO);

	device_set_desc(dev, XGE_DEVSTR);
	return (BUS_PROBE_DEFAULT);
}

static int
xge_fdt_attach(device_t dev)
{
	struct xge_softc *sc;
	phandle_t node;
	phandle_t phy, portid;
	char phy_conn_type[16];
	int ret;

	sc = device_get_softc(dev);
	sc->dev = dev;

	node = ofw_bus_get_node(dev);
	if (node == -1) {
		device_printf(dev, "Cannot find ofw bus node\n");
		return (ENXIO);
	}

	sc->phy_conn_type = PHY_CONN_UNKNOWN;
	if (OF_searchprop(node, "phy-connection-type", phy_conn_type,
	    sizeof(phy_conn_type)) != -1) {
		if (strncasecmp(phy_conn_type, PHY_CONN_RGMII_STR,
		    sizeof((char)PHY_CONN_RGMII_STR)) == 0) {
			/* RGMII connection */
			sc->phy_conn_type = PHY_CONN_RGMII;
		} else if (strncasecmp(phy_conn_type, PHY_CONN_SGMII_STR,
		    sizeof((char)PHY_CONN_SGMII_STR)) == 0) {
			/* SGMII connection */
			sc->phy_conn_type = PHY_CONN_SGMII;
		} else if (strncasecmp(phy_conn_type, PHY_CONN_XGMII_STR,
		    sizeof((char)PHY_CONN_XGMII_STR)) == 0) {
			/* XGMII connection */
			sc->phy_conn_type = PHY_CONN_XGMII;
		}
	}

	if (sc->phy_conn_type == PHY_CONN_UNKNOWN) {
		device_printf(dev, "PHY connection type invalid "
		    "or not found in Device Tree\n");
		return (ENXIO);
	}

	/* Get phy address from FDT */
	if (sc->phy_conn_type == PHY_CONN_RGMII) {
		if (OF_getencprop(node, "phy-handle", &phy, sizeof(phy)) <= 0) {
			device_printf(dev, "PHY not found in device tree\n");
			sc->phyaddr = MII_PHY_ANY;
		} else {
			phy = OF_node_from_xref(phy);
			if (OF_getencprop(phy,
			    "reg", &sc->phyaddr, sizeof(sc->phyaddr)) <= 0) {
				device_printf(dev, "Cannot retrieve PHY address\n");
				sc->phyaddr = MII_PHY_ANY;
			}
		}
	}

	/* Get port id from FDT */
	if (OF_getencprop(node, "port-id", &portid, sizeof(portid)) <= 0) {
		/* Set invalid port-id */
		sc->portid = PORT_ID_INVALID;
	} else
		sc->portid = portid;


#ifdef XGE_DEBUG
	printf("\tPHY connection type: %s\n",
	    (sc->phy_conn_type == PHY_CONN_RGMII) ?
	    PHY_CONN_RGMII_STR : PHY_CONN_UNKNOWN_STR);
#endif

	xge_fdt_get_macaddr(dev);

	/* Map resources for clocks if necessary */
	ret = xge_fdt_map_clocks(dev);
	if (ret != 0) {
		device_printf(dev,
		    "Could not map required clocks' registers\n");
		return (ret);
	}

	ret = xge_attach(dev);
	if (ret != 0)
		xge_fdt_unmap_clocks(dev);

	return (ret);
}

static int
xge_fdt_detach(device_t dev)
{
	struct xge_softc *sc;

	sc = device_get_softc(dev);

	xge_fdt_unmap_clocks(dev);

	return (xge_detach(dev));
}

static void
xge_fdt_get_macaddr(device_t dev)
{
	struct xge_softc *sc;
	phandle_t node;
	uint8_t	addr[ETHER_ADDR_LEN];

	sc = device_get_softc(dev);

	node = ofw_bus_get_node(dev);
	if (node == 0)
		goto err;
	if (OF_getprop(node, "local-mac-address", addr, ETHER_ADDR_LEN) == -1)
		goto err;

	/* Save default HW address */
	memcpy(sc->hwaddr, addr, ETHER_ADDR_LEN);

	return;

err:
	device_printf(dev, "Cannot retrieve MAC address from node\n");
}

/*
 * XXX: This hackish function is needed since we don't have any generic
 *      layer that would parse and configure clocks as we wish in the driver.
 *      Instead we find and set up what we need basing on FDT and use this in
 *      the driver later.
 *
 *      To be removed when appropriate framework for clocks management is
 *      available.
 */
static int
xge_fdt_map_clocks(device_t dev)
{
	struct xge_softc *sc;
	bus_space_handle_t bsh;
	phandle_t node, parent;
	phandle_t clks;
	pcell_t *reg, addr_cells, size_cells;
	vm_paddr_t paddr;
	vm_size_t size;
	char *oname;
	int num;
	size_t i;
	int ret;

	sc = device_get_softc(dev);
	sc->ethclk_bsh = 0;

	reg = NULL;
	node = ofw_bus_get_node(dev);

	/* Find ethclk and map its registers */
	if (sc->phy_conn_type == PHY_CONN_RGMII) {
		ret = 0;
		/*
		 * Currently no devices are created for the clock nodes
		 * not being under any bus so we need to find them here manually.
		 */
		while (OF_getencprop(node, "clocks", &clks, sizeof(clks)) > 0) {
			/* Recurse to get ethclk */
			node = OF_node_from_xref(clks);
			num = OF_getprop_alloc(node, "clock-output-names",
			    sizeof(*oname), (void **)&oname);
			if (num < 0)
				continue;
			if (strncasecmp(oname, "ethclk", num) != 0) {
				free(oname, M_OFWPROP);
				continue;
			}
			free(oname, M_OFWPROP);

			parent = OF_parent(node);
			if (parent == 0) {
				ret = ENXIO;
				break;
			}

			addr_cells = 2;
			OF_getencprop(parent, "#address-cells", &addr_cells,
			    sizeof(addr_cells));

			size_cells = 2;
			OF_getencprop(parent, "#size-cells", &size_cells,
			    sizeof(size_cells));

			num = OF_getencprop_alloc(node, "reg", sizeof(*reg),
			    (void **)&reg);
			if (num < 0) {
				ret = ENXIO;
				break;
			}

			if (num % (addr_cells + size_cells)) {
				ret = ENXIO;
				break;
			}

			paddr = size = 0;
			for (i = 0; i < addr_cells; i++) {
				paddr <<= 32;
				paddr |= reg[i];
			}

			for (i = 0; i < size_cells; i++) {
				size <<= 32;
				size |= reg[addr_cells + i];
			}

			/*
			 * At this point we found the ethclk regs so we may
			 * proceed to map them and save in software context.
			 */
			if (bus_space_map(fdtbus_bs_tag, paddr, size, 0,
			    &bsh) != 0) {
				sc->ethclk_bsh = 0;
				ret = ENXIO;
				device_printf(dev,
				    "Could not map ETHCLK registers\n");
				break;
			}
			sc->ethclk_bsh = bsh;
			sc->ethclk_bst = fdtbus_bs_tag;
			sc->ethclk_bsz = size;
			break;
		}
		free(reg, M_OFWPROP);
		return (ret);
	} else
		return (0);
}

static void
xge_fdt_unmap_clocks(device_t dev)
{
	struct xge_softc *sc;

	sc = device_get_softc(dev);

	if (sc->phy_conn_type == PHY_CONN_RGMII) {
		bus_space_unmap(sc->ethclk_bst, sc->ethclk_bsh, sc->ethclk_bsz);
		sc->ethclk_bsh = 0;
	}
}
