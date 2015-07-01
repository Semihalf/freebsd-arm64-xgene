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
#include <machine/resource.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "if_xge_var.h"

#define	XGE_DEBUG
#undef	XGE_DEBUG

#define	XGE_DEVSTR	"APM X-Gene Ethernet Controller"

static void xge_fdt_get_macaddr(device_t);

static int xge_fdt_probe(device_t);
static int xge_fdt_attach(device_t);

static device_method_t xge_fdt_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		xge_fdt_probe),
	DEVMETHOD(device_attach,	xge_fdt_attach),

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
	    !ofw_bus_is_compatible(dev, "apm,xgene1-sgenet"))
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
		}
	}

	if (sc->phy_conn_type == PHY_CONN_UNKNOWN) {
		device_printf(dev, "PHY connection type invalid "
		    "or not found in Device Tree\n");
		return (ENXIO);
	}

	/* Get phy address from FDT */
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

	/* Get port id from FDT */
	if (OF_getencprop(node, "port-id", &portid, sizeof(portid)) <= 0) {
		/* Set invalid port-id */
		sc->portid = PORT_ID_INVALID;
	} else
		sc->portid = portid;


#ifdef DEBUG
	printf("\tPHY connection type: %s\n",
	    (sc->phy_conn_type == PHY_CONN_RGMII) ?
	    PHY_CONN_RGMII_STR : PHY_CONN_UNKNOWN_STR);
#endif

	xge_fdt_get_macaddr(dev);

	return (xge_attach(dev));
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
