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

#include "opt_platform.h"

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

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "fdt_clock_if.h"
#include <dev/fdt/fdt_clock.h>

#include "xgene_clk_var.h"

#define	XGENE_CLK_SOCPLL_DEVSTR	"APM X-Gene1 SoC PLL"
#define	XGENE_CLK_PCPPLL_DEVSTR	"APM X-Gene1 Processor Complex PLL"
#define	XGENE_CLK_DEVICE_DEVSTR	"APM X-Gene1 Device Clock"
#define	XGENE_CLK_FFC_DEVSTR	"Fixed Factor Clock (dummy)"
#define	XGENE_CLK_FC_DEVSTR	"Fixed Clock (dummy)"

static int xgene_clk_fdt_probe(device_t);
static int xgene_clk_fdt_attach(device_t);
static int xgene_clk_fdt_detach(device_t);

static int xgene_clk_fdt_enable(device_t, int);
static int xgene_clk_fdt_disable(device_t, int);

static struct ofw_compat_data compat_data[] = {
	{"apm,xgene-socpll-clock", (uintptr_t)XGENE_CLK_SOCPLL_DEVSTR},
	{"apm,xgene-pcppll-clock", (uintptr_t)XGENE_CLK_PCPPLL_DEVSTR},
	{"apm,xgene-device-clock", (uintptr_t)XGENE_CLK_DEVICE_DEVSTR},
	{"fixed-factor-clock",	   (uintptr_t)XGENE_CLK_FFC_DEVSTR}, /* dummy */
	{"fixed-clock",		   (uintptr_t)XGENE_CLK_FFC_DEVSTR}, /* dummy */
	{NULL,				0},
};

static device_method_t xgene_clk_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		xgene_clk_fdt_probe),
	DEVMETHOD(device_attach,	xgene_clk_fdt_attach),
	DEVMETHOD(device_detach,	xgene_clk_fdt_detach),
	/* Clock methods */
	DEVMETHOD(fdt_clock_enable,	xgene_clk_fdt_enable),
	DEVMETHOD(fdt_clock_disable,	xgene_clk_fdt_disable),

	/* End */
	DEVMETHOD_END,
};

static driver_t xgene_clk_driver = {
	"xgene_clk",
	xgene_clk_methods,
	sizeof(struct xgene_clk_softc),
};

static devclass_t xgene_clk_devclass;

EARLY_DRIVER_MODULE(clk, simplebus, xgene_clk_driver, xgene_clk_devclass, 0, 0,
    BUS_PASS_TIMER + BUS_PASS_ORDER_MIDDLE);

static int
xgene_clk_fdt_probe(device_t dev)
{
	const struct ofw_compat_data *ocd;

	ocd = ofw_bus_search_compatible(dev, compat_data);
	if (ocd->ocd_str == NULL)
		return (ENXIO);

	device_set_desc(dev, (const char *)ocd->ocd_data);
	return (BUS_PROBE_DEFAULT);
}

static int
xgene_clk_fdt_attach(device_t dev)
{
	struct xgene_clk_softc *sc;
	phandle_t node;
	uint32_t *reg;
	char *reg_names;
	char *reg_temp;
	ssize_t nreg, nreg_names;
	vm_paddr_t paddr;
	u_long psize;
	int rid;
	int err;

	sc = device_get_softc(dev);

	node = ofw_bus_get_node(dev);
	if (node == 0)
		return (ENXIO);

	/*
	 * Special handling for Fixed Clock and Fixed Factor Clock.
	 * We just need a proper node in the tree of clock devices to reach all
	 * of the relevant clocks. Currently neither multiplier nor divider are
	 * saved in this driver for FFC.
	 */
	if (ofw_bus_is_compatible(dev, "fixed-clock") ||
	    ofw_bus_is_compatible(dev, "fixed-factor-clock")) {
		sc->clk_initialized = TRUE;
		err = 0;
		goto end;
	}

	nreg = OF_getencprop_alloc(node, "reg", sizeof(*reg), (void **)&reg);
	nreg = (nreg == -1) ? 0 : (nreg / sizeof(pcell_t));
	nreg_names = OF_getprop_alloc(node, "reg-names", 1, (void **)&reg_names);
	nreg_names = (nreg_names == -1) ? 0 : nreg_names;
	reg_temp = reg_names;

	for (rid = 0; rid < nreg; rid++) {
		if (nreg_names) {
			if (strncmp("div-reg", reg_temp, 7) == 0) {
				err = bus_get_resource(dev, SYS_RES_MEMORY,
				    rid, &paddr, &psize);
				if (err) {
					PRINTF_VERBOSE(dev,
					    "Could not get resource for "
					    "divisor registers\n");
					return (err);
				}

				err = bus_space_map(&memmap_bus, paddr, psize, 0,
				    &sc->clk_div_reg_h);
				if (err) {
					PRINTF_VERBOSE(dev,
					    "Could not map divisor registers\n");
					return (err);
				}

				/* Move to the next name in property */
				reg_temp += strlen(reg_temp) + 1;
				continue;
			}
		}

		err = bus_get_resource(dev, SYS_RES_MEMORY,
		    rid, &paddr, &psize);
		if (err) {
			PRINTF_VERBOSE(dev,
			    "Could not get resource for "
			    "control/status registers\n");
			return (err);
		}

		err = bus_space_map(&memmap_bus, paddr, psize, 0,
		    &sc->clk_csr_reg_h);
		if (err) {
			PRINTF_VERBOSE(dev,
			    "Could not map control/status registers\n");
			return (err);
		}

		/* Move to the next name in property */
		reg_temp += strlen(reg_temp) + 1;
	}

	err = xgene_clk_attach(dev);
end:
	if (!err)
		fdt_clock_register_provider(dev);

	return (err);
}

static int
xgene_clk_fdt_detach(device_t dev)
{

	return (0);
}

static __inline boolean_t
xgene_clk_fdt_have_parent(device_t clockdev)
{
	phandle_t cnode;

	cnode = ofw_bus_get_node(clockdev);
	if (OF_hasprop(cnode, "clocks"))
		return (TRUE);

	return (FALSE);
}

static int
xgene_clk_fdt_enable(device_t clockdev, int index __unused)
{
	int err = 0;

	/* Try to enable parent first */
	if (xgene_clk_fdt_have_parent(clockdev))
		err = fdt_clock_enable_all(clockdev);

	/* Now try to enable this clock */
	if (!err)
		return (xgene_clk_enable(clockdev));

	return (err);
}

static int
xgene_clk_fdt_disable(device_t clockdev, int index __unused)
{

	/* XXX: Does not disable parent clock */
	return (xgene_clk_disable(clockdev));
}
