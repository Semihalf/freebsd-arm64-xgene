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

#include <sys/param.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/proc.h>
#include <sys/condvar.h>
#include <sys/systm.h>
#include <sys/sysctl.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/spibus/spi.h>
#include <dev/spibus/spibusvar.h>
#include <machine/fdt.h>

#include <dev/fdt/fdt_common.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include "spibus_if.h"
#include "bus_if.h"

#include "xgene_spi_var.h"

static int xgene_spi_fdt_probe(device_t);
static phandle_t xgene_spi_get_node(device_t, device_t);

static device_method_t xgene_spi_fdt_methods[] = {
	DEVMETHOD(device_probe,		xgene_spi_fdt_probe),

	DEVMETHOD(ofw_bus_get_node,	xgene_spi_get_node),

	DEVMETHOD_END
};

DEFINE_CLASS_1(xgene_spi, xgene_spi_fdt_driver, xgene_spi_fdt_methods,
    sizeof(struct xgene_spi_softc), xgene_spi_driver);

static devclass_t xgene_spi_fdt_devclass;

DRIVER_MODULE(xgene_spi, simplebus, xgene_spi_fdt_driver, xgene_spi_devclass, 0, 0);
MODULE_DEPEND(xgene_spi, spibus, 1, 1, 1);

static int
xgene_spi_fdt_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev) ||
	    !ofw_bus_is_compatible(dev, "apm,xgene-spi"))
		return (ENXIO);

	device_set_desc(dev, SPI_DEVSTR);

	return (BUS_PROBE_DEFAULT);
}

static __inline phandle_t
xgene_spi_get_node(device_t bus, device_t dev)
{

	return (ofw_bus_get_node(bus));
}
