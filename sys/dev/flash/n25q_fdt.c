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
#include <sys/bio.h>
#include <sys/kthread.h>
#include <sys/sysctl.h>
#include <sys/malloc.h>

#include <geom/geom_disk.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/spibus/spi.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include "spibus_if.h"
#include "bus_if.h"

#include "n25q_var.h"

static struct ofw_compat_data compat_data[] = {
	{"n25q256a", (uintptr_t)N25Q_TYPE_256_DEVSTR},
	{NULL, 0}
};

static int n25q_fdt_probe(device_t);

static device_method_t n25q_fdt_methods[] = {
	DEVMETHOD(device_probe, n25q_fdt_probe),

	DEVMETHOD_END
};

DEFINE_CLASS_1(n25q, n25q_fdt_driver, n25q_fdt_methods,
    sizeof(struct n25q_softc), n25q_driver);

static devclass_t n25q_fdt_devclass;

DRIVER_MODULE(n25q, spibus, n25q_driver, n25q_devclass, NULL, NULL);

static int
n25q_fdt_probe(device_t dev)
{
	const struct ofw_compat_data *ocd;

	ocd = ofw_bus_search_compatible(dev, compat_data);
	if (ocd->ocd_str == NULL)
		return (ENXIO);

	device_set_desc(dev, (const char *)ocd->ocd_data);
	return (BUS_PROBE_DEFAULT);
}
