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
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/rman.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/cpuset.h>
#include <sys/limits.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_pci.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcib_private.h>

#include <machine/cpu.h>
#include <machine/bus.h>
#include <machine/fdt.h>
#include <machine/intr.h>

#include <dev/fdt/fdt_common.h>

#include "pcib_if.h"
#include "xgene_pcie_var.h"

static int xgene_pcie_fdt_probe(device_t self);
static int xgene_pcie_fdt_attach(device_t self);

static device_method_t xgene_pcie_fdt_methods[] = {
	DEVMETHOD(device_probe,		xgene_pcie_fdt_probe),
	DEVMETHOD(device_attach,	xgene_pcie_fdt_attach),

	DEVMETHOD_END
};

DEFINE_CLASS_1(pcie, xgene_pcie_fdt_driver, xgene_pcie_fdt_methods,
		sizeof(struct xgene_pcie_softc), xgene_pcie_driver);

static devclass_t xgene_pcie_fdt_devclass;

DRIVER_MODULE(pcib, simplebus, xgene_pcie_fdt_driver,ro
		xgene_pcie_fdt_devclass, 0, 0);
DRIVER_MODULE(pcib, ofwbus, xgene_pcie_fdt_driver,
		xgene_pcie_fdt_devclass, 0, 0);
//MODULE_DEPEND(xgene_pcie, pci, 1, 1, 1);

static int
xgene_pcie_fdt_probe(device_t self)
{
	if (!ofw_bus_status_okay(self))
		return (ENXIO);

	if (!ofw_bus_is_compatible(self, "apm,xgene-pcie"))
		return (ENXIO);

	device_set_desc(self, "APM X-Gene Integrated PCI-Express Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
xgene_pcie_fdt_attach(device_t self)
{
	struct xgene_pcie_softc *sc;
	u_int retval = 0;
	uint32_t flags;
	phandle_t node;
	pcell_t pci_addr_cells, parent_addr_cells, size_cells;
	pcell_t *ranges_buf, *cell_ptr;
	int cells_count, tuples_count, touple;
	struct range *range;

	sc = device_get_softc(self);
	sc->dev = self;

	node = ofw_bus_get_node(self);

	if (fdt_addrsize_cells(node, &pci_addr_cells, &size_cells) != 0)
			return (ENXIO);

	parent_addr_cells = fdt_parent_addr_cells(node);

	if (parent_addr_cells != 2 || pci_addr_cells != 3 || size_cells != 2) {
		device_printf(sc->dev, "ERROR: "
			"unexpected values of address or size cells in FDT\n");
		return (ENXIO);
	}

	/*
	 * Get 'ranges' property
	 */
	cells_count = OF_getprop_alloc(node, "ranges",
		sizeof(pcell_t), (void **)&ranges_buf);
	if (cells_count == -1) {
		device_printf(sc->dev, "ERROR: wrong FDT 'ranges' property\n");
		return (ENXIO);
	}

	tuples_count = cells_count /
			(pci_addr_cells + parent_addr_cells + size_cells);
	if (tuples_count != 2) {
		device_printf(sc->dev,
			"ERROR: unexpected number of 'ranges' tuples in FDT\n");
		retval = ENXIO;
		goto out;
	}

	bzero(&sc->pci_res.io, sizeof(sc->pci_res.io));
	bzero(&sc->pci_res.mem, sizeof(sc->pci_res.mem));

	cell_ptr = ranges_buf;
	for (touple = 0; touple < tuples_count; touple++) {
		flags = fdt_data_get((void *)cell_ptr, 1);

		if (flags & OFW_PCI_PHYS_HI_SPACE_IO)
			range = &sc->pci_res.io;
		else if (flags & OFW_PCI_PHYS_HI_SPACE_MEM32)
			range = &sc->pci_res.mem;
		else {
			retval = ERANGE;
			goto out;
		}

		range->flags = flags;
		cell_ptr++;
		range->pci_addr = fdt_data_get((void *)cell_ptr, 2);
		cell_ptr += 2;
		range->cpu_addr = fdt_data_get((void *)cell_ptr, 2);
		cell_ptr += 2;
		range->size = fdt_data_get((void *)cell_ptr, 2);
		cell_ptr += 2;

		if (bootverbose) {
			device_printf(sc->dev,
				"\tCPU: 0x%016lx..0x%016lx -> PCI: 0x%016lx\n",
				range->cpu_addr,
				range->cpu_addr + range->size - 1,
				range->pci_addr);
		}
	}
	free(ranges_buf, M_OFWPROP);

	/*
	 * Get 'dma-ranges' property
	 */
	cells_count = OF_getprop_alloc(node, "dma-ranges",
			sizeof(pcell_t), (void **)&ranges_buf);
	if (cells_count == -1) {
		device_printf(sc->dev,
			"ERROR: wrong FDT 'dma-ranges' property\n");
		return (ENXIO);
	}
	tuples_count = cells_count /
				(pci_addr_cells + parent_addr_cells + size_cells);
	if (tuples_count != XGENE_DMA_RANGES_COUNT) {
		device_printf(sc->dev, "ERROR: "
			"unexpected number of 'dma-ranges' tuples in FDT\n");
		retval = ENXIO;
		goto out;
	}

	for (int i = 0 ; i < XGENE_DMA_RANGES_COUNT; i++) {
		bzero(&sc->pci_res.dma_ranges.dma[i],
			sizeof(sc->pci_res.dma_ranges.dma[i]));
	}

	cell_ptr = ranges_buf;
	for (touple = 0; touple < tuples_count; touple++) {
		if (touple < XGENE_DMA_RANGES_COUNT)
			range = &sc->pci_res.dma_ranges.dma[touple];
		else {
			retval = ERANGE;
			goto out;
		}

		range->flags = fdt_data_get((void *)cell_ptr, 1);
		cell_ptr++;
		range->pci_addr = fdt_data_get((void *)cell_ptr, 2);
		cell_ptr += 2;
		range->cpu_addr = fdt_data_get((void *)cell_ptr, 2);
		cell_ptr += 2;
		range->size = fdt_data_get((void *)cell_ptr, 2);
		cell_ptr += 2;

		if (bootverbose) {
			device_printf(sc->dev,
				"\tDMA%d: 0x%016lx..0x%016lx -> PCI: 0x%016lx\n",
				touple,
				range->cpu_addr,
				range->cpu_addr + range->size - 1,
				range->pci_addr);
		}

	}

	return (xgene_pcie_attach(self));

out:
	free(ranges_buf, M_OFWPROP);
	return (retval);
}

