/*-
 * Copyright (c) 2014 The FreeBSD Foundation
 * All rights reserved.
 *
 * This software was developed by Semihalf under
 * the sponsorship of the FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/* PCIe root complex driver for Cavium Thunder SOC */

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
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcib_private.h>
#include <machine/cpu.h>
#include <machine/bus.h>
#include <machine/fdt.h>
#include <machine/intr.h>
#include <dev/fdt/fdt_common.h>

#include "pcib_if.h"

/* Assembling ECAM Configuration Address */
#define PCIE_BUS_SHIFT	20
#define PCIE_SLOT_SHIFT 15
#define PCIE_FUNC_SHIFT 12
#define PCIE_BUS_MASK	0xFF
#define PCIE_SLOT_MASK	0x1F
#define PCIE_FUNC_MASK	0x07
#define PCIE_REG_MASK	0xFFF

#define PCIE_ADDR_OFFSET(bus, slot, func, reg)			\
	((((bus) & PCIE_BUS_MASK) << PCIE_BUS_SHIFT)	|	\
	(((slot) & PCIE_SLOT_MASK) << PCIE_SLOT_SHIFT)	|	\
	(((func) & PCIE_FUNC_MASK) << PCIE_FUNC_SHIFT)	|	\
	((reg) & PCIE_REG_MASK))

#define ECAM_COUNT		4
#define MAX_RANGES_TUPLES	3
#define MIN_RANGES_TUPLES	2

#define THUNDER_ECAM0_CFG_BASE		0x848000000000UL
#define THUNDER_ECAM1_CFG_BASE		0x849000000000UL
#define THUNDER_ECAM2_CFG_BASE		0x84a000000000UL
#define THUNDER_ECAM3_CFG_BASE		0x84b000000000UL

struct pcie_range {
	uint64_t	base;
	uint64_t	size;
};

struct thunder_pcie_softc {
	struct pcie_range	ranges[ECAM_COUNT][MAX_RANGES_TUPLES];
	struct rman		mem_rman;
	struct resource		*res;
	bus_space_tag_t		bst;
	bus_space_handle_t      bsh;
	device_t		dev;
};

/* Forward prototypes */
static int thunder_pcie_probe(device_t dev);
static int thunder_pcie_attach(device_t dev);
static int parse_pci_mem_ranges(struct thunder_pcie_softc *sc, int ecam);
static uint32_t thunder_pcie_read_config(device_t dev, u_int bus, u_int slot,
    u_int func, u_int reg, int bytes);
static void thunder_pcie_write_config(device_t dev, u_int bus, u_int slot,
    u_int func, u_int reg, uint32_t val, int bytes);
static int thunder_pcie_maxslots(device_t dev);
static int thunder_pcie_read_ivar(device_t dev, device_t child, int index,
    uintptr_t *result);
static int thunder_pcie_write_ivar(device_t dev, device_t child, int index,
    uintptr_t value);
static struct resource *thunder_pcie_alloc_resource(device_t dev,
    device_t child, int type, int *rid, u_long start, u_long end,
    u_long count, u_int flags);
static int thunder_pcie_release_resource(device_t dev, device_t child,
    int type, int rid, struct resource *res);
static int thunder_pcie_get_ecam_domain(device_t dev, int *domain);
static int thunder_pcie_map_msi(device_t pcib, device_t child, int irq,
    uint64_t *addr, uint32_t *data);
static int thunder_pcie_alloc_msix(device_t pcib, device_t child, int *irq);
static int thunder_pcie_release_msix(device_t pcib, device_t child, int irq);

static int
thunder_pcie_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_is_compatible(dev, "cavium,thunder-pcie")) {
		device_set_desc(dev, "Cavium Integrated PCI/PCI-E Controller");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
thunder_pcie_attach(device_t dev)
{
	int rid;
	struct thunder_pcie_softc *sc;
	int error;
	int ecam; /* pcie root complex number */
	int tuple;
	uint64_t base, size;

	sc = device_get_softc(dev);
	sc->dev = dev;

	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "could not map memory.\n");
		return (ENXIO);
	}

	if (thunder_pcie_get_ecam_domain(dev, &ecam)) {
		device_printf(dev, "could not determine domain.\n");
		return (ENXIO);
	}

	sc->bst = rman_get_bustag(sc->res);
	sc->bsh = rman_get_bushandle(sc->res);

	sc->mem_rman.rm_type = RMAN_ARRAY;
	sc->mem_rman.rm_descr = "PCIe Memory";

	/* Retrieve 'ranges' property from FDT */

	if (parse_pci_mem_ranges(sc, ecam))
		return (ENXIO);

	/* Initialize rman and allocate memory regions */

	error = rman_init(&sc->mem_rman);
	if (error) {
		device_printf(dev, "rman_init() failed. error = %d\n", error);
		return (error);
	}

	for (tuple = 0; tuple < MAX_RANGES_TUPLES; tuple++) {
		base = sc->ranges[ecam][tuple].base;
		size = sc->ranges[ecam][tuple].size;
		if (base == 0 || size == 0)
			continue; /* empty range element */

		error = rman_manage_region(&sc->mem_rman, base, base + size - 1);
		if (error) {
			device_printf(dev, "rman_manage_region() failed. error = %d\n", error);
			rman_fini(&sc->mem_rman);
			return (error);
		}
	}

	device_add_child(dev, "pci", -1);
	return (bus_generic_attach(dev));
}

static int
parse_pci_mem_ranges(struct thunder_pcie_softc *sc, int ecam)
{
	phandle_t node;
	pcell_t pci_addr_cells, parent_addr_cells, size_cells;
	pcell_t *ranges_buf, *cell_ptr;
	int cells_count, tuples_count;
	int tuple;
	int rv;

	node = ofw_bus_get_node(sc->dev);

	if (ecam > (ECAM_COUNT - 1) || ecam < 0) {
		device_printf(sc->dev, "Unexpected ECAM number\n");
		return (ENXIO);
	}

	if (fdt_addrsize_cells(node, &pci_addr_cells, &size_cells))
		return (ENXIO);

	parent_addr_cells = fdt_parent_addr_cells(node);
	if (parent_addr_cells != 2 || pci_addr_cells != 3 || size_cells != 2) {
		device_printf(sc->dev,
		    "Unexpected number of address or size cells in FDT\n");
		return (ENXIO);
	}

	cells_count = OF_getprop_alloc(node, "ranges",
	    sizeof(pcell_t), (void **)&ranges_buf);
	if (cells_count == -1) {
		device_printf(sc->dev, "Error parsing FDT 'ranges' property\n");
		return (ENXIO);
	}

	tuples_count = cells_count /
	    (pci_addr_cells + parent_addr_cells + size_cells);
	if (tuples_count > MAX_RANGES_TUPLES || tuples_count < MIN_RANGES_TUPLES) {
		device_printf(sc->dev,
		    "Unexpected number of 'ranges' tuples in FDT\n");
		rv = ENXIO;
		goto out;
	}

	cell_ptr = ranges_buf;

	if (bootverbose) {
		device_printf(sc->dev, "parsing FDT for ECAM%d:\n",
		    ecam);
	}

	for (tuple = 0; tuple < tuples_count; tuple++) {
		cell_ptr += pci_addr_cells; /* move ptr to parent addr */
		sc->ranges[ecam][tuple].base = fdt_data_get((void *)cell_ptr, 2);
		cell_ptr += parent_addr_cells; /* move ptr to size cells*/
		sc->ranges[ecam][tuple].size = fdt_data_get((void *)cell_ptr, 2);
		cell_ptr += size_cells; /* move ptr to next tuple*/

		if (bootverbose) {
			device_printf(sc->dev, "\tBase: 0x%jx, Size: 0x%jx\n",
			    sc->ranges[ecam][tuple].base,
			    sc->ranges[ecam][tuple].size);
		}

	}
	for (; tuple < MAX_RANGES_TUPLES; tuple++) {
		/* zero-fill remaining tuples to mark empty elements in array */
		sc->ranges[ecam][tuple].base = 0;
		sc->ranges[ecam][tuple].size = 0;
	}

	rv = 0;
out:
	free(ranges_buf, M_OFWPROP);
	return (rv);
}

static uint32_t
thunder_pcie_read_config(device_t dev, u_int bus, u_int slot,
    u_int func, u_int reg, int bytes)
{
	uint64_t offset;
	uint32_t data;
	struct thunder_pcie_softc *sc;

	if (bus > 255 || slot > 31 || func > 7 || reg > 4095)
		return (~0U);

	sc = device_get_softc(dev);
	offset = PCIE_ADDR_OFFSET(bus, slot, func, reg);

	switch (bytes) {
	case 1:
		data = bus_space_read_1(sc->bst, sc->bsh, offset);
		break;
	case 2:
		data = le16toh(bus_space_read_2(sc->bst, sc->bsh, offset));
		break;
	case 4:
		data = le32toh(bus_space_read_4(sc->bst, sc->bsh, offset));
		break;
	default:
		return (~0U);
	}

	return (data);
}

static void
thunder_pcie_write_config(device_t dev, u_int bus, u_int slot,
    u_int func, u_int reg, uint32_t val, int bytes)
{
	uint64_t offset;
	struct thunder_pcie_softc *sc;

	if (bus > 255 || slot > 31 || func > 7 || reg > 4095)
		return;

	sc = device_get_softc(dev);
	offset = PCIE_ADDR_OFFSET(bus, slot, func, reg);

	switch (bytes) {
	case 1:
		bus_space_write_1(sc->bst, sc->bsh, offset, val);
		break;
	case 2:
		bus_space_write_2(sc->bst, sc->bsh, offset, htole16(val));
		break;
	case 4:
		bus_space_write_4(sc->bst, sc->bsh, offset, htole32(val));
		break;
	default:
		return;
	}

}

static int
thunder_pcie_maxslots(device_t dev)
{

	return 31; /* max slots per bus acc. to standard */
}

static int
thunder_pcie_read_ivar(device_t dev, device_t child, int index,
    uintptr_t *result)
{
	int unit_to_bus[15] = {
		0,1,2,3,4,0,1,2,3,0,1,0,1,2,3
	}; /* WA for now, assumes static topology */
	int domain;

	if (index == PCIB_IVAR_BUS) {
		if (device_get_unit(dev) > 14 || device_get_unit(dev) < 0)
			return (ENOENT);
		*result = unit_to_bus[device_get_unit(dev)];
		return (0);
	}
	if (index == PCIB_IVAR_DOMAIN) {
		if (thunder_pcie_get_ecam_domain(dev, &domain))
			return (ENOENT);
		*result = domain;
		return (0);
	}

	device_printf(dev, "ERROR: Unknown index.\n");
	return (ENOENT);
}

static int
thunder_pcie_write_ivar(device_t dev, device_t child, int index,
    uintptr_t value)
{

	return (ENOENT);
}

static int
thunder_pcie_release_resource(device_t dev, device_t child, int type, int rid,
    struct resource *res)
{

	if (type != SYS_RES_MEMORY)
		return (BUS_RELEASE_RESOURCE(device_get_parent(dev), child,
		    type, rid, res));

	return (rman_release_resource(res));
}

static struct resource *
thunder_pcie_alloc_resource(device_t dev, device_t child, int type, int *rid,
    u_long start, u_long end, u_long count, u_int flags)
{
	struct thunder_pcie_softc *sc = device_get_softc(dev);
	struct rman *rm = NULL;
	struct resource *res;
	int domain;

	switch (type) {
	case SYS_RES_IOPORT:
		goto fail;
		break;
	case SYS_RES_MEMORY:
		rm = &sc->mem_rman;
		break;
	default:
		return (BUS_ALLOC_RESOURCE(device_get_parent(dev), dev,
		    type, rid, start, end, count, flags));
	};

	if ((start == 0UL) && (end == ~0UL)) {
		if (thunder_pcie_get_ecam_domain(dev, &domain))
		       goto fail;
		start = sc->ranges[domain][1].base;
		count = sc->ranges[domain][1].size;
		end = start + count - 1;
	}

	if (bootverbose) {
		device_printf(dev,
		    "rman_reserve_resource: start=%#lx, end=%#lx, count=%#lx\n",
		    start, end, count);
	}

	res = rman_reserve_resource(rm, start, end, count, flags, child);
	if (res == NULL)
		goto fail;

	rman_set_rid(res, *rid);
	rman_set_bustag(res, fdtbus_bs_tag);
	rman_set_bushandle(res, start);

	if (flags & RF_ACTIVE)
		if (bus_activate_resource(child, type, *rid, res)) {
			rman_release_resource(res);
			goto fail;
		}

	return (res);

fail:
	if (bootverbose) {
		device_printf(dev, "%s FAIL: type=%d, rid=%d, "
		    "start=%016lx, end=%016lx, count=%016lx, flags=%x\n",
		    __func__, type, *rid, start, end, count, flags);
	}

	return (NULL);
}

static int
thunder_pcie_get_ecam_domain(device_t dev, int *domain)
{
	u_long start;

	start = bus_get_resource_start(dev, SYS_RES_MEMORY, 0);

	if (start == THUNDER_ECAM0_CFG_BASE)
		*domain = 0;
	else if (start == THUNDER_ECAM1_CFG_BASE)
		*domain = 1;
	else if (start == THUNDER_ECAM2_CFG_BASE)
		*domain = 2;
	else if (start == THUNDER_ECAM3_CFG_BASE)
		*domain = 3;
	else {
		device_printf(dev,
		    "error: incorrect resource address=%#lx.\n", start);
		return (ENXIO);
	}
	return (0);
}

static int
thunder_pcie_map_msi(device_t pcib, device_t child, int irq,
    uint64_t *addr, uint32_t *data)
{
	int error;

	error = 0;
	error = arm_map_msix(child, irq, addr, data);
	return (error);
}

static int
thunder_pcie_alloc_msix(device_t pcib, device_t child, int *irq)
{
	int error;

	error = 0;
	error = arm_alloc_msix(child, irq);
	return (error);
}

static int
thunder_pcie_release_msix(device_t pcib, device_t child, int irq)
{
	int error;

	error = 0;
	error = arm_release_msix(child, irq);
	return (error);
}

static device_method_t thunder_pcie_methods[] = {
	DEVMETHOD(device_probe,			thunder_pcie_probe),
	DEVMETHOD(device_attach,		thunder_pcie_attach),
	DEVMETHOD(pcib_maxslots,		thunder_pcie_maxslots),
	DEVMETHOD(pcib_read_config,		thunder_pcie_read_config),
	DEVMETHOD(pcib_write_config,		thunder_pcie_write_config),
	DEVMETHOD(bus_read_ivar,		thunder_pcie_read_ivar),
	DEVMETHOD(bus_write_ivar,		thunder_pcie_write_ivar),
	DEVMETHOD(bus_alloc_resource,		thunder_pcie_alloc_resource),
	DEVMETHOD(bus_release_resource,		thunder_pcie_release_resource),
	DEVMETHOD(bus_activate_resource,	bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	bus_generic_deactivate_resource),
	DEVMETHOD(bus_setup_intr,		bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,		bus_generic_teardown_intr),
	DEVMETHOD(pcib_map_msi,			thunder_pcie_map_msi),
	DEVMETHOD(pcib_alloc_msix,		thunder_pcie_alloc_msix),
	DEVMETHOD(pcib_release_msix,		thunder_pcie_release_msix),

	DEVMETHOD_END
};

static driver_t thunder_pcie_driver = {
	"pcib",
	thunder_pcie_methods,
	sizeof(struct thunder_pcie_softc),
};

static devclass_t thunder_pcie_devclass;

DRIVER_MODULE(pcib, simplebus, thunder_pcie_driver,
thunder_pcie_devclass, 0, 0);
DRIVER_MODULE(pcib, ofwbus, thunder_pcie_driver,
thunder_pcie_devclass, 0, 0);
