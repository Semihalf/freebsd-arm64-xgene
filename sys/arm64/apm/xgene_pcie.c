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

/*
 * PCIe root complex driver for APM X-Gene SoC
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

#define	XGENE_PCIE_DEBUG
#undef XGENE_PCIE_DEBUG

#define	XGENE_PCIE_RESOURCE_TYPE	0x8
#define	XGENE_PCIE_ROOT_CMPLX_CLASS	0x060400
#define	XGENE_PCIE_ENDPOINT_CLASS	0x118000

#define	XGENE_PCIECORE_CTLANDSTATUS	0x50
#define	XGENE_PIM1_1L			0x80
#define	XGENE_IBAR2			0x98
#define	XGENE_IR2MSK			0x9c
#define	XGENE_PIM2_1L			0xa0
#define	XGENE_IBAR3L			0xb4
#define	XGENE_IR3MSKL			0xbc
#define	XGENE_PIM3_1L			0xc4
#define	XGENE_OMR1BARL			0x100
#define	XGENE_OMR2BARL			0x118
#define	XGENE_OMR3BARL			0x130
#define	XGENE_CFGBARL			0x154
#define	XGENE_CFGBARH			0x158
#define	XGENE_CFGCTL			0x15C
#define	XGENE_RTDID			0x160
#define	XGENE_BRIDGE_CFG_0		0x2000
#define	XGENE_BRIDGE_CFG_4		0x2010
#define	XGENE_SRST			0xc000
#define	XGENE_CLKEN			0xc008
#define	XGENE_AXI_EP_CFG_ACCESS		0x10000
#define	XGENE_EN_COHERENCY		0xF0000000

#define	XGENE_EN_REG			0x00000001
#define	XGENE_OB_LO_IO			0x00000002

#define	XGENE_LINKUP_MASK		0x00000100

#define	XGENE_PCIE_DEVICEID		0xE004
#define	XGENE_PCIE_VENDORID		0x10E8

#define	SIZE_1K				0x00000400
#define	SIZE_1M				0x00100000
#define	SIZE_16M			0x01000000
#define	SIZE_128M			0x08000000
#define	SIZE_1G				0x40000000
#define	SIZE_1T				(SIZE_1G * 1024ULL)

#define	XGENE_DMA_RANGES_COUNT		2

/* Helpers for CSR space R/W access */
#define xgene_pcie_csr_read(reg)					\
	le32toh(bus_read_4(sc->res[XGENE_PCIE_CSR], (reg)))
#define xgene_pcie_csr_write(reg, val)					\
		bus_write_4(sc->res[XGENE_PCIE_CSR], (reg), htole32((val)))

/* Helpers for CFG space R/W access */
#define	xgene_pcie_cfg_read(reg)					\
	le32toh(bus_read_4(sc->res[XGENE_PCIE_CFG], sc->offset + (reg)))
#define	xgene_pcie_cfg_write(reg, val)					\
	bus_write_4(sc->res[XGENE_PCIE_CFG], sc->offset + (reg), htole32((val)))

#define XGENE_PCIE_DEBUG
#ifdef XGENE_PCIE_DEBUG
#define printdbg(fmt, args...)		\
do {					\
	printf("%s(): ", __func__);	\
	printf(fmt, ##args);		\
} while (0)
#define	bootverbose 1
#else
#define	printdbg(fmt, args...)
#endif

typedef enum {
	ROOT_CMPLX,
	ENDPOINT,
} xgene_pcie_mode;

enum xgene_memory_regions {
	XGENE_PCIE_CSR = 0,
	XGENE_PCIE_CFG,
	XGENE_PCIE_MEM_REGIONS, /* total number of memory regions */
};

struct range {
	uint32_t	flags;
	bus_addr_t	pci_addr;
	bus_addr_t	cpu_addr;
	uint64_t	size;

};

struct dma_ranges {
	struct range dma[XGENE_DMA_RANGES_COUNT];
};

struct pci_resources {
	struct range		io;
	struct range		mem;
	struct dma_ranges	dma_ranges;

	struct rman		io_rman;
	struct rman		mem_rman;

	bus_addr_t		io_alloc;
	bus_addr_t		mem_alloc;
};

struct xgene_pcie_softc {
	device_t		dev;

	struct resource *	res[XGENE_PCIE_MEM_REGIONS];
	struct pci_resources	pci_res;

	bus_addr_t		offset;
	int			busnr;
	xgene_pcie_mode		mode;

	struct mtx		rw_mtx;
	boolean_t		mtx_init;

	int irq_min, irq_max, irq_alloc;
};

static struct resource_spec xgene_pcie_mem_spec[] = {
	{ SYS_RES_MEMORY, XGENE_PCIE_CSR, RF_ACTIVE },
	{ SYS_RES_MEMORY, XGENE_PCIE_CFG, RF_ACTIVE },
	{ -1, 0, 0 }
};

/*
 * Forward prototypes
 */
static int xgene_pcie_probe(device_t);
static int xgene_pcie_attach(device_t);

static uint32_t xgene_pcie_read_config(device_t, u_int, u_int, u_int, u_int,
    int);
static void xgene_pcie_write_config(device_t, u_int, u_int, u_int, u_int,
    uint32_t, int);
static int xgene_pcie_maxslots(device_t);
static int xgene_pcie_read_ivar(device_t, device_t, int, uintptr_t *);
static int xgene_pcie_write_ivar(device_t, device_t, int, uintptr_t);
static struct resource *xgene_pcie_alloc_resource(device_t, device_t, int,
    int *, u_long, u_long, u_long, u_int);
static int xgene_pcie_release_resource(device_t, device_t, int, int,
    struct resource *);
static int xgene_pcie_map_msi(device_t, device_t, int, uint64_t *,
    uint32_t *);
static int xgene_pcie_alloc_msix(device_t, device_t, int *);
static int xgene_pcie_release_msix(device_t, device_t, int);
static void xgene_pcie_init_irqs(struct xgene_pcie_softc*);
static int xgene_pcie_setup(struct xgene_pcie_softc *);
static int xgene_pcie_map_range(struct xgene_pcie_softc *, struct range *, int);
static void xgene_pcie_setup_ib_reg(struct xgene_pcie_softc *, uint64_t,
    uint64_t, uint64_t, boolean_t, uint8_t *);
static uint64_t xgene_pcie_xlate_addr_pci_to_cpu(struct xgene_pcie_softc*,
		bus_addr_t, int);
static int xgene_pcib_init(struct xgene_pcie_softc *, int, int);
static int xgene_pcib_init_bar(struct xgene_pcie_softc *, int, int, int, int);
static void xgene_pcie_clear_firmware_config(struct xgene_pcie_softc *);
static int xgene_pcie_parse_fdt_ranges(struct xgene_pcie_softc *);
static int xgene_pcie_select_ib_region(uint8_t *, uint64_t);
static void xgene_pcie_set_cfg_offset(struct xgene_pcie_softc *, u_int);
static uint64_t xgene_pcie_set_ib_mask(struct xgene_pcie_softc *,
    uint32_t, uint32_t, uint64_t);
static void xgene_pcie_set_rtdid_reg(struct xgene_pcie_softc *,
    u_int, u_int, u_int);
static void xgene_pcie_setup_cfg_reg(struct xgene_pcie_softc *);
static void xgene_pcie_setup_outbound_reg(struct xgene_pcie_softc *, uint32_t,
    uint64_t, uint64_t, uint64_t, int);
static void xgene_pcie_setup_pims(struct xgene_pcie_softc *, uint32_t,
    uint64_t, uint64_t);
static void xgene_pcie_write_vendor_device_ids(struct xgene_pcie_softc *);
static void xgene_pcie_linkup_status(struct xgene_pcie_softc *);
static boolean_t xgene_pcie_hide_root_cmplx_bars(struct xgene_pcie_softc *,
    u_int, u_int);
static int xgene_pcie_setup_interrupt(device_t, device_t, struct resource *,
    int, driver_filter_t *, driver_intr_t *, void *, void **);
static int xgene_pcie_alloc_msi(device_t, device_t, int, int, int *);
int xgene_msix_alloc(int, int *);
int xgene_msix_map_msi(int, bus_addr_t *, uint32_t *);
int xgene_msix_release(int, int *);
int xgene_msix_setup_irq(device_t, device_t, struct resource *, int,
    driver_filter_t *, driver_intr_t *, void *, void **);
/*
 * Newbus interface declarations
 */
static device_method_t xgene_pcie_methods[] = {
	DEVMETHOD(device_probe,			xgene_pcie_probe),
	DEVMETHOD(device_attach,		xgene_pcie_attach),

	DEVMETHOD(pcib_maxslots,		xgene_pcie_maxslots),
	DEVMETHOD(pcib_read_config,		xgene_pcie_read_config),
	DEVMETHOD(pcib_write_config,		xgene_pcie_write_config),
	DEVMETHOD(pcib_map_msi,			xgene_pcie_map_msi),
	DEVMETHOD(pcib_alloc_msix,		xgene_pcie_alloc_msix),
	DEVMETHOD(pcib_release_msix,		xgene_pcie_release_msix),
	DEVMETHOD(pcib_alloc_msi,		xgene_pcie_alloc_msi),

	DEVMETHOD(bus_read_ivar,		xgene_pcie_read_ivar),
	DEVMETHOD(bus_write_ivar,		xgene_pcie_write_ivar),
	DEVMETHOD(bus_alloc_resource,		xgene_pcie_alloc_resource),
	DEVMETHOD(bus_release_resource,		xgene_pcie_release_resource),

	DEVMETHOD(bus_activate_resource,	bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	bus_generic_deactivate_resource),
	DEVMETHOD(bus_setup_intr,		xgene_pcie_setup_interrupt),
	DEVMETHOD(bus_teardown_intr,		bus_generic_teardown_intr),

	DEVMETHOD_END
};

static driver_t xgene_pcie_driver = {
	"pcib",
	xgene_pcie_methods,
	sizeof(struct xgene_pcie_softc),
};

static devclass_t xgene_pcie_devclass;

DRIVER_MODULE(pcib, ofwbus, xgene_pcie_driver, xgene_pcie_devclass, 0, 0);
DRIVER_MODULE(pcib, simplebus, xgene_pcie_driver, xgene_pcie_devclass, 0, 0);

static __inline uint32_t
lower_32_bits(uint64_t val)
{

	return ((uint32_t)(val & 0xFFFFFFFF));
}

static __inline uint32_t
higher_32_bits(uint64_t val)
{

	return ((uint32_t)(val >> 32));
}
/*
 * PCIe write/read configuration space
 */
static __inline void
xgene_pcie_cfg_w32(struct xgene_pcie_softc *sc, u_int reg, uint32_t val)
{

	xgene_pcie_cfg_write(reg, val);
}

static __inline void
xgene_pcie_cfg_w16(struct xgene_pcie_softc *sc, u_int reg, uint16_t val)
{
	uint32_t val32 = xgene_pcie_cfg_read(reg & ~0x3);

	switch (reg & 0x3)
	{
	case 2:
		val32 &= ~0xFFFF0000;
		val32 |= (uint32_t)val << 16;
		break;
	case 0:
	default:
		val32 &= ~0xFFFF;
		val32 |= val;
		break;
	}
	xgene_pcie_cfg_write(reg & ~0x3, val32);
}

static __inline void
xgene_pcie_cfg_w8(struct xgene_pcie_softc *sc, u_int reg, uint8_t val)
{
	uint32_t val32 = xgene_pcie_cfg_read(reg & ~0x3);

	switch (reg & 0x3)
	{
	case 0:
		val32 &= ~0xFF;
		val32 |= val;
		break;
	case 1:
		val32 &= ~0xFF00;
		val32 |= (uint32_t)val << 8;
		break;
	case 2:
		val32 &= ~0xFF0000;
		val32 |= (uint32_t)val << 16;
		break;
	case 3:
	default:
		val32 &= ~0xFF000000;
		val32 |= (uint32_t)val << 24;
		break;
	}
	xgene_pcie_cfg_write(reg & ~0x3, val32);
}

static __inline uint32_t
xgene_pcie_cfg_r32(struct xgene_pcie_softc *sc, u_int reg)
{
	uint32_t retval = xgene_pcie_cfg_read(reg);

	return (retval);
}

static __inline uint16_t
xgene_pcie_cfg_r16(struct xgene_pcie_softc *sc, u_int reg)
{
	uint32_t val = xgene_pcie_cfg_read(reg & ~0x3);

	switch (reg & 0x3) {
	case 2:
		val >>= 16;
		break;
	}

	return((uint16_t)(val &= 0xFFFF));
}

static __inline uint8_t
xgene_pcie_cfg_r8(struct xgene_pcie_softc *sc, u_int reg)
{
	uint32_t val = xgene_pcie_cfg_read(reg & ~0x3);

	switch (reg & 0x3) {
	case 3:
		val >>= 24;
		break;
	case 2:
		val >>= 16;
		break;
	case 1:
		val >>= 8;
		break;
	}

	return((uint8_t)(val &= 0xFF));
}

/*
 * APM Specific funcs
 */

/*
 * For Configuration request, RTDID register is used as Bus Number,
 * Device Number and Function number of the header fields.
 */
static void
xgene_pcie_set_rtdid_reg(struct xgene_pcie_softc *sc, u_int bus, u_int slot,
    u_int func)
{
	uint32_t rtdid_val = 0, retval;

	if (sc->mode == ROOT_CMPLX)	{
		rtdid_val = (bus << 8) | (slot << 3) | func;
	}

	xgene_pcie_csr_write(XGENE_RTDID, rtdid_val);
	/* Read register to make sure flush was done */
	retval = xgene_pcie_csr_read(XGENE_RTDID);

	if ((retval & 0xFFFF) != rtdid_val)	{
		printdbg("RTDID value set failed!\n");
	}
}

/*
 * X-Gene PCIe port uses BAR0-BAR1 of RC's configuration space as
 * the translation from PCI bus to native BUS.  Entire DDR region
 * is mapped into PCIe space using these registers, so it can be
 * reached by DMA from EP devices.  The BAR0/1 of bridge should be
 * hidden during enumeration to avoid the sizing and resource allocation
 * by PCIe core.
 */
static boolean_t xgene_pcie_hide_root_cmplx_bars(struct xgene_pcie_softc *sc,
    u_int bus, u_int reg)
{
	if (bus == 0 && sc->mode == ROOT_CMPLX &&
			(reg == PCIR_BAR(0) || reg == PCIR_BAR(1)))
		return true;

	return false;
}

/*
 * When the address bit [17:16] is 2'b01, the Configuration access will be
 * treated as Type 1 and it will be forwarded to external PCIe device.
 */
static void
xgene_pcie_set_cfg_offset(struct xgene_pcie_softc *sc, u_int bus)
{

	if (bus >= 1)
		sc->offset = XGENE_AXI_EP_CFG_ACCESS;
	else
		sc->offset = 0;
}

/* clear BAR configuration which was done by firmware */
static void
xgene_pcie_clear_firmware_config(struct xgene_pcie_softc *sc)
{
	int i;

	for (i = XGENE_PIM1_1L; i <= XGENE_CFGCTL; i += 4)
		xgene_pcie_csr_write(i, 0x0);
}

/* parse data from Device Tree and set memory windows */
static int
xgene_pcie_parse_fdt_ranges(struct xgene_pcie_softc *sc)
{
	u_int retval = 0;
	uint8_t ib_reg_mask = 0;
	uint32_t flags;
	phandle_t node;
	pcell_t pci_addr_cells, parent_addr_cells, size_cells;
	pcell_t *ranges_buf, *cell_ptr;
	int cells_count, tuples_count, touple, i, type;
	struct range *range;

	node = ofw_bus_get_node(sc->dev);

	if (fdt_addrsize_cells(node, &pci_addr_cells, &size_cells))
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

		if (flags & OFW_PCI_PHYS_HI_SPACE_IO) {
			range = &sc->pci_res.io;
			type = SYS_RES_IOPORT;
		} else if (flags & OFW_PCI_PHYS_HI_SPACE_MEM32) {
			range = &sc->pci_res.mem;
			type = SYS_RES_MEMORY;
		} else {
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

		if (xgene_pcie_map_range(sc, range, type)) {
			retval = ERANGE;
			goto out;
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

	for (i = 0 ; i < XGENE_DMA_RANGES_COUNT; i++) {
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

		xgene_pcie_setup_ib_reg(sc, range->cpu_addr, range->pci_addr,
		    range->size ,range->flags & OFW_PCI_PHYS_HI_PREFETCHABLE,
		    &ib_reg_mask);
	}

out:
	free(ranges_buf, M_OFWPROP);
	return (retval);
}

/* write configuration for BAR */
static int
xgene_pcib_init_bar(struct xgene_pcie_softc *sc, int bus, int slot, int func,
    int barno)
{
	bus_addr_t *allocp;
	uint32_t addr_l, addr_h, mask, size;
	int reg, width;

	reg = PCIR_BAR(barno);

	xgene_pcie_write_config(sc->dev, bus, slot, func, reg, ~0U, 4);
	size = xgene_pcie_read_config(sc->dev, bus, slot, func, reg, 4);

	if (size == 0)
		return (1);

	width = ((size & 7) == 4) ? 2 : 1;

	if (size & 1) {			/* I/O port */
		allocp = &sc->pci_res.io_alloc;
		size &= ~3;
		if ((size & 0xffff0000) == 0)
			size |= 0xffff0000;
	} else {			/* memory */
		allocp = &sc->pci_res.mem_alloc;
		size &= ~15;
	}
	mask = ~size;
	size = mask + 1;
	/* Sanity check (must be a power of 2). */
	if (size & mask)
		return (width);

	addr_l = ((lower_32_bits(*allocp) + mask) & ~mask);
	addr_h = higher_32_bits(*allocp);
	*allocp = (bus_addr_t)addr_h << 32 | (addr_l + size);

	if (bootverbose) {
		printf("PCI %u:%u:%u:%u: BAR%d (reg %x): "
		    "size=%#x: addr_h=%#x addr_l=%#x\n",
		    device_get_unit(sc->dev), bus, slot, func, barno, reg,
		    size, addr_h, addr_l);
	}

	xgene_pcie_write_config(sc->dev, bus, slot, func, reg, addr_l, 4);
	if(width == 2) {
		xgene_pcie_write_config(sc->dev, bus, slot, func, reg + 4,
		    addr_h, 4);
	}

	return (width);
}

/* enumerate PCIe devices */
static int
xgene_pcib_init(struct xgene_pcie_softc *sc, int bus, int maxslot)
{
	int secbus;
	int old_pribus, old_secbus, old_subbus;
	int new_pribus, new_secbus, new_subbus;
	int slot, func, maxfunc;
	int bar, maxbar;
	uint8_t command, hdrtype, class, subclass;
	bus_addr_t iobase, iolimit, membase, memlimit;
	/*  uint8_t intline, intpin; */

	secbus = bus;
	for (slot = 0; slot <= maxslot; slot++) {
		maxfunc = 0;
		for (func = 0; func <= maxfunc; func++) {
			hdrtype = xgene_pcie_read_config(sc->dev, bus, slot,
			    func, PCIR_HDRTYPE, 1);

			if ((hdrtype & PCIM_HDRTYPE) > PCI_MAXHDRTYPE)
				continue;

			if (func == 0 && (hdrtype & PCIM_MFDEV))
				maxfunc = PCI_FUNCMAX;

			/*
			 * disable device mem/io function
			 * while programming init values
			 */
			command = xgene_pcie_read_config(sc->dev, bus, slot,
			    func, PCIR_COMMAND, 1);
			command &= ~(PCIM_CMD_MEMEN | PCIM_CMD_PORTEN);
			xgene_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_COMMAND, command, 1);

			/* Program the base address registers. */
			maxbar = (hdrtype & PCIM_HDRTYPE) ? 0 : 6;
			bar = 0;
			while (bar < maxbar) {
				bar += xgene_pcib_init_bar(sc, bus, slot, func,
				    bar);
			}
#if 0
			/* Perform interrupt routing. */
			intpin = xgene_pcib_read_config(sc->dev, bus, slot,
			    func, PCIR_INTPIN, 1);
			intline = xgene_pcib_route_int(sc, bus, slot, func,
			    intpin);
			xgene_pcib_write_config(sc->sc_dev, bus, slot, func,
			    PCIR_INTLINE, intline, 1);
#endif

			/* enable device mem/io function */
			command |= PCIM_CMD_MEMEN | PCIM_CMD_PORTEN |
			    PCIM_CMD_BUSMASTEREN;
			xgene_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_COMMAND, command, 1);

			/*
			 * Handle PCI-PCI bridges
			 */
			class = xgene_pcie_read_config(sc->dev, bus, slot,
			    func, PCIR_CLASS, 1);
			subclass = xgene_pcie_read_config(sc->dev, bus, slot,
			    func, PCIR_SUBCLASS, 1);

			/* Allow only proper PCI-PCI briges */
			if (class != PCIC_BRIDGE || subclass != PCIS_BRIDGE_PCI)
				continue;

			secbus++;

			/* Program I/O decoder. */
			iobase = sc->pci_res.io.pci_addr;
			xgene_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_IOBASEL_1, iobase >> 8, 1);
			xgene_pcie_write_config(sc->dev, bus, slot, func,
				PCIR_IOBASEH_1, iobase >> 16, 2);

			iolimit = iobase + sc->pci_res.io.size - 1;
			xgene_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_IOLIMITL_1, iolimit >> 8, 1);
			xgene_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_IOLIMITH_1, iolimit >> 16, 2);

			/* Program (non-prefetchable) memory decoder. */
			xgene_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_MEMBASE_1, 0x0010, 2);
			xgene_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_MEMLIMIT_1, 0x000f, 2);

			/* Program prefetchable memory decoder. */
			membase = sc->pci_res.mem.pci_addr;
			xgene_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_PMBASEL_1, lower_32_bits(membase) >> 16, 2);
			xgene_pcie_write_config(sc->dev, bus, slot, func,
				PCIR_PMBASEH_1, higher_32_bits(membase), 4);

			memlimit = membase + sc->pci_res.mem.size - 1;
			xgene_pcie_write_config(sc->dev, bus, slot, func,
				PCIR_PMLIMITL_1, lower_32_bits(memlimit) >> 16, 2);
			xgene_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_PMLIMITH_1, higher_32_bits(memlimit), 4);

			/* Read currect bus register configuration */
			old_pribus = xgene_pcie_read_config(sc->dev, bus,
			    slot, func, PCIR_PRIBUS_1, 1);
			old_secbus = xgene_pcie_read_config(sc->dev, bus,
			    slot, func, PCIR_SECBUS_1, 1);
			old_subbus = xgene_pcie_read_config(sc->dev, bus,
			    slot, func, PCIR_SUBBUS_1, 1);

			if (bootverbose ) {
				printf("PCI: reading firmware bus numbers for "
				    "secbus = %d (bus/sec/sub) = (%d/%d/%d)\n",
				    secbus, old_pribus, old_secbus, old_subbus);
			}

			new_pribus = bus;
			new_secbus = secbus;

			secbus = xgene_pcib_init(sc, secbus,
			    (subclass == PCIS_BRIDGE_PCI) ? PCI_SLOTMAX : 0);

			new_subbus = secbus;

			if (bootverbose) {
				printf("PCI: translate firmware bus numbers "
				    "for secbus %d (%d/%d/%d) -> (%d/%d/%d)\n",
				    secbus, old_pribus, old_secbus, old_subbus,
				    new_pribus, new_secbus, new_subbus);
			}

			xgene_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_PRIBUS_1, new_pribus, 1);
			xgene_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_SECBUS_1, new_secbus, 1);
			xgene_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_SUBBUS_1, new_subbus, 1);
		}
	}

	return (secbus);
}

static void
xgene_pcie_write_vendor_device_ids(struct xgene_pcie_softc *sc)
{
	uint32_t val;

	/* setup the vendor and device IDs correctly */
	val = (XGENE_PCIE_DEVICEID << 16) | XGENE_PCIE_VENDORID;
	xgene_pcie_csr_write(XGENE_BRIDGE_CFG_0, val);
}

/*
 * APM PCIe controller has 3 in-bound memory regions that
 * can be used depending on requested size
 */
static int
xgene_pcie_select_ib_region(uint8_t* ib_reg_mask, uint64_t size)
{
	int retval = -1;

	if ((size > 4) && (size < SIZE_16M) && !(*ib_reg_mask & (1 << 1))) {
		*ib_reg_mask |= (1 << 1);
		retval = 1;
	} else if ((size > SIZE_1K) &&
	    (size < SIZE_1T) && !(*ib_reg_mask & (1 << 0))) {
		*ib_reg_mask |= (1 << 0);
		retval = 0;
	} else if ((size > SIZE_1M) &&
	    (size < SIZE_1T) && !(*ib_reg_mask & (1 << 2))) {
		*ib_reg_mask |= (1 << 2);
		retval = 2;
	}

	return (retval);
}

static uint64_t
xgene_pcie_set_ib_mask(struct xgene_pcie_softc *sc, uint32_t addr,
    uint32_t flags, uint64_t size)
{
	uint64_t mask = (~(size - 1) & PCIM_BAR_MEM_BASE) | flags;
	uint32_t val32 = 0;
	uint32_t val;

	val32 = xgene_pcie_csr_read(addr);
	val = (val32 & 0x0000ffff) | (lower_32_bits(mask) << 16);
	xgene_pcie_csr_write(addr,val);

	val32 = xgene_pcie_csr_read(addr + 0x04);
	val = (val32 & 0xffff0000) | (lower_32_bits(mask) >> 16);
	xgene_pcie_csr_write(addr + 0x04, val);

	val32 = xgene_pcie_csr_read(addr + 0x04);
	val = (val32 & 0x0000ffff) | (higher_32_bits(mask) << 16);
	xgene_pcie_csr_write(addr + 0x04, val);

	val32 = xgene_pcie_csr_read(addr + 0x08);
	val = (val32 & 0xffff0000) | (higher_32_bits(mask) >> 16);
	xgene_pcie_csr_write(addr + 0x08, val);

	return mask;
}

static void
xgene_pcie_setup_pims(struct xgene_pcie_softc *sc, uint32_t pim_addr,
    uint64_t pci_addr, uint64_t size)
{
	xgene_pcie_csr_write(pim_addr, lower_32_bits(pci_addr));
	xgene_pcie_csr_write(pim_addr + 0x04,
	    higher_32_bits(pci_addr) | XGENE_EN_COHERENCY);
	xgene_pcie_csr_write(pim_addr + 0x10, lower_32_bits(size));
	xgene_pcie_csr_write(pim_addr + 0x14, higher_32_bits(size));
}

static void
xgene_pcie_setup_ib_reg(struct xgene_pcie_softc *sc, uint64_t cpu_addr,
    uint64_t pci_addr, uint64_t size, boolean_t prefetch_flag,
    uint8_t *ib_reg_mask)
{
	int region;
	uint32_t addr, pim_addr;
	uint64_t mask = ~(size - 1) | XGENE_EN_REG;
	uint32_t flags = PCIM_BAR_MEM_64;

	mask = ~(size - 1) | XGENE_EN_REG;
	region = xgene_pcie_select_ib_region(ib_reg_mask, size);
	if (region < 0) {
		device_printf(sc->dev,"Invalid PCIe DMA-range configuration\n");
		return;
	}

	if (prefetch_flag)
		flags |= PCIM_BAR_MEM_PREFETCH;

	addr = (lower_32_bits(cpu_addr) & PCIM_BAR_MEM_BASE) | flags;

	switch (region) {
	case 0:
		xgene_pcie_set_ib_mask(sc, XGENE_BRIDGE_CFG_4, flags, size);
		xgene_pcie_cfg_write(PCIR_BAR(0), addr);
		xgene_pcie_cfg_write(PCIR_BAR(1), higher_32_bits(cpu_addr));
		pim_addr = XGENE_PIM1_1L;
		break;
	case 1:
		xgene_pcie_csr_write(XGENE_IBAR2, addr);
		xgene_pcie_csr_write(XGENE_IR2MSK, lower_32_bits(mask));
		pim_addr = XGENE_PIM2_1L;
		break;
	case 2:
		xgene_pcie_csr_write(XGENE_IBAR3L, addr);
		xgene_pcie_csr_write(XGENE_IBAR3L + 0x04, higher_32_bits(cpu_addr));
		xgene_pcie_csr_write(XGENE_IR3MSKL, lower_32_bits(mask));
		xgene_pcie_csr_write(XGENE_IR3MSKL + 0x04, higher_32_bits(mask));
		pim_addr = XGENE_PIM3_1L;
		break;
	}

	xgene_pcie_setup_pims(sc, pim_addr, pci_addr, ~(size - 1));
}

static void
xgene_pcie_setup_outbound_reg(struct xgene_pcie_softc *sc, uint32_t reg,
    uint64_t cpu_addr, uint64_t pci_addr, uint64_t size, int type)
{
	uint64_t mask = 0;
	uint32_t min_size;
	uint32_t flag = XGENE_EN_REG;

	switch (type) {
	case SYS_RES_MEMORY:
		min_size = SIZE_128M;
		break;

	case SYS_RES_IOPORT:
		min_size = 128;
		flag |= XGENE_OB_LO_IO;
		break;
	}

	if (size >= min_size)
		mask = ~(size - 1) | flag;
	else {
		device_printf(sc->dev, "res size %#lx less than minimum %#x\n",
		    (uint64_t)size, min_size);
	}

	/* Write addresses to CSR registers */
	xgene_pcie_csr_write(reg, lower_32_bits(cpu_addr));
	xgene_pcie_csr_write(reg + 0x04, higher_32_bits(cpu_addr));

	xgene_pcie_csr_write(reg + 0x08, lower_32_bits(mask));
	xgene_pcie_csr_write(reg + 0x0c, higher_32_bits(mask));

	xgene_pcie_csr_write(reg + 0x10, lower_32_bits(pci_addr));
	xgene_pcie_csr_write(reg + 0x14, higher_32_bits(pci_addr));
}

static void
xgene_pcie_setup_cfg_reg(struct xgene_pcie_softc* sc)
{
	vm_paddr_t addr;

	/* get memory space physical address */
	addr = pmap_kextract(rman_get_bushandle(sc->res[XGENE_PCIE_CFG]));
	printdbg("Bus handle = %#lx\n", addr);

	xgene_pcie_csr_write(XGENE_CFGBARL, lower_32_bits(addr));
	xgene_pcie_csr_write(XGENE_CFGBARH, higher_32_bits(addr));

	xgene_pcie_csr_write(XGENE_CFGCTL, XGENE_EN_REG);
}

static int
xgene_pcie_map_range(struct xgene_pcie_softc *sc, struct range *range, int type)
{
	device_t self = sc->dev;
	int err;
	struct rman *rm;
	char *descr;
	uint32_t ob_reg;
	bus_addr_t start, end;
	bus_addr_t *alloc;

	switch (type) {
	case SYS_RES_MEMORY:
		descr = "PCIe Memory resource";
		rm = &sc->pci_res.mem_rman;
		alloc = &sc->pci_res.mem_alloc;
		ob_reg = XGENE_OMR1BARL;
		break;
	case SYS_RES_IOPORT:
		descr = "PCIe I/O resource";
		rm = &sc->pci_res.io_rman;
		alloc = &sc->pci_res.io_alloc;
		ob_reg = XGENE_OMR3BARL;
		break;
	default:
		return (ERANGE);
	}

	/* save bus address for BAR allocation algorithm */
	*alloc = range->pci_addr;

	/* init resource manager for cpu memory window */
	rm->rm_type = RMAN_ARRAY;
	rm->rm_descr = descr;
	err = rman_init(rm);
	if (err) {
		device_printf(self,
		    "ERROR: rman_init() for %s failed (err: %d)\n", descr, err);
		return (err);
	}

	start = range->cpu_addr;
	end = range->cpu_addr + range->size -1;
	err = rman_manage_region(rm, start, end);
	if (err) {
		device_printf(self,
		    "ERROR: rman_manage_region() for %s failed (err: %d)\n",
		    descr, err);
		rman_fini(rm);
		return (err);
	}

	/* Setup outbound regions in controller's register */
	xgene_pcie_setup_outbound_reg(sc, ob_reg, range->cpu_addr,
	    range->pci_addr, range->size, type);

	return (0);
}

/* XXX: ARM64TODO: This function is generally a workaround
 * until there will be OFW functionality to parse interrupt-map
 * for obtaining IRQ numbers and calculating its offset */
static void
xgene_pcie_init_irqs(struct xgene_pcie_softc *sc)
{
	struct ofw_bus_iinfo pci_iinfo;
	uint8_t *ptr8;
	pcell_t *ptr32, size, imin[2], imax[2];
	int i;
	u_int min = UINT_MAX, max = 0, irq;

	const int column_num_in_fdt = 8;

	ofw_bus_setup_iinfo(ofw_bus_get_node(sc->dev),
			&pci_iinfo, sizeof(pcell_t));

	ptr8 = pci_iinfo.opi_imap;
	size = pci_iinfo.opi_imapsz;

	/* XXX: ofw_bus_setup_iinfo returns 1byte pointers
	 * need to translate this to word pointers */
	size = size / (column_num_in_fdt * 4); /* size in words */
	ptr8 -= 3;
	ptr32 = (pcell_t*)ptr8;

	ptr32 += 6; /* IRQ numbers are in 6th column */
	for (i = 0; i < size; i++) {
		irq = fdt_data_get((void *)ptr32, 1);

		if(irq < min)
			min = irq;
		if(irq > max)
			max = irq;
		ptr32 += column_num_in_fdt;
	}

	/* XXX: Want to call ofw_map_intr to add SPI offset for IRQ */
	imin[0] = 0;
	imax[0] = 0;
	imin[1] = min;
	imax[1] = max;

	sc->irq_min = ofw_bus_map_intr(sc->dev, 0, 3, imin);
	sc->irq_max = ofw_bus_map_intr(sc->dev, 0, 3, imax);

	sc->irq_alloc = sc->irq_min;
	printfdbg("IRQ: min = %d max = %d\n", sc->irq_min, sc->irq_max);

	return;
}

static void
xgene_pcie_linkup_status(struct xgene_pcie_softc *sc)
{
	uint32_t val;

	val = xgene_pcie_csr_read(XGENE_PCIECORE_CTLANDSTATUS);
	if (val & XGENE_LINKUP_MASK)
		device_printf(sc->dev,"(RC) link up\n");
	else
		device_printf(sc->dev,"(RC) link down\n");
}


static int
xgene_pcie_setup(struct xgene_pcie_softc *sc)
{
	int err;

	xgene_pcie_clear_firmware_config(sc);
	xgene_pcie_write_vendor_device_ids(sc);

	/* Parse FDT for memory windows */
	err = xgene_pcie_parse_fdt_ranges(sc);
	if (err) {
		device_printf(sc->dev,
		    "ERROR: parsing FDT ranges failed (err: %d)\n", err);
		return (ENXIO);
	}

	xgene_pcie_setup_cfg_reg(sc);
	xgene_pcie_linkup_status(sc);

	return (err);
}

/*
 * Generic device interfacee
 */

static int
xgene_pcie_probe(device_t self)
{

	if (!ofw_bus_status_okay(self))
		return (ENXIO);

	if (!ofw_bus_is_compatible(self, "apm,xgene-pcie"))
		return (ENXIO);

	device_set_desc(self, "APM X-Gene Integrated PCI-Express Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
xgene_pcie_attach(device_t self)
{
	struct xgene_pcie_softc *sc;
	uint32_t val;
	int err, maxslot = 0;

	sc = device_get_softc(self);
	sc->dev = self;

	sc->busnr = 0;

	err = bus_alloc_resources(self, xgene_pcie_mem_spec, sc->res);
	if (err) {
		device_printf(self,
		    "ERROR: cannot map resources memory (err: %d)\n", err);
		return (ENXIO);
	}

	/* Read class type of APM PCIe agent */
	val = xgene_pcie_cfg_read(XGENE_PCIE_RESOURCE_TYPE);
	switch (val >> 8) {
	case XGENE_PCIE_ROOT_CMPLX_CLASS:
		sc->mode = ROOT_CMPLX;
		xgene_pcie_init_irqs(sc);
		break;
	case XGENE_PCIE_ENDPOINT_CLASS:
		sc->mode = ENDPOINT;
		break;
	default:
		device_printf(self, "ERROR: "
		    "firmware set inappropriate device class in register\n");
		return (ENXIO);
	}

	/* Init R/W mutex */
	if (!sc->mtx_init) {
		mtx_init(&sc->rw_mtx, "pcicfg", NULL, MTX_SPIN);
		sc->mtx_init = true;
	}

	/* Set up X-Gene specific PCIe configuration */
	err = xgene_pcie_setup(sc);
	if (err)
		return (err);

	/* Enumerate PCIe devices */
	maxslot = 0;
	xgene_pcib_init(sc, sc->busnr, maxslot);

	device_add_child(self, "pci", -1);

	return (bus_generic_attach(self));
}

static uint32_t
xgene_pcie_read_config(device_t dev, u_int bus, u_int slot,
    u_int func, u_int reg, int bytes)
{
	struct xgene_pcie_softc* sc = device_get_softc(dev);
	uint32_t retval = ~0U;

	/* Root Complex can have only one function */
	if (sc->mode == ROOT_CMPLX && bus == 0 && (slot != 0 || func != 0))
		return (retval);

		mtx_lock_spin(&sc->rw_mtx);

	xgene_pcie_set_rtdid_reg(sc, bus, slot, func);
	xgene_pcie_set_cfg_offset(sc, bus);
	switch (bytes) {
	case 1:
		retval = xgene_pcie_cfg_r8(sc, reg);
		break;
	case 2:
		retval = xgene_pcie_cfg_r16(sc, reg);
		break;
	case 4:
		retval = xgene_pcie_cfg_r32(sc, reg);
		break;
	default:
		retval = ~0U;
	}

	mtx_unlock_spin(&sc->rw_mtx);

	return (retval);
}

static void
xgene_pcie_write_config(device_t dev, u_int bus, u_int slot,
    u_int func, u_int reg, uint32_t val, int bytes)
{
	struct xgene_pcie_softc* sc = device_get_softc(dev);

	if (bus > 255 || slot > 31 || func > 7 || reg > 4095)
			return;

	/* Only Root Complex should program RC's BAR0 and BAR1 */
	if(xgene_pcie_hide_root_cmplx_bars(sc, bus, reg))
		return;

	mtx_lock_spin(&sc->rw_mtx);

	xgene_pcie_set_rtdid_reg(sc, bus, slot, func);
	xgene_pcie_set_cfg_offset(sc, bus);

	switch (bytes) {
	case 1:
		xgene_pcie_cfg_w8(sc, reg, (uint8_t)val);
		break;
	case 2:
		xgene_pcie_cfg_w16(sc, reg, (uint16_t)val);
		break;
	case 4:
		xgene_pcie_cfg_w32(sc, reg, val);
		break;
	default:
		return;
	}

	mtx_unlock_spin(&sc->rw_mtx);
}

static int
xgene_pcie_maxslots(device_t dev)
{

	return (PCI_SLOTMAX);
}

static int
xgene_pcie_read_ivar(device_t dev, device_t child, int index,
    uintptr_t *result)
{
	struct xgene_pcie_softc *sc = device_get_softc(dev);

	switch (index) {
	case PCIB_IVAR_BUS:
		*result = sc->busnr;
		return (0);
	case PCIB_IVAR_DOMAIN:
		*result = device_get_unit(dev);
		return (0);
	}

	return (ENOENT);
}

static int
xgene_pcie_write_ivar(device_t dev, device_t child, int index,
    uintptr_t value)
{
	struct xgene_pcie_softc * sc = device_get_softc(dev);

	switch (index) {
	case PCIB_IVAR_BUS:
		sc->busnr = value;
		return (0);
	}

	return (ENOENT);
}

static uint64_t
xgene_pcie_xlate_addr_pci_to_cpu(struct xgene_pcie_softc *sc, bus_addr_t bus_addr, int type)
{
	struct range *r;
	uint64_t offset;

	switch (type) {
	case SYS_RES_IOPORT:
		r = &sc->pci_res.io;
		break;
	case SYS_RES_MEMORY:
		r = &sc->pci_res.mem;
		break;
	default:
		/* should never fall here */
		return (~0ul);
	}

	/* Find physical address corresponding to given bus address */
	if (bus_addr >= r->pci_addr &&
		bus_addr < r->pci_addr + r->size) {
		/* Given bus addr is in the range.
		 * Translate bus addr to cpu addr.
		 */
		offset = bus_addr - r->pci_addr;
		return (r->cpu_addr + offset);
	}
	return (~0ul);
}

static struct resource*
xgene_pcie_alloc_resource(device_t dev, device_t child, int type, int *rid,
    u_long start, u_long end, u_long count, u_int flags)
{
	struct xgene_pcie_softc *sc = device_get_softc(dev);
	struct rman *rm = NULL;
	struct resource *res;

	printdbg("request bus addr: %#lx..%#lx\n", start, end);

	switch (type) {
	case SYS_RES_IOPORT:
		rm = &sc->pci_res.io_rman;
		start = xgene_pcie_xlate_addr_pci_to_cpu(sc, start, type);
		end = start + count -1;
		break;
	case SYS_RES_MEMORY:
		rm = &sc->pci_res.mem_rman;

		if ((start == 0UL) && (end == ~0UL)) {
			start = sc->pci_res.mem.cpu_addr;
			end = sc->pci_res.mem.cpu_addr + sc->pci_res.mem.size - 1;
			count = sc->pci_res.mem.size;
		} else {
			start = xgene_pcie_xlate_addr_pci_to_cpu(sc, start, type);
			end = start + count -1;
		}
		break;
	case SYS_RES_IRQ:
		if (sc->irq_alloc + count <= sc->irq_max) {
			start = sc->irq_alloc;
			end = start + count -1;
			sc->irq_alloc += count;
		}
		/* no break on purpose */
	default:
		return (BUS_ALLOC_RESOURCE(device_get_parent(dev), dev,
			type, rid, start, end, count, flags));
	};

	printdbg("xlated to cpu addr: %#lx..%#lx\n", start, end);

	if (start == ~0ul)
		return (NULL);

	res = rman_reserve_resource(rm, start, end, count, flags, child);
	if (res == NULL)
		return (NULL);

	rman_set_rid(res, *rid);
	rman_set_bustag(res, fdtbus_bs_tag);
	rman_set_bushandle(res, start);

	if (flags & RF_ACTIVE)
		if (bus_activate_resource(child, type, *rid, res)) {
			rman_release_resource(res);
			return (NULL);
		}

	return (res);
}

static int
xgene_pcie_release_resource(device_t dev, device_t child,
    int type, int rid, struct resource *res)
{

	if (type != SYS_RES_IOPORT && type != SYS_RES_MEMORY) {
		return (BUS_RELEASE_RESOURCE(device_get_parent(dev), child,
			type, rid, res));
	}

	return (rman_release_resource(res));
}

static int
xgene_pcie_map_msi(device_t pcib, device_t child, int irq,
    uint64_t *addr, uint32_t *data)
{

	printdbg("enter with irq=%d\n", irq);
#if 0
	return (xgene_msix_map_msi(irq, addr, data));
#else
	return (108877);
#endif
}

static int
xgene_pcie_alloc_msi(device_t dev, device_t child, int count,
				   int maxcount __unused, int *irqs)
{
	printdbg("enter with cnt=%d\n", count);

	return (xgene_msix_alloc(count, irqs));
}

static int
xgene_pcie_alloc_msix(device_t pcib, device_t child, int *irq)
{

#if 0
	xgene_msix_alloc(1, irq));
#else
	return (109999);
#endif
}

static int
xgene_pcie_release_msix(device_t pcib, device_t child, int irq)
{

#if 0
	xgene_msix_release(1, &irq));
#else
	return (209999);
#endif
}

static int
xgene_pcie_setup_interrupt(device_t dev, device_t child, struct resource *irq,
    int flags, driver_filter_t *filter, driver_intr_t *intr, void *arg,
    void **cookiep)
{

	printdbg("enter\n");

	return (xgene_msix_setup_irq(dev, child, irq, flags, filter, intr, arg,
	    cookiep));
}
