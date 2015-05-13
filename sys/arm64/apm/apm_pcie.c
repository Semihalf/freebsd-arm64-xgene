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

#define APM_PCIE_RESOURCE_TYPE 0x8
#define APM_PCIE_ROOT_CMPLX_CLASS 0x060400
#define APM_PCIE_ENDPOINT_CLASS 0x118000

#define APM_PCIECORE_CTLANDSTATUS	0x50
#define APM_PIM1_1L		0x80
#define APM_IBAR2		0x98
#define APM_IR2MSK		0x9c
#define APM_PIM2_1L		0xa0
#define APM_IBAR3L		0xb4
#define APM_IR3MSKL		0xbc
#define APM_PIM3_1L		0xc4
#define APM_OMR1BARL	0x100
#define APM_OMR2BARL	0x118
#define APM_OMR3BARL	0x130
#define APM_CFGBARL		0x154
#define APM_CFGBARH		0x158
#define APM_CFGCTL		0x15C
#define APM_RTDID		0x160
#define APM_BRIDGE_CFG_0 0x2000
#define APM_BRIDGE_CFG_4 0x2010
#define APM_SRST		0xc000
#define APM_CLKEN		0xc008
#define APM_AXI_EP_CFG_ACCESS	0x10000
#define APM_EN_COHERENCY	0xF0000000

#define APM_EN_REG		0x00000001
#define APM_OB_LO_IO	0x00000002

#define APM_LINKUP_MASK			0x00000100

#define APM_PCIE_DEVICEID	0xE004
#define APM_PCIE_VENDORID	0x10E8

#define SIZE_1K	0x00000400
#define SIZE_1M	0x00100000
#define SIZE_16M	0x01000000
#define SIZE_128M	0x08000000
#define SIZE_1G	0x40000000
#define SIZE_1T	(SIZE_1G * 1024ULL)


typedef enum {
	ROOT_CMPLX,
	ENDPOINT,
} apm_pcie_mode;

#define APM_DMA_RANGES_COUNT 2
enum apm_memory_regions {
	APM_PCIE_CSR = 0,
	APM_PCIE_CFG,
	APM_PCIE_MEM_REGIONS, /* total number of memory regions */
};

/* Helpers for CSR space R/W access */
#define apm_pcie_csr_read(reg)	\
	le32toh(bus_read_4(sc->res[APM_PCIE_CSR], (reg)))
#define apm_pcie_csr_write(reg, val)	\
		bus_write_4(sc->res[APM_PCIE_CSR], (reg), htole32((val)))

/* Helpers for CFG space R/W access */
#define	apm_pcie_cfg_read(reg)	\
	le32toh(bus_read_4(sc->res[APM_PCIE_CFG], sc->offset + (reg)))
#define apm_pcie_cfg_write(reg, val)	\
	bus_write_4(sc->res[APM_PCIE_CFG], sc->offset + (reg), htole32((val)))

#if 1
#define printdbg(fmt,args...) do { printf("%s(): ", __func__); \
								   printf(fmt,##args); } while (0)
#define bootverbose 1
#else
#define printdbg(fmt,args...)
#endif

struct range {
	uint32_t flags;
	bus_addr_t pci_addr;
	bus_addr_t cpu_addr;
	uint64_t size;

};
struct dma_ranges {
	struct range dma[APM_DMA_RANGES_COUNT];
};

struct pci_resources {
	struct range io;
	struct range mem;
	struct dma_ranges dma_ranges;

	struct rman	io_rman;
	struct rman	mem_rman;

	bus_addr_t io_alloc;
	bus_addr_t mem_alloc;
};

struct apm_pcie_softc {
	device_t 		dev;

	struct resource	*res[APM_PCIE_MEM_REGIONS];
	struct pci_resources pci_res;

	bus_addr_t offset;
	int busnr;
	apm_pcie_mode	mode;

	struct mtx rw_mtx;
	bool mtx_init;
};

static struct resource_spec apm_pcie_mem_spec[] = {
	{ SYS_RES_MEMORY, APM_PCIE_CSR, RF_ACTIVE },
	{ SYS_RES_MEMORY, APM_PCIE_CFG, RF_ACTIVE },
	{ -1, 0, 0 }
};

/*
 * Forward prototypes
 */
static int apm_pcie_probe(device_t);
static int apm_pcie_attach(device_t);

static uint32_t apm_pcie_read_config(device_t,u_int,u_int,u_int,u_int,int);
static void apm_pcie_write_config(device_t,u_int,u_int,u_int,u_int,uint32_t,int);
static int apm_pcie_maxslots(device_t);
static int apm_pcie_read_ivar(device_t, device_t,int,uintptr_t*);
static int apm_pcie_write_ivar(device_t dev, device_t child, int index,
    uintptr_t value);
static struct resource *apm_pcie_alloc_resource(device_t dev,
    device_t child, int type, int *rid, u_long start, u_long end,
    u_long count, u_int flags);
static int apm_pcie_release_resource(device_t dev, device_t child,
    int type, int rid, struct resource *res);
static int apm_pcie_map_msi(device_t pcib, device_t child, int irq,
    uint64_t *addr, uint32_t *data);
static int apm_pcie_alloc_msix(device_t pcib, device_t child, int *irq);
static int apm_pcie_release_msix(device_t pcib, device_t child, int irq);

static void apm_pcie_setup(struct apm_pcie_softc *sc);
static int apm_pcie_map_range(struct apm_pcie_softc* sc, struct range *range, int type);
static void apm_pcie_setup_ib_reg(struct apm_pcie_softc* sc, uint64_t cpu_addr,
	uint64_t pci_addr, uint64_t size, bool prefetch_flag, uint8_t *ib_reg_mask);

static int apm_pcib_init(struct apm_pcie_softc *sc, int bus, int maxslot);
static int apm_pcib_init_bar(struct apm_pcie_softc *sc, int bus, int slot, int func,
    int barno);
static void apm_pcie_clear_firmware_config(struct apm_pcie_softc* sc);
static u_int apm_pcie_parse_fdt_ranges(struct apm_pcie_softc *sc);
static int apm_pcie_select_ib_region(uint8_t* ib_reg_mask, uint64_t size);
static void apm_pcie_set_cfg_offset(struct apm_pcie_softc* sc, u_int bus);
static uint64_t apm_pcie_set_ib_mask(struct apm_pcie_softc* sc,
	uint32_t addr, uint32_t flags, uint64_t size);
static void apm_pcie_set_rtdid_reg(struct apm_pcie_softc* sc,
	u_int bus, u_int slot, u_int func);
static void apm_pcie_setup_cfg_reg(struct apm_pcie_softc* sc);
static void apm_pcie_setup_outbound_reg(struct apm_pcie_softc* sc, uint32_t reg,
	uint64_t cpu_addr, uint64_t pci_addr, uint64_t size, int type);
static void apm_pcie_setup_pims(struct apm_pcie_softc *sc, uint32_t pim_addr,
	uint64_t pci_addr, uint64_t size);
static void apm_pcie_write_vendor_device_ids(struct apm_pcie_softc* sc);
static void apm_pcie_linkup_status(struct apm_pcie_softc* sc);
static bool apm_pcie_hide_root_cmplx_bars(struct apm_pcie_softc* sc, u_int bus, u_int reg);
static int apm_pcie_setup_interrupt(device_t dev, device_t child, struct resource *irq,
	    int flags, driver_filter_t *filter, driver_intr_t *intr, void *arg,
	    void **cookiep);

int apm_msix_alloc(int,int*);
int apm_msix_map_msi(int,bus_addr_t*,uint32_t*);
int apm_msix_release(int,int*);
int apm_msix_setup_irq(device_t,device_t,struct resource*,int,driver_filter_t*,
		driver_intr_t*,void*,void**);
/*
 * Newbus interface declarations
 */
static device_method_t apm_pcie_methods[] = {
	DEVMETHOD(device_probe,			apm_pcie_probe),
	DEVMETHOD(device_attach,		apm_pcie_attach),

	DEVMETHOD(pcib_maxslots,		apm_pcie_maxslots),
	DEVMETHOD(pcib_read_config,		apm_pcie_read_config),
	DEVMETHOD(pcib_write_config,	apm_pcie_write_config),
	DEVMETHOD(pcib_map_msi,			apm_pcie_map_msi),
	DEVMETHOD(pcib_alloc_msix,		apm_pcie_alloc_msix),
	DEVMETHOD(pcib_release_msix,	apm_pcie_release_msix),

	DEVMETHOD(bus_read_ivar,			apm_pcie_read_ivar),
	DEVMETHOD(bus_write_ivar,			apm_pcie_write_ivar),
	DEVMETHOD(bus_alloc_resource,		apm_pcie_alloc_resource),
	DEVMETHOD(bus_release_resource,		apm_pcie_release_resource),

	DEVMETHOD(bus_activate_resource,	bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	bus_generic_deactivate_resource),
	DEVMETHOD(bus_setup_intr,			apm_pcie_setup_interrupt),
	DEVMETHOD(bus_teardown_intr,		bus_generic_teardown_intr),

	DEVMETHOD_END
};

static driver_t apm_pcie_driver = {
	"pcib",
	apm_pcie_methods,
	sizeof(struct apm_pcie_softc),
};

static devclass_t apm_pcie_devclass;

DRIVER_MODULE(pcib, ofwbus, apm_pcie_driver,
apm_pcie_devclass, 0, 0);
DRIVER_MODULE(pcib, simplebus, apm_pcie_driver,
apm_pcie_devclass, 0, 0);


static inline uint32_t
lower_32_bits(uint64_t val)
{
	return ((uint32_t)(val & 0xFFFFFFFF));
}

static inline uint32_t
higher_32_bits(uint64_t val)
{
	return ((uint32_t)(val >> 32));
}
/*
 * PCIe write/read configuration space
 */
static inline void
apm_pcie_cfg_w32(struct apm_pcie_softc* sc, u_int reg, uint32_t val)
{
	apm_pcie_cfg_write(reg, val);
}

static inline void
apm_pcie_cfg_w16(struct apm_pcie_softc* sc, u_int reg, uint16_t val)
{
	uint32_t val32 = apm_pcie_cfg_read(reg & ~0x3);

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
	apm_pcie_cfg_write(reg & ~0x3, val32);
}

static inline void
apm_pcie_cfg_w8(struct apm_pcie_softc* sc, u_int reg, uint8_t val)
{
	uint32_t val32 = apm_pcie_cfg_read(reg & ~0x3);

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
	apm_pcie_cfg_write(reg & ~0x3, val32);
}

static inline uint32_t
apm_pcie_cfg_r32(struct apm_pcie_softc* sc, u_int reg)
{
	uint32_t retval = apm_pcie_cfg_read(reg);	
	return (retval);
}

static inline uint16_t
apm_pcie_cfg_r16(struct apm_pcie_softc* sc, u_int reg)
{
	uint32_t val = apm_pcie_cfg_read(reg & ~0x3);

	switch (reg & 0x3) {
	case 2:
		val >>= 16;
		break;
	}

	return((uint16_t)(val &= 0xFFFF));
}

static inline uint8_t
apm_pcie_cfg_r8(struct apm_pcie_softc* sc, u_int reg) {
	uint32_t val = apm_pcie_cfg_read(reg & ~0x3);

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
apm_pcie_set_rtdid_reg(struct apm_pcie_softc* sc,
									u_int bus, u_int slot, u_int func)
{
	uint32_t rtdid_val = 0, retval;

	if (sc->mode == ROOT_CMPLX)	{
		rtdid_val = (bus << 8) | (slot << 3) | func;
	}

	apm_pcie_csr_write(APM_RTDID, rtdid_val);
	/* Read register to make sure flush was done */
	retval = apm_pcie_csr_read(APM_RTDID);

	if ((retval & 0xFFFF) != rtdid_val)	{
		printdbg("RTDID value set failed!\n");
	}
}

/*
 * When the address bit [17:16] is 2'b01, the Configuration access will be
 * treated as Type 1 and it will be forwarded to external PCIe device.
 */
static void
apm_pcie_set_cfg_offset(struct apm_pcie_softc* sc, u_int bus)
{
	if (bus >= 1) {
		sc->offset = APM_AXI_EP_CFG_ACCESS;
	} else {
		sc->offset = 0;
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
static bool
apm_pcie_hide_root_cmplx_bars(struct apm_pcie_softc* sc, u_int bus, u_int reg)
{
	bool retval = false;
	if ((bus == 0) && (sc->mode == ROOT_CMPLX) &&
			((reg == PCIR_BAR(0)) || (reg == PCIR_BAR(1))))
		retval = true;

	return (retval);
}

/* clear BAR configuration which was done by firmware */
static void
apm_pcie_clear_firmware_config(struct apm_pcie_softc* sc)
{
	int i;
	for (i = APM_PIM1_1L; i <= APM_CFGCTL; i += 4) {
		apm_pcie_csr_write(i, 0x0);
	}
}

/* parse data from Device Tree and set memory windows */
static u_int
apm_pcie_parse_fdt_ranges(struct apm_pcie_softc *sc)
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
		device_printf(sc->dev,
		"ERROR: unexpected values of address or size cells in FDT\n");
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
		}
		else if (flags & OFW_PCI_PHYS_HI_SPACE_MEM32) {
			range = &sc->pci_res.mem;
			type = SYS_RES_MEMORY;
		}
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

		if (bootverbose)
			device_printf(sc->dev, "\tCPU: 0x%016lx..0x%016lx -> PCI: 0x%016lx\n",
					range->cpu_addr,
					range->cpu_addr + range->size - 1,
					range->pci_addr);

		if (apm_pcie_map_range(sc, range, type)) {
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
		device_printf(sc->dev, "ERROR: wrong FDT 'dma-ranges' property\n");
		return (ENXIO);
	}
	tuples_count = cells_count /
			    (pci_addr_cells + parent_addr_cells + size_cells);
	if (tuples_count != APM_DMA_RANGES_COUNT) {
		device_printf(sc->dev,
			"ERROR: unexpected number of 'dma-ranges' tuples in FDT\n");
		retval = ENXIO;
		goto out;
	}

	for (i = 0 ; i < APM_DMA_RANGES_COUNT; i++)	{
		bzero(&sc->pci_res.dma_ranges.dma[i], sizeof(sc->pci_res.dma_ranges.dma[i]));
	}

	cell_ptr = ranges_buf;
	for (touple = 0; touple < tuples_count; touple++) {
		if (touple < APM_DMA_RANGES_COUNT)
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

		if (bootverbose)
			device_printf(sc->dev, "\tDMA%d: 0x%016lx..0x%016lx -> PCI: 0x%016lx\n",
					touple,
					range->cpu_addr,
					range->cpu_addr + range->size - 1,
					range->pci_addr);

		apm_pcie_setup_ib_reg(sc, range->cpu_addr, range->pci_addr,
				range->size ,range->flags & OFW_PCI_PHYS_HI_PREFETCHABLE,
				&ib_reg_mask);
	}

out:
	free(ranges_buf, M_OFWPROP);
	return (retval);
}

/* write configuration for BAR */
static int
apm_pcib_init_bar(struct apm_pcie_softc *sc, int bus, int slot, int func,
    int barno)
{
	bus_addr_t *allocp;
	uint32_t addr_l, addr_h, mask, size;
	int reg, width;

	reg = PCIR_BAR(barno);

	apm_pcie_write_config(sc->dev, bus, slot, func, reg, ~0U, 4);
	size = apm_pcie_read_config(sc->dev, bus, slot, func, reg, 4);

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

	if (bootverbose)
		printf("PCI %u:%u:%u:%u: BAR%d (reg %x): size=%#x: addr_h=%#x addr_l=%#x\n",
		    device_get_unit(sc->dev), bus, slot, func, barno, reg,
		    size, addr_h, addr_l);

	apm_pcie_write_config(sc->dev, bus, slot, func, reg, addr_l, 4);
	if(width == 2)
		apm_pcie_write_config(sc->dev, bus, slot, func, reg + 4,
				addr_h, 4);
	return (width);
}

/* enumerate PCIe devices */
static int
apm_pcib_init(struct apm_pcie_softc *sc, int bus, int maxslot)
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
			hdrtype = apm_pcie_read_config(sc->dev, bus, slot,
			    func, PCIR_HDRTYPE, 1);

			if ((hdrtype & PCIM_HDRTYPE) > PCI_MAXHDRTYPE)
				continue;

			if (func == 0 && (hdrtype & PCIM_MFDEV))
				maxfunc = PCI_FUNCMAX;

			/* disable device mem/io function
			 * while programming init values */
			command = apm_pcie_read_config(sc->dev, bus, slot,
			    func, PCIR_COMMAND, 1);
			command &= ~(PCIM_CMD_MEMEN | PCIM_CMD_PORTEN);
			apm_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_COMMAND, command, 1);

			/* Program the base address registers. */
			maxbar = (hdrtype & PCIM_HDRTYPE) ? 1 : 6;
			bar = 0;
			while (bar < maxbar)
				bar += apm_pcib_init_bar(sc, bus, slot, func,
				    bar);

			/* Perform interrupt routing.
			intpin = apm_pcib_read_config(sc->dev, bus, slot,
			    func, PCIR_INTPIN, 1);
			intline = apm_pcib_route_int(sc, bus, slot, func,
			    intpin);
			apm_pcib_write_config(sc->sc_dev, bus, slot, func,
			    PCIR_INTLINE, intline, 1); */

			/* enable device mem/io function */
			command |= PCIM_CMD_MEMEN | PCIM_CMD_PORTEN |
			    PCIM_CMD_BUSMASTEREN;
			apm_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_COMMAND, command, 1);

			/*
			 * Handle PCI-PCI bridges
			 */
			class = apm_pcie_read_config(sc->dev, bus, slot,
			    func, PCIR_CLASS, 1);
			subclass = apm_pcie_read_config(sc->dev, bus, slot,
			    func, PCIR_SUBCLASS, 1);

			/* Allow only proper PCI-PCI briges */
			if (class != PCIC_BRIDGE || subclass != PCIS_BRIDGE_PCI)
				continue;

			secbus++;

			/* Program I/O decoder. */
			iobase = sc->pci_res.io.pci_addr;
			apm_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_IOBASEL_1, iobase >> 8, 1);
			apm_pcie_write_config(sc->dev, bus, slot, func,
				PCIR_IOBASEH_1, iobase >> 16, 2);

			iolimit = iobase + sc->pci_res.io.size - 1;
			apm_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_IOLIMITL_1, iolimit >> 8, 1);
			apm_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_IOLIMITH_1, iolimit >> 16, 2);

			/* Program (non-prefetchable) memory decoder. */
			apm_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_MEMBASE_1, 0x0010, 2);
			apm_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_MEMLIMIT_1, 0x000f, 2);

			/* Program prefetchable memory decoder. */
			membase = sc->pci_res.mem.cpu_addr;
			apm_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_PMBASEL_1, lower_32_bits(membase), 2);
			apm_pcie_write_config(sc->dev, bus, slot, func,
				PCIR_PMBASEH_1, higher_32_bits(membase), 4);

			memlimit = membase + sc->pci_res.mem.size - 1;
			apm_pcie_write_config(sc->dev, bus, slot, func,
				PCIR_PMLIMITL_1, lower_32_bits(memlimit), 2);
			apm_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_PMLIMITH_1, higher_32_bits(memlimit), 4);

			/* Read currect bus register configuration */
			old_pribus = apm_pcie_read_config(sc->dev, bus,
			    slot, func, PCIR_PRIBUS_1, 1);
			old_secbus = apm_pcie_read_config(sc->dev, bus,
			    slot, func, PCIR_SECBUS_1, 1);
			old_subbus = apm_pcie_read_config(sc->dev, bus,
			    slot, func, PCIR_SUBBUS_1, 1);

			if (bootverbose)
				printf("PCI: reading firmware bus numbers for "
				    "secbus = %d (bus/sec/sub) = (%d/%d/%d)\n",
				    secbus, old_pribus, old_secbus, old_subbus);

			new_pribus = bus;
			new_secbus = secbus;

			secbus = apm_pcib_init(sc, secbus,
			    (subclass == PCIS_BRIDGE_PCI) ? PCI_SLOTMAX : 0);

			new_subbus = secbus;

			if (bootverbose)
				printf("PCI: translate firmware bus numbers "
				    "for secbus %d (%d/%d/%d) -> (%d/%d/%d)\n",
				    secbus, old_pribus, old_secbus, old_subbus,
				    new_pribus, new_secbus, new_subbus);

			apm_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_PRIBUS_1, new_pribus, 1);
			apm_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_SECBUS_1, new_secbus, 1);
			apm_pcie_write_config(sc->dev, bus, slot, func,
			    PCIR_SUBBUS_1, new_subbus, 1);
		}
	}

	return (secbus);
}

static void
apm_pcie_write_vendor_device_ids(struct apm_pcie_softc* sc)
{
	uint32_t val;

	/* setup the vendor and device IDs correctly */
	val = (APM_PCIE_DEVICEID << 16) | APM_PCIE_VENDORID;
	apm_pcie_csr_write(APM_BRIDGE_CFG_0, val);
}

/* APM PCIe controller has 3 in-bound memory regions that
 * can be used depending on requested size */
static int
apm_pcie_select_ib_region(uint8_t* ib_reg_mask, uint64_t size)
{
	int retval = -1;

	if ((size > 4) && (size < SIZE_16M) &&
						!(*ib_reg_mask & (1 << 1))) {
		*ib_reg_mask |= (1 << 1);
		retval = 1;
	}

	else if ((size > SIZE_1K) && (size < SIZE_1T) &&
								!(*ib_reg_mask & (1 << 0))) {
		*ib_reg_mask |= (1 << 0);
		retval = 0;
	}

	else if ((size > SIZE_1M) && (size < SIZE_1T) &&
								!(*ib_reg_mask & (1 << 2))) {
		*ib_reg_mask |= (1 << 2);
		retval = 2;
	}

	return (retval);
}

static uint64_t
apm_pcie_set_ib_mask(struct apm_pcie_softc* sc,
		uint32_t addr, uint32_t flags, uint64_t size)
{
	uint64_t mask = (~(size - 1) & PCIM_BAR_MEM_BASE) | flags;
	uint32_t val32 = 0;
	uint32_t val;

	val32 = apm_pcie_csr_read(addr);
	val = (val32 & 0x0000ffff) | (lower_32_bits(mask) << 16);
	apm_pcie_csr_write(addr,val);

	val32 = apm_pcie_csr_read(addr + 0x04);
	val = (val32 & 0xffff0000) | (lower_32_bits(mask) >> 16);
	apm_pcie_csr_write(addr + 0x04, val);

	val32 = apm_pcie_csr_read(addr + 0x04);
	val = (val32 & 0x0000ffff) | (higher_32_bits(mask) << 16);
	apm_pcie_csr_write(addr + 0x04, val);

	val32 = apm_pcie_csr_read(addr + 0x08);
	val = (val32 & 0xffff0000) | (higher_32_bits(mask) >> 16);
	apm_pcie_csr_write(addr + 0x08, val);

	return mask;
}

static void
apm_pcie_setup_pims(struct apm_pcie_softc *sc, uint32_t pim_addr,
		uint64_t pci_addr, uint64_t size)
{
	apm_pcie_csr_write(pim_addr, lower_32_bits(pci_addr));
	apm_pcie_csr_write(pim_addr + 0x04,
			higher_32_bits(pci_addr) | APM_EN_COHERENCY);
	apm_pcie_csr_write(pim_addr + 0x10, lower_32_bits(size));
	apm_pcie_csr_write(pim_addr + 0x14, higher_32_bits(size));
}

static void
apm_pcie_setup_ib_reg(struct apm_pcie_softc* sc, uint64_t cpu_addr,
		uint64_t pci_addr, uint64_t size, bool prefetch_flag, uint8_t *ib_reg_mask)
{
	int region;
	uint32_t addr, pim_addr;
	uint64_t mask = ~(size - 1) | APM_EN_REG;
	uint32_t flags = PCIM_BAR_MEM_64;

	mask = ~(size - 1) | APM_EN_REG;
	region = apm_pcie_select_ib_region(ib_reg_mask, size);
	if (region < 0) {
		device_printf(sc->dev,"Invalid PCIe DMA-range configuration\n");
		return;
	}

	if (prefetch_flag)
		flags |= PCIM_BAR_MEM_PREFETCH;

	addr = (lower_32_bits(cpu_addr) & PCIM_BAR_MEM_BASE) | flags;

	switch (region) {
	case 0:
		apm_pcie_set_ib_mask(sc, APM_BRIDGE_CFG_4, flags, size);
		apm_pcie_cfg_write(PCIR_BAR(0), addr);
		apm_pcie_cfg_write(PCIR_BAR(1), higher_32_bits(cpu_addr));
		pim_addr = APM_PIM1_1L;
		break;
	case 1:
		apm_pcie_csr_write(APM_IBAR2, addr);
		apm_pcie_csr_write(APM_IR2MSK, lower_32_bits(mask));
		pim_addr = APM_PIM2_1L;
		break;
	case 2:
		apm_pcie_csr_write(APM_IBAR3L, addr);
		apm_pcie_csr_write(APM_IBAR3L + 0x04, cpu_addr >> 32);
		apm_pcie_csr_write(APM_IR3MSKL, lower_32_bits(mask));
		apm_pcie_csr_write(APM_IR3MSKL + 0x04, mask >> 32);
		pim_addr = APM_PIM3_1L;
		break;
	}

	apm_pcie_setup_pims(sc, pim_addr, pci_addr, ~(size - 1));
}

static void
apm_pcie_setup_outbound_reg(struct apm_pcie_softc* sc, uint32_t reg,
		uint64_t cpu_addr, uint64_t pci_addr, uint64_t size, int type)
{
	uint64_t mask = 0;
	uint32_t min_size;
	uint32_t flag = APM_EN_REG;

	switch (type) {
	case SYS_RES_MEMORY:
		min_size = SIZE_128M;
		break;

	case SYS_RES_IOPORT:
		min_size = 128;
		flag |= APM_OB_LO_IO;
		break;
	}

	if (size >= min_size)
		mask = ~(size - 1) | flag;
	else
		device_printf(sc->dev, "res size %#lx less than minimum %#x\n",
			 (uint64_t)size, min_size);

	/* Write addresses to CSR registers */
	apm_pcie_csr_write(reg, lower_32_bits(cpu_addr));
	apm_pcie_csr_write(reg + 0x04, higher_32_bits(cpu_addr));

	apm_pcie_csr_write(reg + 0x08, lower_32_bits(mask));
	apm_pcie_csr_write(reg + 0x0c, higher_32_bits(mask));

	apm_pcie_csr_write(reg + 0x10, lower_32_bits(pci_addr));
	apm_pcie_csr_write(reg + 0x14, higher_32_bits(pci_addr));
}

static void
apm_pcie_setup_cfg_reg(struct apm_pcie_softc* sc)
{
	vm_paddr_t addr;
	/* get memory space physical address */
	addr = pmap_kextract(rman_get_bushandle(sc->res[APM_PCIE_CFG]));
	printdbg("Bus handle = %#lx\n", addr);

	apm_pcie_csr_write(APM_CFGBARL, lower_32_bits(addr));
	apm_pcie_csr_write(APM_CFGBARH, higher_32_bits(addr));

	apm_pcie_csr_write(APM_CFGCTL, APM_EN_REG);
}

static int
apm_pcie_map_range(struct apm_pcie_softc* sc, struct range *range, int type)
{
	device_t self = sc->dev;
	int err;
	struct rman *rm;
	char *descr;
	uint32_t ob_reg;
	bus_addr_t start, end;

	switch (type) {
	case SYS_RES_MEMORY:
		descr = "PCIe Memory resource";
		rm = &sc->pci_res.mem_rman;

		start = range->cpu_addr;
		end = range->cpu_addr + range->size -1;

		ob_reg = APM_OMR1BARL;
		sc->pci_res.mem_alloc = range->cpu_addr;
		break;

	case SYS_RES_IOPORT:
		descr = "PCIe I/O resource";
		rm = &sc->pci_res.io_rman;

		start = range->pci_addr;
		end = range->pci_addr + range->size -1;

		ob_reg = APM_OMR3BARL;
		sc->pci_res.io_alloc = range->pci_addr;
		break;
	default:
		return (ERANGE);
	}

	rm->rm_type = RMAN_ARRAY;
	rm->rm_descr = descr;
	err = rman_init(rm);
	if (err) {
		device_printf(self, "ERROR: rman_init() for %s failed (err: %d)\n", descr, err);
		return (err);
	}

	err = rman_manage_region(rm, start, end);
	if (err) {
		device_printf(self, "ERROR: rman_manage_region() for %s failed (err: %d)\n", descr, err);
		rman_fini(rm);
		return (err);
	}

	/* Setup outbound regions in controller's register */
	apm_pcie_setup_outbound_reg(sc, ob_reg, range->cpu_addr,
			range->pci_addr, range->size, type);

	return (0);
}

static void
apm_pcie_linkup_status(struct apm_pcie_softc* sc)
{
	uint32_t val;

	val = apm_pcie_csr_read(APM_PCIECORE_CTLANDSTATUS);
	if (val & APM_LINKUP_MASK)
		device_printf(sc->dev,"(RC) link up\n");
	else
		device_printf(sc->dev,"(RC) link down\n");
}


static void
apm_pcie_setup(struct apm_pcie_softc *sc)
{
	apm_pcie_clear_firmware_config(sc);

	apm_pcie_write_vendor_device_ids(sc);

	apm_pcie_setup_cfg_reg(sc);

	apm_pcie_linkup_status(sc);
}

/*
 * Generic device interfacee
 */

static int
apm_pcie_probe(device_t self)
{
	if (!ofw_bus_status_okay(self))
		return (ENXIO);

	if (!ofw_bus_is_compatible(self, "apm,xgene-pcie"))
		return (ENXIO);

	device_set_desc(self, "APM Integrated PCI-Express Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
apm_pcie_attach(device_t self)
{
	struct apm_pcie_softc	*sc;
	uint32_t val;
	int err, maxslot = 0;

	sc = device_get_softc(self);
	sc->dev = self;

	sc->busnr = 0;

	err = bus_alloc_resources(self, apm_pcie_mem_spec, sc->res);
	if (err) {
		device_printf(self,
				"ERROR: cannot map resources memory (err: %d)\n", err);
		return (ENXIO);
	}

	/* Read class type of APM PCIe agent */
	val = apm_pcie_cfg_read(APM_PCIE_RESOURCE_TYPE);
	switch (val >> 8) {
	case APM_PCIE_ROOT_CMPLX_CLASS:
		sc->mode = ROOT_CMPLX;
		break;
	case APM_PCIE_ENDPOINT_CLASS:
		sc->mode = ENDPOINT;
		break;
	default:
		device_printf(self,
				"ERROR: firmware set inappropriate device class in register\n");
		return (ENXIO);
	}

	/* Parse FDT for memory windows */
	err = apm_pcie_parse_fdt_ranges(sc);
	if (err) {
		device_printf(self,
				"ERROR: parsing FDT ranges failed (err: %d)\n", err);
		return (ENXIO);
	}

	/* Init R/W mutex */
	if (!sc->mtx_init) {
		mtx_init(&sc->rw_mtx, "pcicfg", NULL, MTX_SPIN);
		sc->mtx_init = true;
	}

	/* Set up APM specific PCIe configuration */
	apm_pcie_setup(sc);

	maxslot = 0;
	apm_pcib_init(sc, sc->busnr, maxslot);

	device_add_child(self, "pci", -1);
	return (bus_generic_attach(self));
}

static uint32_t
apm_pcie_read_config(device_t dev, u_int bus, u_int slot,
    u_int func, u_int reg, int bytes)
{
	struct apm_pcie_softc* sc = device_get_softc(dev);
	uint32_t retval = ~0U;

	/* Root Complex can have only one function */
	if (sc->mode == ROOT_CMPLX && bus == 0 && (slot != 0 || func != 0)) {
		return (retval);
	}

	if (!apm_pcie_hide_root_cmplx_bars(sc, bus, reg)) {
		mtx_lock_spin(&sc->rw_mtx);

		apm_pcie_set_rtdid_reg(sc, bus, slot, func);
		apm_pcie_set_cfg_offset(sc, bus);
		switch (bytes) {
		case 1:
			retval = apm_pcie_cfg_r8(sc, reg);
			break;
		case 2:
			retval = apm_pcie_cfg_r16(sc, reg);
			break;
		case 4:
			retval = apm_pcie_cfg_r32(sc, reg);
			break;
		default:
			retval = ~0U;
		}

		mtx_unlock_spin(&sc->rw_mtx);
	}

	return (retval);
}

static void
apm_pcie_write_config(device_t dev, u_int bus, u_int slot,
    u_int func, u_int reg, uint32_t val, int bytes)
{
	struct apm_pcie_softc* sc = device_get_softc(dev);

	if (bus > 255 || slot > 31 || func > 7 || reg > 4095)
			return;

	if (!apm_pcie_hide_root_cmplx_bars(sc, bus, reg)) {

		mtx_lock_spin(&sc->rw_mtx);

		apm_pcie_set_rtdid_reg(sc, bus, slot, func);
		apm_pcie_set_cfg_offset(sc, bus);

		switch (bytes) {
		case 1:
			apm_pcie_cfg_w8(sc, reg, (uint8_t)val);
			break;
		case 2:
			apm_pcie_cfg_w16(sc, reg, (uint16_t)val);
			break;
		case 4:
			apm_pcie_cfg_w32(sc, reg, val);
			break;
		default:
			return;
		}

		mtx_unlock_spin(&sc->rw_mtx);
	}
}

static int
apm_pcie_maxslots(device_t dev)
{
	return (PCI_SLOTMAX);
}

static int
apm_pcie_read_ivar(device_t dev, device_t child, int index,
    uintptr_t *result)
{
	struct apm_pcie_softc *sc = device_get_softc(dev);

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
apm_pcie_write_ivar(device_t dev, device_t child, int index,
    uintptr_t value)
{
	struct apm_pcie_softc * sc = device_get_softc(dev);

	switch (index) {
	case PCIB_IVAR_BUS:
		sc->busnr = value;
		return (0);
	}

	return (ENOENT);
}

static struct resource*
apm_pcie_alloc_resource(device_t dev,
    device_t child, int type, int *rid, u_long start, u_long end,
    u_long count, u_int flags)
{
	struct apm_pcie_softc *sc = device_get_softc(dev);
	struct rman *rm = NULL;
	struct resource *res;

	switch (type) {
	case SYS_RES_IOPORT:
		rm = &sc->pci_res.io_rman;

		break;
	case SYS_RES_MEMORY:
		rm = &sc->pci_res.mem_rman;

		if ((start == 0UL) && (end == ~0UL)) {
			start = sc->pci_res.mem.cpu_addr;
			end = sc->pci_res.mem.cpu_addr + sc->pci_res.mem.size - 1;
			count = sc->pci_res.mem.size;
		}
		break;
	default:
		return (BUS_ALLOC_RESOURCE(device_get_parent(dev), dev,
			type, rid, start, end, count, flags));
	};

	printdbg("request: %#lx..%#lx\n", start, end);
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
apm_pcie_release_resource(device_t dev, device_t child,
    int type, int rid, struct resource *res)
{
	if (type != SYS_RES_IOPORT && type != SYS_RES_MEMORY)
		return (BUS_RELEASE_RESOURCE(device_get_parent(dev), child,
			type, rid, res));

	return (rman_release_resource(res));
}

static int
apm_pcie_map_msi(device_t pcib, device_t child, int irq,
    uint64_t *addr, uint32_t *data)
{
	return (apm_msix_map_msi(irq,addr,data));
}

static int
apm_pcie_alloc_msix(device_t pcib, device_t child, int *irq)
{
	return (apm_msix_alloc(1, irq));
}

static int
apm_pcie_release_msix(device_t pcib, device_t child, int irq)
{
	return (apm_msix_release(1, &irq));
}

static int
apm_pcie_setup_interrupt(device_t dev, device_t child, struct resource *irq,
	    int flags, driver_filter_t *filter, driver_intr_t *intr, void *arg,
	    void **cookiep)
{
	return (apm_msix_setup_irq(dev, child, irq, flags,
			filter, intr, arg, cookiep));
}
