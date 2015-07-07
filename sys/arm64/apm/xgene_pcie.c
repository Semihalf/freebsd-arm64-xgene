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
#include "xgene_pcie_var.h"

#define	XGENE_PCIE_DEBUG
#undef XGENE_PCIE_DEBUG

/*
 * For Configuration request, RTDID register is used as Bus Number,
 * Device Number and Function number of the header fields.
 */
static void
xgene_pcie_set_rtdid_reg(struct xgene_pcie_softc *sc, u_int bus, u_int slot,
    u_int func)
{
	uint32_t rtdid_val = 0, retval;

	rtdid_val = (bus << 8) | (slot << 3) | func;

	xgene_pcie_csr_write(XGENE_RTDID, rtdid_val);
	/* Read register to make sure flush was done */
	retval = xgene_pcie_csr_read(XGENE_RTDID);

	if ((retval & 0xFFFF) != rtdid_val)	{
		device_printf(sc->dev, "RTDID value set failed!\n");
	}
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

/*
 * X-Gene PCIe port uses BAR0-BAR1 of RC's configuration space as
 * the translation from PCI bus to native BUS.  Entire DDR region
 * is mapped into PCIe space using these registers, so it can be
 * reached by DMA from EP devices.  The BAR0/1 of bridge should be
 * hidden during enumeration to avoid the sizing and resource allocation
 * by PCIe core.
 */
static boolean_t
xgene_pcie_hide_root_cmplx_bars(struct xgene_pcie_softc *sc, u_int bus,
    u_int reg)
{
	boolean_t retval = false;

	if ((bus == 0) &&
	    ((reg == PCIR_BAR(0)) || (reg == PCIR_BAR(1))))
		retval = true;

	return (retval);
}

/* Clear BAR, PIM & POM configuration which was done by firmware */
static void
xgene_pcie_clear_firmware_config(struct xgene_pcie_softc *sc)
{
	int i;

	for (i = XGENE_PIM1_1L; i <= XGENE_CFGCTL; i += 4)
		xgene_pcie_csr_write(i, 0x0);
}

/* Parse data from Device Tree and set memory windows */
static int
xgene_pcie_setup_memory_windows(struct xgene_pcie_softc *sc)
{
	uint8_t ib_reg_mask = 0;
	struct range *range;
	int retval;

	/* setup memory window */
	range = &sc->pci_res.mem;
	retval = xgene_pcie_map_range(sc, range, SYS_RES_MEMORY);
	if (retval != 0)
		goto err;

	/* setup IO window */
	range = &sc->pci_res.io;
	retval = xgene_pcie_map_range(sc, range, SYS_RES_IOPORT);
	if (retval != 0)
		goto err;

	/* setup DMA transaction windows */
	for (int i = 0; i < XGENE_DMA_RANGES_COUNT; i++) {
		range = &sc->pci_res.dma_ranges.dma[i];

		xgene_pcie_setup_ib_reg(sc, range->cpu_addr, range->pci_addr,
		range->size ,range->flags & OFW_PCI_PHYS_HI_PREFETCHABLE,
		&ib_reg_mask);
	}
err:
	return (retval);
}

static int
xgene_pcie_route_interrupt(device_t bus, device_t dev, int pin)
{
	struct xgene_pcie_softc *sc;
	struct ofw_pci_register reg;
	uint32_t pintr, mintr[4];
	int icells;
	phandle_t iparent;

	sc = device_get_softc(bus);
	pintr = pin;

	/* Fabricate imap information in case this isn't an OFW device */
	bzero(&reg, sizeof(reg));
	reg.phys_hi = (pci_get_bus(dev) << OFW_PCI_PHYS_HI_BUSSHIFT) |
	    (pci_get_slot(dev) << OFW_PCI_PHYS_HI_DEVICESHIFT) |
	    (pci_get_function(dev) << OFW_PCI_PHYS_HI_FUNCTIONSHIFT);

	icells = ofw_bus_lookup_imap(ofw_bus_get_node(dev), &sc->pci_iinfo,
	    &reg, sizeof(reg), &pintr, sizeof(pintr), mintr, sizeof(mintr),
	    &iparent);
	if (icells > 0)
		return (ofw_bus_map_intr(dev, iparent, icells, mintr));

	/* Maybe it's a real interrupt, not an intpin */
	if (pin > 4)
		return (pin);

	device_printf(bus, "could not route pin %d for device %d.%d\n",
	    pin, pci_get_slot(dev), pci_get_function(dev));
	return (PCI_INVALID_IRQ);
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

	/* check if BAR is 32 or 64 bit */
	width = ((size & PCIM_BAR_MEM_TYPE) == PCIM_BAR_MEM_64) ? 2 : 1;

	if (size & PCIM_BAR_IO_SPACE) {			/* I/O port */
		allocp = &sc->pci_res.io_alloc;
		/* clear bits 0-1 according to PCIe spec */
		size &= ~0x3;
		if ((size & 0xffff0000) == 0)
			size |= 0xffff0000;
	} else {					/* memory */
		allocp = &sc->pci_res.mem_alloc;
		/* clear bits 0-6 according to PCIe spec */
		size &= ~0x7F;
	}
	mask = ~size;
	size = mask + 1;
	/* Sanity check (must be a power of 2). */
	if (size & mask)
		return (width);

	/* align allocation address to size */
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
		xgene_pcie_csr_write(XGENE_IBAR3L + 0x04, 
					higher_32_bits(cpu_addr));
		xgene_pcie_csr_write(XGENE_IR3MSKL, lower_32_bits(mask));
		xgene_pcie_csr_write(XGENE_IR3MSKL + 0x04, 
					higher_32_bits(mask));
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
	if (err != 0) {
		device_printf(self,
		    "ERROR: rman_init() for %s failed (err: %d)\n", descr, err);
		return (err);
	}

	start = range->cpu_addr;
	end = range->cpu_addr + range->size -1;
	err = rman_manage_region(rm, start, end);
	if (err != 0) {
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
	uint8_t *ptr8;
	pcell_t *ptr32, size, imin[2], imax[2];
	int i;
	u_int min = UINT_MAX, max = 0, irq;

	const int column_num_in_fdt = 8;

	ofw_bus_setup_iinfo(ofw_bus_get_node(sc->dev),
			&sc->pci_iinfo, sizeof(pcell_t));

	ptr8 = sc->pci_iinfo.opi_imap;
	size = sc->pci_iinfo.opi_imapsz;

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
	printdbg("IRQ: min = %d max = %d\n", sc->irq_min, sc->irq_max);

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

	/* Clean and update basic information */
	xgene_pcie_clear_firmware_config(sc);
	xgene_pcie_write_vendor_device_ids(sc);

	/* Setup memory windows */
	if (xgene_pcie_setup_memory_windows(sc) != 0)
		return (ENXIO);

	/* Enable configuration registers and show status */
	xgene_pcie_setup_cfg_reg(sc);
	xgene_pcie_linkup_status(sc);

	return (0);
}

/*
 * Generic device interfacee
 */
int
xgene_pcie_attach(device_t self)
{
	struct xgene_pcie_softc *sc;
	uint32_t val;
	int err, maxslot = 0;

	sc = device_get_softc(self);

	sc->busnr = 0;

	err = bus_alloc_resources(self, xgene_pcie_mem_spec, sc->res);
	if (err != 0) {
		device_printf(self,
		    "ERROR: cannot map resources memory (err: %d)\n", err);
		return (ENXIO);
	}

	/* Read class type of APM PCIe agent */
	val = xgene_pcie_cfg_read(XGENE_PCIE_RESOURCE_TYPE);
	switch (val >> 8) {
	case XGENE_PCIE_ROOT_CMPLX_CLASS:
		xgene_pcie_init_irqs(sc);
		break;
	case XGENE_PCIE_ENDPOINT_CLASS:
		/* Currently end-point class is not supported,
		 * no break on purpose */
	default:
		device_printf(self, "ERROR: "
		    "firmware set inappropriate device class in register\n");
		/* release allocated resources */
		bus_release_resources(self, xgene_pcie_mem_spec,
							      sc->res);
		return (ENXIO);
	}

	/* Init R/W mutex */
	if (!sc->mtx_init) {
		mtx_init(&sc->rw_mtx, "pcicfg", NULL, MTX_SPIN);
		sc->mtx_init = true;
	}

	/* Set up X-Gene specific PCIe configuration */
	err = xgene_pcie_setup(sc);
	if (err != 0) {
		/* release allocated resources */
		bus_release_resources(self, xgene_pcie_mem_spec,
							      sc->res);
		return (err);
	}

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
	if (bus == 0 && (slot != 0 || func != 0))
		return (retval);

	if (!xgene_pcie_hide_root_cmplx_bars(sc, bus, reg)) {
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
	}

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
		break;
	}

	mtx_unlock_spin(&sc->rw_mtx);
	return;
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
xgene_pcie_xlate_addr_pci_to_cpu(struct xgene_pcie_softc *sc, 
    bus_addr_t bus_addr, int type)
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
			end = sc->pci_res.mem.cpu_addr + 
				    sc->pci_res.mem.size - 1;
			count = sc->pci_res.mem.size;
		} else {
			start = xgene_pcie_xlate_addr_pci_to_cpu(sc, 
				    start, type);
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

	printdbg("translated to cpu addr: %#lx..%#lx\n", start, end);

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

/*
 * Newbus interface declarations
 */
static device_method_t xgene_pcie_methods[] = {
	DEVMETHOD(pcib_maxslots,		xgene_pcie_maxslots),
	DEVMETHOD(pcib_read_config,		xgene_pcie_read_config),
	DEVMETHOD(pcib_write_config,		xgene_pcie_write_config),
	DEVMETHOD(pcib_route_interrupt,		xgene_pcie_route_interrupt),

	DEVMETHOD(bus_read_ivar,		xgene_pcie_read_ivar),
	DEVMETHOD(bus_write_ivar,		xgene_pcie_write_ivar),
	DEVMETHOD(bus_alloc_resource,		xgene_pcie_alloc_resource),
	DEVMETHOD(bus_release_resource,		xgene_pcie_release_resource),

	DEVMETHOD(bus_activate_resource,	bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	bus_generic_deactivate_resource),
	DEVMETHOD(bus_setup_intr,		bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,		bus_generic_teardown_intr),

	DEVMETHOD_END
};

DEFINE_CLASS_0(pcib, xgene_pcie_driver, xgene_pcie_methods, sizeof(struct xgene_pcie_softc));
