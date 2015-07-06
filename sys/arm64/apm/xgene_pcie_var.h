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

#ifndef __XGENE_PCIE_H_
#define __XGENE_PCIE_H_

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

#ifdef XGENE_PCIE_DEBUG
#define printdbg(fmt, args...)		\
do {					\
	printf("%s(): ", __func__);	\
	printf(fmt, ##args);		\
} while (0)
#else
#define	printdbg(fmt, args...)
#endif

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
	struct mtx		rw_mtx;
	boolean_t		mtx_init;
	u_int			irq_min;
	u_int			irq_max;
	u_int			irq_alloc;
	struct ofw_bus_iinfo	pci_iinfo;
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
static int xgene_pcie_route_interrupt(device_t, device_t, int);

/* Bit masking helpers */
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

#endif /* __XGENE_PCIE_H_ */
