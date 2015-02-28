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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bitset.h>
#include <sys/bitstring.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/ktr.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/pciio.h>
#include <sys/pcpu.h>
#include <sys/proc.h>
#include <sys/cpuset.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <dev/pci/pcivar.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/intr.h>

#include "gic_v3_reg.h"
#include "gic_v3_var.h"

MALLOC_DEFINE(M_GIC_V3_ITS, "GICv3 ITS", GIC_V3_ITS_DEVSTR);

devclass_t gic_v3_its_devclass;

static int its_alloc_tables(struct gic_v3_its_softc *);
static void its_free_tables(struct gic_v3_its_softc *);
static int its_init_commandq(struct gic_v3_its_softc *);
static int its_init_cpu(struct gic_v3_its_softc *);
static int its_init_cpu_collection(struct gic_v3_its_softc *);

static int its_cmd_send(struct gic_v3_its_softc *, struct its_cmd_desc *);

static void its_cmd_mapc(struct gic_v3_its_softc *, struct its_col *, uint8_t);
static void its_cmd_mapvi(struct gic_v3_its_softc *, struct its_dev *, uint32_t,
    uint32_t);
static void its_cmd_mapi(struct gic_v3_its_softc *, struct its_dev *, uint32_t);
static void its_cmd_inv(struct gic_v3_its_softc *, struct its_dev *, uint32_t);
static void its_cmd_invall(struct gic_v3_its_softc *, struct its_col *);

static int lpi_init_conftable(struct gic_v3_its_softc *);
static int lpi_bitmap_init(struct gic_v3_its_softc *);
static int lpi_init_cpu(struct gic_v3_its_softc *);
static int lpi_config_cpu(struct gic_v3_its_softc *);

const char *its_ptab_cache[] = {
	[GITS_BASER_CACHE_NCNB] = "(NC,NB)",
	[GITS_BASER_CACHE_NC] = "(NC)",
	[GITS_BASER_CACHE_RAWT] = "(RA,WT)",
	[GITS_BASER_CACHE_RAWB] = "(RA,WB)",
	[GITS_BASER_CACHE_WAWT] = "(WA,WT)",
	[GITS_BASER_CACHE_WAWB] = "(WA,WB)",
	[GITS_BASER_CACHE_RAWAWT] = "(RAWA,WT)",
	[GITS_BASER_CACHE_RAWAWB] = "(RAWA,WB)",
};

const char *its_ptab_share[] = {
	[GITS_BASER_SHARE_NS] = "none",
	[GITS_BASER_SHARE_IS] = "inner",
	[GITS_BASER_SHARE_OS] = "outer",
	[GITS_BASER_SHARE_RES] = "none",
};

const char *its_ptab_type[] = {
	[GITS_BASER_TYPE_UNIMPL] = "Unimplemented",
	[GITS_BASER_TYPE_DEV] = "Devices",
	[GITS_BASER_TYPE_VP] = "Virtual Processors",
	[GITS_BASER_TYPE_PP] = "Physical Processors",
	[GITS_BASER_TYPE_IC] = "Interrupt Collections",
	[GITS_BASER_TYPE_RES5] = "Reserved (5)",
	[GITS_BASER_TYPE_RES6] = "Reserved (6)",
	[GITS_BASER_TYPE_RES7] = "Reserved (7)",
};

static struct gic_v3_its_softc *its_sc;

#define	gic_its_read(sc, len, reg)		\
({						\
	bus_read_##len(&sc->its_res[0],		\
	    reg);				\
})

#define	gic_its_write(sc, len, reg, val)	\
({						\
	bus_write_##len(&sc->its_res[0],	\
	    reg, val);				\
})

int
gic_v3_its_attach(device_t dev)
{
	struct gic_v3_its_softc *sc;
	uint64_t gits_tmp;
	uint32_t gits_pidr2;
	int rid;
	int ret;

	sc = device_get_softc(dev);

	rid = 0;
	sc->its_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->its_res == NULL)
		return (ENXIO);

	sc->dev = dev;

	gits_pidr2 = gic_its_read(sc, 4, GITS_PIDR2);
	switch (gits_pidr2 & GITS_PIDR2_ARCH_MASK) {
	case GITS_PIDR2_ARCH_GICv3: /* fall through */
	case GITS_PIDR2_ARCH_GICv4:
		if (bootverbose) {
			device_printf(dev, "ITS found. Architecture rev. %u\n",
			    (u_int)(gits_pidr2 & GITS_PIDR2_ARCH_MASK) >> 4);
		}
		break;
	default:
		device_printf(dev, "No ITS found in the system\n");
		ret = ENODEV;
		goto error;
	}

	/* 1. Initialize commands queue */
	ret = its_init_commandq(sc);
	if (ret)
		goto error;

	/* 2. Provide memory for any private ITS tables */
	ret = its_alloc_tables(sc);
	if (ret)
		goto error;

	/* 3. Allocate collections. One per-CPU */
	sc->its_cols = malloc(sizeof(*sc->its_cols) * MAXCPU,
	    M_GIC_V3_ITS, M_WAITOK | M_ZERO);

	/* 4. Enable ITS in GITS_CTLR */
	gits_tmp = gic_its_read(sc, 4, GITS_CTLR);
	gic_its_write(sc, 4, GITS_CTLR, gits_tmp | GITS_CTLR_EN);

	/* 5. Initialize LPIs configuration table */
	ret = lpi_init_conftable(sc);
	if (ret)
		goto error;

	/* 6. LPIs bitmap init */
	ret = lpi_bitmap_init(sc);
	if (ret)
		goto error;

	/* 7. CPU init */
	(void)its_init_cpu(sc);

	/* 8. Init ITS devices list */
	TAILQ_INIT(&sc->its_dev_list);

	arm_register_msi_pic(dev);

	/*
	 * XXX: We need to have ITS software context when.
	 * Being called by the interrupt code (mask/unmask).
	 * This may be used only when one ITS is present in
	 * the system and eventually should be removed.
	 */
	its_sc = sc;

	return (0);

error:
	gic_v3_its_detach(dev);
	return (ret);
}

/* Will not detach but use it for convenience */
int
gic_v3_its_detach(device_t dev)
{
	device_t parent;
	struct gic_v3_softc *gic_sc;
	struct gic_v3_its_softc *sc;
	u_int cpuid;
	int rid = 0;

	sc = device_get_softc(dev);
	cpuid = PCPU_GET(cpuid);

	/* Release what's possible */

	/* Command queue */
	if ((void *)sc->its_cmdq_base != NULL) {
		contigfree((void *)sc->its_cmdq_base,
		    ITS_CMDQ_SIZE, M_GIC_V3_ITS);
	}
	/* ITTs */
	its_free_tables(sc);
	/* Collections */
	free(sc->its_cols, M_GIC_V3_ITS);
	/* LPI config table */
	parent = device_get_parent(sc->dev);
	gic_sc = device_get_softc(parent);
	if ((void *)gic_sc->gic_redists.lpis.conf_base != NULL) {
		contigfree((void *)gic_sc->gic_redists.lpis.conf_base,
		    LPI_CONFTAB_SIZE, M_GIC_V3_ITS);
	}
	if ((void *)gic_sc->gic_redists.lpis.pend_base[cpuid] != NULL) {
		contigfree((void *)gic_sc->gic_redists.lpis.pend_base[cpuid],
		    roundup2(LPI_PENDTAB_SIZE, PAGE_SIZE_64K), M_GIC_V3_ITS);
	}

	/* Resource... */
	bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->its_res);

	return (0);
}

static int
its_alloc_tables(struct gic_v3_its_softc *sc)
{
	uint64_t gits_baser, gits_tmp;
	uint64_t type, esize, cache, share, psz;
	uint64_t gits_typer;
	size_t page_size, npages, nitspages, nidents, tn;
	size_t its_tbl_size;
	vm_offset_t ptab_vaddr;
	vm_paddr_t ptab_paddr;
	boolean_t first = TRUE;

	page_size = PAGE_SIZE_64K;

	/* Read features first */
	gits_typer = gic_its_read(sc, 8, GITS_TYPER);

	for (tn = 0; tn < GITS_BASER_NUM; tn++) {
		gits_baser = gic_its_read(sc, 8, GITS_BASER(tn));
		type = GITS_BASER_TYPE(gits_baser);
		/* Get the Table Entry size */
		esize = GITS_BASER_ESIZE(gits_baser);

		switch (type) {
		case GITS_BASER_TYPE_UNIMPL:	/* fall through */
		case GITS_BASER_TYPE_RES5:
		case GITS_BASER_TYPE_RES6:
		case GITS_BASER_TYPE_RES7:
			continue;
		case GITS_BASER_TYPE_DEV:
			nidents = (1 << GITS_TYPER_DEVB(gits_typer));
			its_tbl_size = esize * nidents;
			its_tbl_size = roundup2(its_tbl_size, page_size);
			npages = howmany(its_tbl_size, PAGE_SIZE);
			break;
		default:
			npages = howmany(page_size, PAGE_SIZE);
			break;
		}

		/* Allocate required space */
		ptab_vaddr = (vm_offset_t)contigmalloc(npages * PAGE_SIZE,
		    M_GIC_V3_ITS, M_WAITOK | M_ZERO, 0, ~0UL, PAGE_SIZE, 0);

		sc->its_ptabs[tn].ptab_vaddr = ptab_vaddr;
		sc->its_ptabs[tn].ptab_pgsz = PAGE_SIZE;
		sc->its_ptabs[tn].ptab_npages = npages;

		ptab_paddr = vtophys(ptab_vaddr);
		KASSERT((ptab_paddr & GITS_BASER_PA_MASK) == ptab_paddr,
		    ("%s: Unaligned PA for Interrupt Translation Table",
		    device_get_name(sc->dev)));

		/* Set defaults: WAWB, IS */
		cache = GITS_BASER_CACHE_WAWB;
		share = GITS_BASER_SHARE_IS;

		while (1) {
			nitspages = howmany(its_tbl_size, page_size);

			switch (page_size) {
			case (1 << 12):		/* 4KB */
				psz = GITS_BASER_PSZ_4K;
				break;
			case (1 << 14):		/* 16KB */
				psz = GITS_BASER_PSZ_4K;
				break;
			case (1 << 16):		/* 64KB */
				psz = GITS_BASER_PSZ_64K;
				break;
			default:
				/* XXX: Other page sizes are currently not supported */
				device_printf(sc->dev, "Unsupported page size: %zuKB\n",
				    page_size / 1024);
				its_free_tables(sc);
				return (ENXIO);
			}

			/* Clear fields under modification first */
			gits_baser &= ~(GITS_BASER_VALID |
			    GITS_BASER_CACHE_MASK | GITS_BASER_TYPE_MASK |
			    GITS_BASER_ESIZE_MASK | GITS_BASER_PA_MASK |
			    GITS_BASER_SHARE_MASK | GITS_BASER_PSZ_MASK);
			/* Construct register value */
			gits_baser |=
			    (type << GITS_BASER_TYPE_SHIFT) |
			    ((esize - 1) << GITS_BASER_ESIZE_SHIFT) |
			    (cache << GITS_BASER_CACHE_SHIFT) |
			    (share << GITS_BASER_SHARE_SHIFT) |
			    (psz << GITS_BASER_PSZ_SHIFT) |
			    ptab_paddr | (nitspages - 1) |
			    GITS_BASER_VALID;

			gic_its_write(sc, 8, GITS_BASER(tn), gits_baser);
			/*
			 * Verify.
			 * Depending on implementation we may encounter
			 * shareability and page size mismatch.
			 */
			gits_tmp = gic_its_read(sc, 8, GITS_BASER(tn));
			if ((gits_tmp ^ gits_baser) & GITS_BASER_SHARE_MASK) {
				share = gits_tmp & GITS_BASER_SHARE_MASK;
				share >>= GITS_BASER_SHARE_SHIFT;
				continue;
			}

			if ((gits_tmp ^ gits_baser) & GITS_BASER_PSZ_MASK) {
				switch (page_size) {
				case (1 << 14):
					page_size = (1 << 12);
					continue;
				case (1 << 16):
					page_size = (1 << 14);
					continue;
				}
			}
			/* We did what we could */
			break;
		}
		/*
		 * Do not compare Cacheability field since
		 * it is implementation defined.
		 */
		gits_tmp &= ~GITS_BASER_CACHE_MASK;
		gits_baser &= ~GITS_BASER_CACHE_MASK;

		if (gits_tmp != gits_baser) {
			device_printf(sc->dev,
			    "Could not allocate ITS tables\n");
			its_free_tables(sc);
			return (ENXIO);
		}

		if (bootverbose) {
			if (first) {
				device_printf(sc->dev,
				    "Allocated ITS private tables:\n");
				first = FALSE;
			}
			device_printf(sc->dev,
			       "\tPTAB%zu for %s: PA 0x%lx, %lu entries,"
			       " cache policy %s, %s shareable,"
			       " page size %zuKB\n", tn, its_ptab_type[type],
			       ptab_paddr, (page_size * nitspages) / esize,
			       its_ptab_cache[cache], its_ptab_share[share],
			       page_size / 1024);
		}
	}

	return (0);
}

static void
its_free_tables(struct gic_v3_its_softc *sc)
{
	vm_offset_t ptab_vaddr;
	size_t size;
	size_t tn;

	for (tn = 0; tn < GITS_BASER_NUM; tn++) {
		ptab_vaddr = sc->its_ptabs[tn].ptab_vaddr;
		if (!ptab_vaddr)
			continue;
		size = sc->its_ptabs[tn].ptab_pgsz;
		size *= sc->its_ptabs[tn].ptab_npages;

		if ((void *)ptab_vaddr != NULL)
			contigfree((void *)ptab_vaddr, size, M_GIC_V3_ITS);

		/* Clear the table description */
		memset(&sc->its_ptabs[tn], 0,
		    sizeof(sc->its_ptabs[tn]));
	}
}

static int
its_init_commandq(struct gic_v3_its_softc *sc)
{
	uint64_t gits_cbaser, gits_tmp;
	uint64_t cache, share;
	vm_paddr_t cmdq_paddr;
	device_t dev;

	dev = sc->dev;
	/* Allocate memory for command queue */
	sc->its_cmdq_base = contigmalloc(ITS_CMDQ_SIZE, M_GIC_V3_ITS,
	    (M_WAITOK | M_ZERO),
	    0, ~0UL, ITS_CMDQ_SIZE, 0);
	/* Set command queue write pointer (command queue empty) */
	sc->its_cmdq_write = sc->its_cmdq_base;

	/* Save command queue pointer and attributes */
	cmdq_paddr = vtophys(sc->its_cmdq_base);
	KASSERT((cmdq_paddr & GITS_CBASER_PA_MASK) == cmdq_paddr,
	    ("%s: Unaligned PA for ITS Commands Queue", device_get_name(dev)));

	/* Set defaults: Normal Inner WAWB, IS */
	cache = GITS_CBASER_CACHE_NIWAWB;
	share = GITS_CBASER_SHARE_IS;

	gits_cbaser = (cmdq_paddr |
	    (cache << GITS_CBASER_CACHE_SHIFT) |
	    (share << GITS_CBASER_SHARE_SHIFT) |
	    /* Number of 4KB pages - 1 */
	    (ITS_CMDQ_SIZE / (1 << 12) - 1) |
	    /* Valid bit */
	    GITS_CBASER_VALID);

	gic_its_write(sc, 8, GITS_CBASER, gits_cbaser);
	gits_tmp = gic_its_read(sc, 8, GITS_CBASER);

	if ((gits_tmp ^ gits_cbaser) & GITS_CBASER_SHARE_MASK) {
		if (bootverbose) {
			device_printf(dev,
			    "Will use cache flushing for commands queue\n");
		}
		/* Command queue needs cache flushing */
		sc->its_flags |= ITS_FLAGS_CMDQ_FLUSH;
	}

	gic_its_write(sc, 8, GITS_CWRITER, 0x0);

	return (0);
}

static int
its_init_cpu(struct gic_v3_its_softc *sc)
{
	device_t parent;
	struct gic_v3_softc *gic_sc;

	/*
	 * Check for LPIs support on this Re-Distributor.
	 */
	parent = device_get_parent(sc->dev);
	gic_sc = device_get_softc(parent);
	if (!(gic_r_read(gic_sc, 4, GICR_TYPER) & GICR_TYPER_PLPIS)) {
		if (bootverbose) {
			device_printf(sc->dev,
			    "LPIs not supported on CPU%u\n", PCPU_GET(cpuid));
		}
		return (ENXIO);
	}

	if (lpi_init_cpu(sc))
		return (ENXIO);

	/* Init collections */
	its_init_cpu_collection(sc);

	return (0);
}

static int
its_init_cpu_collection(struct gic_v3_its_softc *sc)
{
	device_t parent;
	struct gic_v3_softc *gic_sc;
	uint64_t typer;
	uint64_t target;
	vm_offset_t redist_base;
	u_int cpuid;

	cpuid = PCPU_GET(cpuid);
	parent = device_get_parent(sc->dev);
	gic_sc = device_get_softc(parent);

	typer = gic_its_read(sc, 8, GITS_TYPER);
	if (typer & GITS_TYPER_PTA) {
		redist_base =
		    rman_get_bushandle(gic_sc->gic_redists.pcpu[cpuid]);
		/*
		 * Target Address correspond to the base physical
		 * address of Re-Distributors.
		 */
		target = vtophys(redist_base);
	} else {
		/* Target Address correspond to unique processor numbers */
		typer = gic_r_read(gic_sc, 8, GICR_TYPER);
		target = GICR_TYPER_CPUNUM(typer);
	}

	sc->its_cols[cpuid].col_target = target;
	sc->its_cols[cpuid].col_id = cpuid;

	its_cmd_mapc(sc, &sc->its_cols[cpuid], 1);
	its_cmd_invall(sc, &sc->its_cols[cpuid]);

	return (0);
}

static int
lpi_init_conftable(struct gic_v3_its_softc *sc)
{
	device_t parent;
	struct gic_v3_softc *gic_sc;
	vm_offset_t conf_base;
	uint8_t prio_default;

	parent = device_get_parent(sc->dev);
	gic_sc = device_get_softc(parent);
	/*
	 * LPI Configuration Table settings.
	 * Notice that Configuration Table is shared among all
	 * Re-Distributors, so this is going to be created just once.
	 */
	conf_base = (vm_offset_t)contigmalloc(LPI_CONFTAB_SIZE,
	    M_GIC_V3_ITS, M_WAITOK | M_ZERO, 0, ~0UL, PAGE_SIZE_64K, 0);

	KASSERT((vtophys(conf_base) & PAGE_MASK_64K) == 0,
	    ("LPI Configuration Table not aligned to 64 KB"));

	if (!conf_base) {
		if (bootverbose) {
			device_printf(sc->dev,
			    "Could not allocate memory for LPI "
			    "Configuration Table\n");
		}
		return (ENOMEM);
	}

	if (bootverbose) {
		device_printf(sc->dev,
		    "LPI Configuration Table at PA: 0x%lx\n",
		    vtophys(conf_base));
	}

	/*
	 * Let the default priority be aligned with all other
	 * interrupts assuming that each interrupt is assigned
	 * MAX priority at startup. MAX priority on the other
	 * hand cannot be higher than 0xFC for LPIs.
	 */
	prio_default = GIC_PRIORITY_MAX;

	/* Write each settings byte to LPI configuration table */
	memset((void *)conf_base,
	    (prio_default & LPI_CONF_PRIO_MASK) | LPI_CONF_GROUP1,
	    LPI_CONFTAB_SIZE);

	/* XXX ARM64TODO: Clean dcache under table here */

	gic_sc->gic_redists.lpis.conf_base = conf_base;

	return (0);
}

static int
lpi_init_cpu(struct gic_v3_its_softc *sc)
{
	device_t parent;
	struct gic_v3_softc *gic_sc;
	vm_offset_t pend_base;
	u_int cpuid;

	parent = device_get_parent(sc->dev);
	gic_sc = device_get_softc(parent);

	/*
	 * LPI Pending Table settings.
	 * This has to be done for each Re-Distributor, hence for each CPU.
	 */
	cpuid = PCPU_GET(cpuid);

	pend_base = (vm_offset_t)contigmalloc(
	    roundup2(LPI_PENDTAB_SIZE, PAGE_SIZE_64K), M_GIC_V3_ITS,
	    (M_WAITOK | M_ZERO), 0, ~0UL, PAGE_SIZE_64K, 0);

	KASSERT((vtophys(pend_base) & PAGE_MASK_64K) == 0,
	    ("LPI Pending Table not aligned to 64 KB"));

	if (!pend_base) {
		if (bootverbose) {
			device_printf(sc->dev,
			    "Could not allocate memory for LPI "
			    "Pending Table on CPU%u\n", cpuid);
		}
		return (ENOMEM);
	}
	if (bootverbose) {
		device_printf(sc->dev,
		    "LPI Pending Table for CPU%u at PA: 0x%lx\n",
		    cpuid, vtophys(pend_base));
	}

	gic_sc->gic_redists.lpis.pend_base[cpuid] = pend_base;

	lpi_config_cpu(sc);

	return (0);
}

static int
lpi_config_cpu(struct gic_v3_its_softc *sc)
{
	device_t parent;
	struct gic_v3_softc *gic_sc;
	vm_offset_t conf_base, pend_base;
	uint64_t gicr_xbaser, gicr_temp;
	uint64_t cache, share, idbits;
	uint32_t gicr_ctlr;
	u_int cpuid;

	parent = device_get_parent(sc->dev);
	gic_sc = device_get_softc(parent);
	cpuid = PCPU_GET(cpuid);

	conf_base = gic_sc->gic_redists.lpis.conf_base;
	pend_base = gic_sc->gic_redists.lpis.pend_base[cpuid];

	/* Disable LPIs */
	gicr_ctlr = gic_r_read(gic_sc, 4, GICR_CTLR);
	gicr_ctlr &= ~GICR_CTLR_LPI_ENABLE;
	gic_r_write(gic_sc, 4, GICR_CTLR, gicr_ctlr);
	/* Assuming full system barrier */
	dsb();

	/*
	 * Set GICR_PROPBASER
	 */

	/*
	 * Find out how many bits do we need for LPI identifiers.
	 * Remark 1.: Even though we have (LPI_CONFTAB_SIZE / 8) LPIs
	 *	      the notified LPI ID still starts from 8192.
	 * Remark 2.: This could be done on compilation time but there
	 *	      seems to be no sufficient macro.
	 */
	idbits = flsl(LPI_CONFTAB_SIZE + 8192) - 1;

	/* Set defaults: Normal Inner WAWB, IS */
	cache = GICR_PROPBASER_CACHE_NIWAWB;
	share = GICR_PROPBASER_SHARE_IS;

	gicr_xbaser = vtophys(conf_base) |
	    ((idbits - 1) & GICR_PROPBASER_IDBITS_MASK) |
	    (cache << GICR_PROPBASER_CACHE_SHIFT) |
	    (share << GICR_PROPBASER_SHARE_SHIFT);

	gic_r_write(gic_sc, 8, GICR_PROPBASER, gicr_xbaser);
	gicr_temp = gic_r_read(gic_sc, 8, GICR_PROPBASER);

	if ((gicr_xbaser ^ gicr_temp) & GICR_PROPBASER_SHARE_MASK) {
		if (bootverbose) {
			device_printf(sc->dev,
			    "Will use cache flushing for LPI "
			    "Configuration Table\n");
		}
		gic_sc->gic_redists.lpis.flags |= LPI_FLAGS_CONF_FLUSH;
	}

	/*
	 * Set GICR_PENDBASER
	 */

	/* Set defaults: Normal Inner WAWB, IS */
	cache = GICR_PENDBASER_CACHE_NIWAWB;
	share = GICR_PENDBASER_SHARE_IS;

	gicr_xbaser = vtophys(pend_base) |
	    (cache << GICR_PENDBASER_CACHE_SHIFT) |
	    (share << GICR_PENDBASER_SHARE_SHIFT);

	gic_r_write(gic_sc, 8, GICR_PENDBASER, gicr_xbaser);

	/* Enable LPIs */
	gicr_ctlr = gic_r_read(gic_sc, 4, GICR_CTLR);
	gicr_ctlr |= GICR_CTLR_LPI_ENABLE;
	gic_r_write(gic_sc, 4, GICR_CTLR, gicr_ctlr);

	dsb();

	return (0);
}

static int
lpi_bitmap_init(struct gic_v3_its_softc *sc)
{
	device_t parent;
	struct gic_v3_softc *gic_sc;
	uint32_t lpi_id_num;
	size_t lpi_chunks_num;
	size_t bits_in_chunk;

	parent = device_get_parent(sc->dev);
	gic_sc = device_get_softc(parent);

	lpi_id_num = (1 << gic_sc->gic_idbits) - 1;
	/* Substract IDs dedicated for SGIs, PPIs and SPIs */
	lpi_id_num -= 8192;

	sc->its_lpi_maxid = lpi_id_num;

	bits_in_chunk = sizeof(*sc->its_lpi_bitmap) * NBBY;

	/*
	 * Round up to the number of bits in chunk.
	 * We will need to take care to avoid using invalid LPI IDs later.
	 */
	lpi_id_num = roundup2(lpi_id_num, bits_in_chunk);
	lpi_chunks_num = lpi_id_num / bits_in_chunk;

	sc->its_lpi_bitmap =
	    contigmalloc((lpi_chunks_num * sizeof(*sc->its_lpi_bitmap)),
	    M_GIC_V3_ITS, M_WAITOK | M_ZERO, 0, ~0UL,
	    sizeof(*sc->its_lpi_bitmap), 0);

	return (0);
}

static int
lpi_alloc_chunk(struct gic_v3_its_softc *sc, struct lpi_chunk *lpic,
    u_int nvecs)
{
	int fclr; /* First cleared bit */
	uint8_t *bitmap;
	size_t nb, i;

	bitmap = (uint8_t *)sc->its_lpi_bitmap;

	fclr = 0;
retry:
	/* Check other bits - sloooow */
	for (i = 0, nb = fclr; i < nvecs; i++, nb++) {
		if (nb > sc->its_lpi_maxid)
			return (EINVAL);

		if (isset(bitmap, nb)) {
			/* To little free bits in this area. Move on. */
			fclr = nb + 1;
			goto retry;
		}
	}
	/* This area is free. Take it. */
	bit_nset(bitmap, fclr, fclr + nvecs - 1);
	lpic->lpi_base = fclr + 8192;
	lpic->lpi_num = nvecs;
	lpic->lpi_free = lpic->lpi_num;

	return (0);
}

static void
lpi_configure(struct gic_v3_its_softc *sc, struct its_dev *its_dev,
    uint32_t lpinum, boolean_t unmask)
{
	device_t parent;
	struct gic_v3_softc *gic_sc;
	uint8_t *conf_byte;

	parent = device_get_parent(sc->dev);
	gic_sc = device_get_softc(parent);

	conf_byte = (uint8_t *)gic_sc->gic_redists.lpis.conf_base;
	conf_byte += (lpinum - 8192);

	if (unmask)
		*conf_byte |= LPI_CONF_ENABLE;
	else
		*conf_byte &= ~LPI_CONF_ENABLE;

	if (gic_sc->gic_redists.lpis.flags & LPI_FLAGS_CONF_FLUSH) {
		/* XXX ARM64TODO: Flush the cache when we have it enabled */
	} else {
		/* XXX Inner shareable store is enough */
		dsb();
	}

	its_cmd_inv(sc, its_dev, lpinum);
}

void
lpi_unmask_irq(device_t parent, uint32_t irq)
{
	struct its_dev *its_dev;

	TAILQ_FOREACH(its_dev, &its_sc->its_dev_list, entry) {
		if (irq >= its_dev->lpis.lpi_base &&
		    irq < (its_dev->lpis.lpi_base + its_dev->lpis.lpi_num)) {
			lpi_configure(its_sc, its_dev, irq, 1);
			return;
		}
	}

	KASSERT(0, ("Trying to unmaks not existing LPI: %u\n", irq));
}

void
lpi_mask_irq(device_t parent, uint32_t irq)
{
	struct its_dev *its_dev;

	TAILQ_FOREACH(its_dev, &its_sc->its_dev_list, entry) {
		if (irq >= its_dev->lpis.lpi_base &&
		    irq < (its_dev->lpis.lpi_base + its_dev->lpis.lpi_num)) {
			lpi_configure(its_sc, its_dev, irq, 0);
			return;
		}
	}

	KASSERT(0, ("Trying to mask not existing LPI: %u\n", irq));
}

/*
 * Commands handling.
 */

static __inline void
cmd_format_command(struct its_cmd *cmd, uint8_t cmd_type)
{
	/* Command field: DW0 [7:0] */
	cmd->cmd_dword[0] &= ~0xFFUL;
	cmd->cmd_dword[0] |= cmd_type & 0xFF;
}

static __inline void
cmd_format_devid(struct its_cmd *cmd, uint32_t devid)
{
	/* Device ID field: DW0 [63:32] */
	cmd->cmd_dword[0] &= ~(0xFFFFFFFFUL << 32);
	cmd->cmd_dword[0] |= ((uint64_t)devid << 32);
}

static __inline void
cmd_format_size(struct its_cmd *cmd, uint16_t size)
{
	/* Size field: DW1 [4:0] */
	cmd->cmd_dword[1] &= ~0xFFUL;
	cmd->cmd_dword[1] |= (size & 0xFF);
}

static __inline void
cmd_format_id(struct its_cmd *cmd, uint32_t id)
{
	/* ID field: DW1 [31:0] */
	cmd->cmd_dword[1] &= ~0xFFFFFFFFUL;
	cmd->cmd_dword[1] |= id;
}

static __inline void
cmd_format_pid(struct its_cmd *cmd, uint32_t pid)
{
	/* Physical ID field: DW1 [63:32] */
	cmd->cmd_dword[1] &= ~(0xFFFFFFFFUL << 32);
	cmd->cmd_dword[1] |= ((uint64_t)pid << 32);
}

static __inline void
cmd_format_col(struct its_cmd *cmd, uint16_t col_id)
{
	/* Collection field: DW2 [16:0] */
	cmd->cmd_dword[2] &= ~0xFFFFUL;
	cmd->cmd_dword[2] |= col_id;
}

static __inline void
cmd_format_target(struct its_cmd *cmd, uint64_t target)
{
	/* Target Address field: DW2 [47:16] */
	cmd->cmd_dword[2] &= ~(0xFFFFFFFFUL << 16);
	cmd->cmd_dword[2] |= (target & (0xFFFFFFFFUL << 16));
}

static __inline void
cmd_format_itt(struct its_cmd *cmd, uint64_t itt)
{
	/* ITT Address field: DW2 [47:8] */
	cmd->cmd_dword[2] &= ~0xFFFFFFFFFF00UL;
	cmd->cmd_dword[2] |= (itt & 0xFFFFFFFFFF00UL);
}

static __inline void
cmd_format_valid(struct its_cmd *cmd, uint8_t valid)
{
	/* Valid field: DW2 [63] */
	cmd->cmd_dword[2] &= ~(1UL << 63);
	cmd->cmd_dword[2] |= ((uint64_t)valid << 63);
}

static __inline void
cmd_fix_endian(struct its_cmd *cmd)
{
	size_t i;

	for (i = 0; i < nitems(cmd->cmd_dword); i++)
		cmd->cmd_dword[i] = htole64(cmd->cmd_dword[i]);
}

static void
its_cmd_mapc(struct gic_v3_its_softc *sc, struct its_col *col, uint8_t valid)
{
	struct its_cmd_desc desc;

	desc.cmd_type = ITS_CMD_MAPC;
	desc.cmd_desc_mapc.col = col;
	/*
	 * Valid bit set - map the collection.
	 * Valid bit cleared - unmap the collection.
	 */
	desc.cmd_desc_mapc.valid = valid;

	its_cmd_send(sc, &desc);
}

static void
its_cmd_mapvi(struct gic_v3_its_softc *sc, struct its_dev *its_dev,
    uint32_t id, uint32_t pid)
{
	struct its_cmd_desc desc;

	desc.cmd_type = ITS_CMD_MAPVI;
	desc.cmd_desc_mapvi.its_dev = its_dev;
	desc.cmd_desc_mapvi.id = id;
	desc.cmd_desc_mapvi.pid = pid;

	its_cmd_send(sc, &desc);
}

static void __unused
its_cmd_mapi(struct gic_v3_its_softc *sc, struct its_dev *its_dev,
    uint32_t lpinum)
{
	struct its_cmd_desc desc;

	desc.cmd_type = ITS_CMD_MAPI;
	desc.cmd_desc_mapi.its_dev = its_dev;
	desc.cmd_desc_mapi.lpinum = lpinum;

	its_cmd_send(sc, &desc);
}

static void
its_cmd_mapd(struct gic_v3_its_softc *sc, struct its_dev *its_dev,
    uint8_t valid)
{
	struct its_cmd_desc desc;

	desc.cmd_type = ITS_CMD_MAPD;
	desc.cmd_desc_mapd.its_dev = its_dev;
	desc.cmd_desc_mapd.valid = valid;

	its_cmd_send(sc, &desc);
}

static void
its_cmd_inv(struct gic_v3_its_softc *sc, struct its_dev *its_dev,
    uint32_t lpinum)
{
	struct its_cmd_desc desc;

	desc.cmd_type = ITS_CMD_INV;
	desc.cmd_desc_inv.lpinum = lpinum - its_dev->lpis.lpi_base;
	desc.cmd_desc_inv.its_dev = its_dev;

	its_cmd_send(sc, &desc);
}

static void
its_cmd_invall(struct gic_v3_its_softc *sc, struct its_col *col)
{
	struct its_cmd_desc desc;

	desc.cmd_type = ITS_CMD_INVALL;
	desc.cmd_desc_invall.col = col;

	its_cmd_send(sc, &desc);
}

/*
 * Helper routines for commands processing.
 */
static __inline boolean_t
its_cmd_queue_full(struct gic_v3_its_softc *sc)
{
	size_t read_idx, write_idx;

	write_idx = (size_t)(sc->its_cmdq_write - sc->its_cmdq_base);
	read_idx = gic_its_read(sc, 4, GITS_CREADR) / sizeof(struct its_cmd);

	/*
	 * The queue is full when the write offset points
	 * at the command before the current read offset.
	 */
	if (((write_idx + 1) % ITS_CMDQ_NENTRIES) == read_idx)
		return (TRUE);

	return (FALSE);
}

static __inline void
its_cmd_sync(struct gic_v3_its_softc *sc, struct its_cmd *cmd __unused)
{

	if (sc->its_flags & ITS_FLAGS_CMDQ_FLUSH) {
		/* XXX ARM64TODO: Flush dcache under cmd */
	} else {
		/*
		 * XXX: This is way to much.
		 * Should only be inner shareable, store.
		 */
		dsb();
	}

}

static struct its_cmd *
its_cmd_alloc(struct gic_v3_its_softc *sc)
{
	struct its_cmd *cmd;
	size_t us_left = 1000000;

	while (its_cmd_queue_full(sc)) {
		if (us_left-- == 0) {
			/* Timeout while waiting for free command */
			device_printf(sc->dev,
			    "Timeout while waiting for free command\n");
			return (NULL);
		}
		DELAY(1);
	}

	cmd = sc->its_cmdq_write;
	sc->its_cmdq_write++;

	if (sc->its_cmdq_write == (sc->its_cmdq_base + ITS_CMDQ_NENTRIES)) {
		/* Wrap the queue */
		sc->its_cmdq_write = sc->its_cmdq_base;
	}

	return (cmd);
}

static uint64_t
its_cmd_prepare(struct its_cmd *cmd, struct its_cmd_desc *desc)
{
	uint64_t target;
	uint8_t cmd_type;
	u_int size;
	boolean_t error;

	error = FALSE;
	cmd_type = desc->cmd_type;
	target = ITS_TARGET_NONE;

	switch (cmd_type) {
	case ITS_CMD_SYNC:	/* Wait for previous commands completion */
		target = desc->cmd_desc_sync.col->col_target;
		cmd_format_command(cmd, ITS_CMD_SYNC);
		cmd_format_target(cmd, target);
		break;
	case ITS_CMD_MAPD:	/* Assign ITT to device */
		target = desc->cmd_desc_mapd.its_dev->col->col_target;
		cmd_format_command(cmd, ITS_CMD_MAPD);
		cmd_format_itt(cmd, vtophys(desc->cmd_desc_mapd.its_dev->itt));
		/*
		 * Size describes number of bits to encode interrupt IDs
		 * supported by the device minus one.
		 * When V (valid) bit is zero, this field should be written
		 * as zero.
		 */
		if (desc->cmd_desc_mapd.valid) {
			size =
			    MAX(1, flsl(desc->cmd_desc_mapd.its_dev->lpis.lpi_num)) - 1;
		} else
			size = 0;

		cmd_format_size(cmd, size);
		cmd_format_devid(cmd, desc->cmd_desc_mapd.its_dev->devid);
		cmd_format_valid(cmd, desc->cmd_desc_mapd.valid);
		break;
	case ITS_CMD_MAPC:	/* Map collection to Re-Distributor */
		target = desc->cmd_desc_mapc.col->col_target;
		cmd_format_command(cmd, ITS_CMD_MAPC);
		cmd_format_col(cmd, desc->cmd_desc_mapc.col->col_id);
		cmd_format_valid(cmd, desc->cmd_desc_mapc.valid);
		cmd_format_target(cmd, target);
		break;
	case ITS_CMD_MAPVI:
		target = desc->cmd_desc_mapvi.its_dev->col->col_target;
		cmd_format_command(cmd, ITS_CMD_MAPVI);
		cmd_format_devid(cmd, desc->cmd_desc_mapvi.its_dev->devid);
		cmd_format_id(cmd, desc->cmd_desc_mapvi.id);
		cmd_format_pid(cmd, desc->cmd_desc_mapvi.pid);
		cmd_format_col(cmd, desc->cmd_desc_mapvi.its_dev->col->col_id);
		break;
	case ITS_CMD_MAPI:
		target = desc->cmd_desc_mapi.its_dev->col->col_target;
		cmd_format_command(cmd, ITS_CMD_MAPI);
		cmd_format_devid(cmd, desc->cmd_desc_mapi.its_dev->devid);
		cmd_format_id(cmd, desc->cmd_desc_mapi.lpinum);
		cmd_format_col(cmd, desc->cmd_desc_mapi.its_dev->col->col_id);
		break;
	case ITS_CMD_INV:
		target = desc->cmd_desc_inv.its_dev->col->col_target;
		cmd_format_command(cmd, ITS_CMD_INV);
		cmd_format_devid(cmd, desc->cmd_desc_inv.its_dev->devid);
		cmd_format_id(cmd, desc->cmd_desc_inv.lpinum);
		break;
	case ITS_CMD_INVALL:
		cmd_format_command(cmd, ITS_CMD_INVALL);
		cmd_format_col(cmd, desc->cmd_desc_invall.col->col_id);
		break;
	default:
		error = TRUE;
		break;
	}

	if (!error)
		cmd_fix_endian(cmd);

	return (target);
}

static __inline uint64_t
its_cmd_cwriter_offset(struct gic_v3_its_softc *sc, struct its_cmd *cmd)
{
	uint64_t off;

	off = (cmd - sc->its_cmdq_base) * sizeof(*cmd);

	return (off);
}

static void
its_cmd_wait_completion(struct gic_v3_its_softc *sc, struct its_cmd *cmd_first,
    struct its_cmd *cmd_last)
{
	uint64_t first, last, read;
	size_t us_left = 1000000;

	first = its_cmd_cwriter_offset(sc, cmd_first);
	last = its_cmd_cwriter_offset(sc, cmd_last);

	while (1) {
		read = gic_its_read(sc, 8, GITS_CREADR);
		if (read < first || read >= last)
			break;

		if (us_left-- == 0) {
			/* This means timeout */
			device_printf(sc->dev,
			    "Timeout while waiting for CMD completion.\n");
			return;
		}
		DELAY(1);
	}
}

static int
its_cmd_send(struct gic_v3_its_softc *sc, struct its_cmd_desc *desc)
{
	struct its_cmd *cmd, *cmd_sync;
	struct its_col col_sync;
	struct its_cmd_desc desc_sync;
	uint64_t target, cwriter;

	cmd = its_cmd_alloc(sc);
	if (!cmd) {
		device_printf(sc->dev, "no memory for cmd queue\n");
		return (EBUSY);
	}

	target = its_cmd_prepare(cmd, desc);
	its_cmd_sync(sc, cmd);

	if (target != ITS_TARGET_NONE) {
		cmd_sync = its_cmd_alloc(sc);
		if (!cmd_sync)
			goto end;
		desc_sync.cmd_type = ITS_CMD_SYNC;
		col_sync.col_target = target;
		desc_sync.cmd_desc_sync.col = &col_sync;
		its_cmd_prepare(cmd_sync, &desc_sync);
		its_cmd_sync(sc, cmd_sync);
	}
end:
	/* Update GITS_CWRITER */
	cwriter = its_cmd_cwriter_offset(sc, sc->its_cmdq_write);
	gic_its_write(sc, 8, GITS_CWRITER, cwriter);

	its_cmd_wait_completion(sc, cmd, sc->its_cmdq_write);

	return (0);
}

static struct its_dev *
its_device_find(struct gic_v3_its_softc *sc, device_t pci_dev)
{
	struct its_dev *its_dev;

	/* Find existing device if any */
	TAILQ_FOREACH(its_dev, &sc->its_dev_list, entry) {
		if (its_dev->pci_dev == pci_dev)
			return (its_dev);
	}

	return (NULL);
}

static struct its_dev *
its_device_alloc(struct gic_v3_its_softc *sc, device_t pci_dev)
{
	struct its_dev	*newdev;
	uint64_t typer;
	uint32_t devid;
	u_int nvecs;
	u_int cpuid;
	size_t esize;

	/* Find existing device if any */
	newdev = its_device_find(sc, pci_dev);
	if (newdev != NULL)
		return (newdev);

	devid = PCI_DEVID(pci_dev);
	nvecs = PCI_MSIX_NUM(pci_dev);

	/* There was no previously created device. Create one now */
	newdev = malloc(sizeof(*newdev), M_GIC_V3_ITS, M_WAITOK | M_ZERO);
	newdev->pci_dev = pci_dev;
	newdev->devid = devid;

	if (lpi_alloc_chunk(sc, &newdev->lpis, nvecs)) {
		free(newdev, M_GIC_V3_ITS);
		return (NULL);
	}

	/* Get ITT entry size */
	typer = gic_its_read(sc, 8, GITS_TYPER);
	esize = GITS_TYPER_ITTES(typer);
	/*
	 * Allocate ITT for this device.
	 * PA has to be 256 B aligned. At least two entries for device.
	 */
	newdev->itt = (vm_offset_t)contigmalloc(
	    roundup2(roundup2(nvecs, 2) * esize, 0x100), M_GIC_V3_ITS,
	    (M_WAITOK | M_ZERO), 0, ~0UL, 0x100, 0);

	/* XXX: Bind device interrupts to this CPU */
	cpuid = PCPU_GET(cpuid);
	newdev->col = &sc->its_cols[cpuid];

	TAILQ_INSERT_TAIL(&sc->its_dev_list, newdev, entry);

	/* Map device to its ITT */
	its_cmd_mapd(sc, newdev, 1);

	return (newdev);
}

static __inline void
its_device_asign_lpi(struct its_dev *its_dev, u_int *irq)
{

	KASSERT((its_dev->lpis.lpi_free > 0),
	    ("Cannot provide more LPIs for this device. LPI num: %u, free %u",
	    its_dev->lpis.lpi_num, its_dev->lpis.lpi_free));
	*irq = its_dev->lpis.lpi_base + (its_dev->lpis.lpi_num -
	    its_dev->lpis.lpi_free);
	its_dev->lpis.lpi_free--;
}
/*
 * Message signalled interrupts handling.
 */

/*
 * XXX ARM64TODO: Watch out for "irq" type.
 *
 * In theory GIC can handle up to (2^32 - 1) interrupt IDs whereas
 * we pass "irq" pointer of type integer. This is obviously wrong but
 * is determined by the way as PCI layer wants it to be done.
 * WARNING: devid must be PCI BUS+FUNCTION pair.
 */
int
gic_v3_its_alloc_msix(device_t dev, device_t pci_dev, int *irq)
{
	struct gic_v3_its_softc *sc;
	struct its_dev *its_dev;

	sc = device_get_softc(dev);
	/*
	 * TODO: Allocate device as seen by ITS if not already available.
	 *	 Notice that MSI-X interrupts are allocated on one-by-one basis.
	 */
	its_dev = its_device_alloc(sc, pci_dev);
	if (its_dev == NULL)
		return (ENOMEM);

	its_device_asign_lpi(its_dev, irq);

	return (0);
}

static void
lpi_map_to_device(struct gic_v3_its_softc *sc, struct its_dev *its_dev,
    uint32_t id, uint32_t pid)
{

	KASSERT((pid >= its_dev->lpis.lpi_base) &&
		(pid < (its_dev->lpis.lpi_base + its_dev->lpis.lpi_num)),
		("Trying to map ivalid LPI %u for this device\n", pid));

	its_cmd_mapvi(sc, its_dev, id, pid);
}

int
gic_v3_its_map_msix(device_t dev, device_t pci_dev, int irq, uint64_t *addr,
    uint32_t *data)
{
	struct gic_v3_its_softc *sc;
	bus_space_handle_t its_bsh;
	struct its_dev *its_dev;
	uint64_t its_pa;
	uint32_t id;

	sc = device_get_softc(dev);
	/* Verify that this device is allocated and owns this LPI */
	its_dev = its_device_find(sc, pci_dev);
	if (its_dev == NULL)
		return (EINVAL);

	id = irq - its_dev->lpis.lpi_base;
	lpi_map_to_device(sc, its_dev, id, irq);

	its_bsh = rman_get_bushandle(&sc->its_res[0]);
	its_pa = vtophys(its_bsh);

	*addr = (its_pa + GITS_TRANSLATER);
	*data = id;

	return (0);
}
