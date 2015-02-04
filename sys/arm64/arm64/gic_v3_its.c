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
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/ktr.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/pcpu.h>
#include <sys/proc.h>
#include <sys/cpuset.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>

#include "gic_v3_reg.h"
#include "gic_v3_var.h"

MALLOC_DEFINE(M_GIC_V3_ITS, "GICv3 ITS", GIC_V3_ITS_DEVSTR);

devclass_t gic_v3_its_devclass;

static int its_alloc_tables(struct gic_v3_its_softc *);
static void its_free_tables(struct gic_v3_its_softc *);
static int its_init_commandq(struct gic_v3_its_softc *);

static int lpi_init_conftable(struct gic_v3_its_softc *);

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

	/* 5. Allocate LPI table */
	ret = lpi_init_conftable(sc);
	if (ret)
		goto error;

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
	int rid = 0;

	sc = device_get_softc(dev);

	/* Release what's possible */

	/* Command queue */
	if ((void *)sc->its_cmdq != NULL)
		contigfree((void *)sc->its_cmdq, ITS_CMDQ_SIZE, M_GIC_V3_ITS);
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

	/* Resource... */
	bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->its_res);

	return (0);
}

/* XXX: Currently assume that page size is 4 KB */
CTASSERT(PAGE_SIZE == (1 << 12));

static int
its_alloc_tables(struct gic_v3_its_softc *sc)
{
	uint64_t gits_baser, gits_tmp;
	uint64_t type, esize, cache, share, psz;
	uint32_t gits_typer;
	size_t page_size, npages, nitspages, nidents, tn;
	size_t its_tbl_size;
	vm_offset_t ptab_vaddr;
	vm_paddr_t ptab_paddr;
	boolean_t first = TRUE;

	page_size = PAGE_SIZE_64K;

	/* Read features first */
	gits_typer = gic_its_read(sc, 4, GITS_TYPER);

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
			nidents = (1 << GITS_TYPER_DEVB(gits_typer)) - 1;
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

		/* XXX: Keep it non-cacheable and non-bufferable for now. */
		cache = GITS_BASER_CACHE_NCNB;
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
			    ptab_paddr | (nitspages - 1);

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

		if (gits_tmp != gits_baser) {
			device_printf(sc->dev,
			    "Could not allocate ITS tables\n");
			its_free_tables(sc);
			return (ENXIO);
		}

		/* Works fine so set the valid bit */
		gits_baser |= GITS_BASER_VALID;
		gic_its_write(sc, 8, GITS_BASER(tn), gits_baser);

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
	vm_paddr_t cmdq_paddr;
	device_t dev;

	dev = sc->dev;
	/* Allocate memory for command queue */
	sc->its_cmdq = contigmalloc(ITS_CMDQ_SIZE, M_GIC_V3_ITS, M_WAITOK,
	    0, ~0UL, ITS_CMDQ_SIZE, 0);

	/* Set command queue write pointer and attributes */
	cmdq_paddr = vtophys(sc->its_cmdq);
	KASSERT((cmdq_paddr & GITS_CBASER_PA_MASK) == cmdq_paddr,
	    ("%s: Unaligned PA for ITS Commands Queue", device_get_name(dev)));

	gits_cbaser = (cmdq_paddr |
	    /* XXX: Leave it uncacheable for now */
	    (GITS_CBASER_CACHE_NIN << GITS_CBASER_CACHE_SHIFT) |
	    /* Inner shareable */
	    (GITS_CBASER_SHARE_IS << GITS_CBASER_SHARE_SHIFT) |
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
lpi_init_conftable(struct gic_v3_its_softc *sc)
{
	device_t parent;
	struct gic_v3_softc *gic_sc;
	vm_offset_t conf_base;
	uint8_t prio_default;

	parent = device_get_parent(sc->dev);
	gic_sc = device_get_softc(parent);
	KASSERT(gic_sc != NULL,
	    ("%s: Cannot get GIC software context\n", __func__));

	gic_sc->gic_redists.lpis.conf_base =
	    (vm_offset_t)contigmalloc(LPI_CONFTAB_SIZE, M_GIC_V3_ITS,
		M_WAITOK, 0, ~0UL, PAGE_SIZE_64K, 0);

	conf_base = gic_sc->gic_redists.lpis.conf_base;

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
	 * Let the default priority be aligned with all other interrupts
	 * assuming that each interrupt is assigned MAX priority at startup.
	 * MAX priority on the other hand cannot be higher than 0xFC for LPIs.
	 */
	prio_default = GIC_PRIORITY_MAX;

	/* Write each settings byte to LPI configuration table */
	memset((void *)conf_base,
	    (prio_default & LPI_CONF_PRIO_MASK) | LPI_CONF_GROUP1,
	    LPI_CONFTAB_SIZE);

	/* XXX ARM64TODO: Clean dcache under table here */

	return (0);
}
