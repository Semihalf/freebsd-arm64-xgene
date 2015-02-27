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

#ifndef _GIC_V3_VAR_H_
#define _GIC_V3_VAR_H_

#define	GIC_V3_DEVSTR	"ARM Generic Interrupt Controller v3.0"

#define	LPI_FLAGS_CONF_FLUSH	(1UL << 0)
#define	LPI_CONFTAB_SIZE	PAGE_SIZE_64K
/* 1 bit per LPI + 1 KB more for the obligatory PPI, SGI, SPI stuff */
#define	LPI_PENDTAB_SIZE	((LPI_CONFTAB_SIZE / 8) + 0x400)

struct redist_lpis {
	vm_offset_t		conf_base;
	vm_offset_t		pend_base[MAXCPU];
	uint64_t		flags;
};

struct gic_redists {
	/*
	 * Re-Distributor region description.
	 * We will have few of those depending
	 * on the #redistributor-regions property in FDT.
	 */
	struct resource **	regions;
	/* Number of Re-Distributor regions */
	u_int			nregions;
	/* Per-CPU Re-Distributor handler */
	struct resource	*	pcpu[MAXCPU];
	/* LPIs data */
	struct redist_lpis	lpis;
};

struct gic_v3_softc {
	device_t		dev;
	struct resource	**	gic_res;
	struct mtx		gic_mtx;
	/* Distributor */
	struct resource *	gic_dist;
	/* Re-Distributors */
	struct gic_redists	gic_redists;

	u_int			gic_nirqs;
	u_int			gic_idbits;
};

extern devclass_t gic_v3_devclass;

MALLOC_DECLARE(M_GIC_V3);

/* Device and PIC methods */
int gic_v3_attach(device_t dev);
int gic_v3_detach(device_t dev);

void gic_v3_dispatch(device_t, struct trapframe *);
void gic_v3_eoi(device_t, u_int);
void gic_v3_mask_irq(device_t, u_int);
void gic_v3_unmask_irq(device_t, u_int);

/*
 * ITS
 */
#define	GIC_V3_ITS_DEVSTR	"ARM GIC Interrupt Translation Service"
#define	GIC_V3_ITS_COMPSTR	"arm,gic-v3-its"

/* LPI chunk owned by ITS device */
struct lpi_chunk {
	u_int	lpi_base;
	u_int	lpi_num;
	u_int	lpi_free;	/* First free LPI in set */
};

/* ITS device */
struct its_dev {
	TAILQ_ENTRY(its_dev)	entry;
	/* PCI device */
	device_t		pci_dev;
	/* Device ID (i.e. PCI device ID) */
	uint32_t		devid;
	/* List of assigned LPIs */
	struct lpi_chunk	lpis;
	/* Virtual address of ITT */
	vm_offset_t		itt;
	/* Interrupt collection */
	struct its_col *	col;
};
TAILQ_HEAD(its_dev_list, its_dev);

/* ITS private table description */
struct its_ptab {
	vm_offset_t	ptab_vaddr;	/* Virtual Address of table */
	size_t		ptab_pgsz;	/* Page size */
	size_t		ptab_npages;	/* Number of pages */
};

/* ITS collection description. */
struct its_col {
	uint64_t	col_target;	/* Target Re-Distributor */
	uint64_t	col_id;		/* Collection ID */
};

/* ITS command. Each command is 32 bytes long */
struct its_cmd {
	uint64_t	cmd_dword[4];	/* ITS command double word */
};

#define	ITS_CMD_SYNC	(0x05)
#define	ITS_CMD_MAPD	(0x08)
#define	ITS_CMD_MAPC	(0x09)
#define	ITS_CMD_MAPVI	(0x0a)
#define	ITS_CMD_MAPI	(0x0b)
#define	ITS_CMD_INV	(0x0c)
#define	ITS_CMD_INVALL	(0x0d)

/*
 * ITS command descriptor.
 * Idea for command description passing taken from Linux.
 */
struct its_cmd_desc {
	uint8_t cmd_type;

	union {
		struct {
			struct its_col *col;
		} cmd_desc_sync;

		struct {
			struct its_col *col;
			uint8_t valid;
		} cmd_desc_mapc;

		struct {
			struct its_dev *its_dev;
			uint32_t pid;
			uint32_t id;
		} cmd_desc_mapvi;

		struct {
			struct its_dev *its_dev;
			uint32_t lpinum;
		} cmd_desc_mapi;

		struct {
			struct its_dev *its_dev;
			uint8_t valid;
		} cmd_desc_mapd;

		struct {
			struct its_dev *its_dev;
			uint32_t lpinum;
		} cmd_desc_inv;

		struct {
			struct its_col *col;
		} cmd_desc_invall;
	};
};

#define	ITS_CMDQ_SIZE		PAGE_SIZE_64K
#define	ITS_CMDQ_NENTRIES	(ITS_CMDQ_SIZE / sizeof(struct its_cmd))

#define	ITS_FLAGS_CMDQ_FLUSH	(1UL << 0)

#define	ITS_TARGET_NONE		0xFBADBEEF

struct gic_v3_its_softc {
	device_t		dev;
	struct resource	*	its_res;

	struct its_cmd *	its_cmdq_base;	/* ITS command queue base */
	struct its_cmd *	its_cmdq_write;	/* ITS command queue write ptr */
	struct its_ptab		its_ptabs[GITS_BASER_NUM];/* ITS private tables */
	struct its_col *	its_cols;	/* Per-CPU collections */

	uint64_t		its_flags;

	struct its_dev_list	its_dev_list;

	unsigned long *		its_lpi_bitmap;
	uint32_t		its_lpi_maxid;
};

extern devclass_t gic_v3_its_devclass;

int gic_v3_its_attach(device_t);
int gic_v3_its_detach(device_t);

int gic_v3_its_alloc_msix(device_t, device_t, int *);
int gic_v3_its_map_msix(device_t, device_t, int, uint64_t *, uint32_t *);

void lpi_unmask_irq(device_t, uint32_t);
void lpi_mask_irq(device_t, uint32_t);
/*
 * GIC Distributor accessors.
 * Notice that only GIC sofc can be passed.
 */
#define	gic_d_read(sc, len, reg)		\
({						\
	bus_read_##len(sc->gic_dist, reg);	\
})

#define	gic_d_write(sc, len, reg, val)		\
({						\
	bus_write_##len(sc->gic_dist, reg, val);\
})

/* GIC Re-Distributor accessors (per-CPU) */
#define	gic_r_read(sc, len, reg)		\
({						\
	u_int cpu = PCPU_GET(cpuid);		\
						\
	bus_read_##len(				\
	    sc->gic_redists.pcpu[cpu],		\
	    reg);				\
})

#define	gic_r_write(sc, len, reg, val)		\
({						\
	u_int cpu = PCPU_GET(cpuid);		\
						\
	bus_write_##len(			\
	    sc->gic_redists.pcpu[cpu],		\
	    reg, val);				\
})

#define	PCI_DEVID(pci_dev)				\
({							\
	(((pci_get_domain(pci_dev) >> 2) << 19) |	\
	 ((pci_get_domain(pci_dev) % 4) << 16) |	\
	 (pci_get_bus(pci_dev) << 8) |			\
	 (pci_get_slot(pci_dev) << 3) |			\
	 (pci_get_function(pci_dev) << 0));		\
})

/*
 * Request number of maximum MSI-X vectors for this device.
 * Device can ask for less vectors than maximum supported but not more.
 */
#define	PCI_MSIX_NUM(pci_dev)			\
({						\
	struct pci_devinfo *dinfo;		\
	pcicfgregs *cfg;			\
						\
	dinfo = device_get_ivars(pci_dev);	\
	cfg = &dinfo->cfg;			\
						\
	cfg->msix.msix_msgnum;			\
})

#endif /* _GIC_V3_VAR_H_ */
