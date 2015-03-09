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
#include <machine/intr.h>

#include "pic_if.h"

#include "gic_v3_reg.h"
#include "gic_v3_var.h"

/*
 * Driver-specific definitions.
 */
MALLOC_DEFINE(M_GIC_V3, "GICv3", GIC_V3_DEVSTR);

devclass_t gic_v3_devclass;

/*
 * Helper functions and definitions.
 */
/* Destination registers, either Distributor or Re-Distributor */
enum gic_v3_xdist {
	DIST = 0,
	REDIST,
};

/* Helper routines starting with gic_v3_ */
static int gic_v3_dist_init(struct gic_v3_softc *);
static int gic_v3_redist_find(struct gic_v3_softc *);
static int gic_v3_redist_init(struct gic_v3_softc *);
static int gic_v3_cpu_init(struct gic_v3_softc *);
static void gic_v3_wait_for_rwp(struct gic_v3_softc *, enum gic_v3_xdist);

/* A sequence of init functions for primary (boot) and secondary CPUs */
typedef int (*gic_v3_initseq_t) (struct gic_v3_softc *);
/* Primary CPU initialization sequence */
static gic_v3_initseq_t gic_v3_primary_init[] = {
	gic_v3_dist_init,
	gic_v3_redist_init,
	gic_v3_cpu_init,
	NULL
};
/* Secondary CPU initialization sequence */
static gic_v3_initseq_t gic_v3_secondary_init[] __unused = {
	NULL
};

/*
 * Device interface.
 */
int
gic_v3_attach(device_t dev)
{
	struct gic_v3_softc *sc;
	gic_v3_initseq_t *init_func;
	uint32_t typer;
	int rid;
	int err;
	size_t i;

	sc = device_get_softc(dev);
	sc->dev = dev;
	err = 0;

	/* Initialize mutex */
	mtx_init(&sc->gic_mtx, "GICv3 lock", NULL, MTX_SPIN);

	/*
	 * Allocate array of struct resource.
	 * One entry for Distributor and all remaining for Re-Distributor.
	 */
	sc->gic_res = malloc(
	    sizeof(sc->gic_res) * (sc->gic_redists.nregions + 1),
	    M_GIC_V3, M_WAITOK);

	/* Now allocate corresponding resources */
	for (i = 0, rid = 0; i < (sc->gic_redists.nregions + 1); i++, rid++) {
		sc->gic_res[rid] = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
		    &rid, RF_ACTIVE);
		if (sc->gic_res[rid] == NULL) {
			err = ENXIO;
			goto error;
		}
	}

	/*
	 * Distributor interface
	 */
	sc->gic_dist = sc->gic_res[0];

	/*
	 * Re-Dristributor interface
	 */
	/* Allocate space under region descriptions */
	sc->gic_redists.regions = malloc(
	    sizeof(*sc->gic_redists.regions) * sc->gic_redists.nregions,
	    M_GIC_V3, M_WAITOK);

	/* Fill-up bus_space information for each region. */
	for (i = 0, rid = 1; i < sc->gic_redists.nregions; i++, rid++)
		sc->gic_redists.regions[i] = sc->gic_res[rid];

	/* Get the number of supported SPI interrupts */
	typer = gic_d_read(sc, 4, GICD_TYPER);
	sc->gic_nirqs = typer & 0x1F;
	sc->gic_nirqs = (sc->gic_nirqs + 1) * 32;
	if (sc->gic_nirqs > 1020)
		sc->gic_nirqs = 1020;

	/* Get the number of supported interrupt identifier bits */
	sc->gic_idbits = GICD_TYPER_IDBITS(typer);

	if (bootverbose) {
		device_printf(dev, "SPIs: %u, IDs: %u\n",
		    sc->gic_nirqs, (1 << sc->gic_idbits) - 1);
	}

	/* Train init sequence for boot CPU */
	for (init_func = gic_v3_primary_init; *init_func != NULL; init_func++) {
		err = (*init_func)(sc);
		if (err)
			goto error;
	}
	/*
	 * Full success.
	 * Now register PIC to the interrupts handling layer.
	 */
	arm_register_root_pic(dev, sc->gic_nirqs);

error:
	return (err);
}

int
gic_v3_detach(device_t dev)
{
	struct gic_v3_softc *sc;
	size_t i;
	int rid;

	sc = device_get_softc(dev);

	if (device_is_attached(dev)) {
		/*
		 * XXX: We should probably deregister PIC
		 */
	}
	for (i = 0, rid = 0; i < (sc->gic_redists.nregions + 1); i++, rid++)
		bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->gic_res[rid]);

	for (i = 0; i < MAXCPU; i++)
		free(sc->gic_redists.pcpu[i], M_GIC_V3);

	free(sc->gic_res, M_GIC_V3);
	free(sc->gic_redists.regions, M_GIC_V3);

	return (0);
}

/*
 * PIC interface.
 */
void
gic_v3_dispatch(device_t dev, struct trapframe *frame)
{
	uint64_t active_irq;

	while (1) {
		active_irq = gic_icc_read(IAR1);

		if (__predict_false(active_irq == ICC_IAR1_EL1_SPUR))
			break;

		if (__predict_true((active_irq >= GIC_FIRST_PPI &&
		    active_irq <= GIC_LAST_SPI) || active_irq >= GIC_FIRST_LPI)) {
			arm_dispatch_intr(active_irq, frame);
			continue;
		}

		if (active_irq <= GIC_LAST_SGI) {
			/*
			 * TODO: Implement proper SGI handling.
			 *	Mask it if such is received for some reason.
			 */
			device_printf(dev, "SGI received on UP system\n");
			PIC_MASK(dev, active_irq);
		}
	}
}

void
gic_v3_eoi(device_t dev, u_int irq)
{

	gic_icc_write(EOIR1, (uint64_t)irq);
}

void
gic_v3_mask_irq(device_t dev, u_int irq)
{
	struct gic_v3_softc *sc;
	uint32_t mask;

	sc = device_get_softc(dev);

	mask = (1 << (irq % 32));

	if (irq < GIC_FIRST_SPI) { /* SGIs and PPIs in corresponding Re-Distributor */
		gic_r_write(sc, 4,
		    PAGE_SIZE_64K + GICD_ICENABLER(irq >> 5), mask);
		gic_v3_wait_for_rwp(sc, REDIST);
	} else if (irq >= GIC_FIRST_SPI && irq < GIC_LAST_SPI) { /* SPIs in distributor */
		gic_r_write(sc, 4, GICD_ICENABLER(irq >> 5), mask);
		gic_v3_wait_for_rwp(sc, DIST);
	} else if (irq >= GIC_FIRST_LPI) { /* LPIs */
		lpi_mask_irq(dev, irq);
	} else {
		KASSERT(0, ("%s: Unsupported IRQ number %u", __func__, irq));
	}

}

void
gic_v3_unmask_irq(device_t dev, u_int irq)
{
	struct gic_v3_softc *sc;
	uint32_t mask;

	sc = device_get_softc(dev);

	mask = (1 << (irq % 32));

	if (irq < GIC_FIRST_SPI) { /* SGIs and PPIs in corresponding Re-Distributor */
		gic_r_write(sc, 4,
		    PAGE_SIZE_64K + GICD_ISENABLER(irq >> 5), mask);
		gic_v3_wait_for_rwp(sc, REDIST);
	} else if (irq >= GIC_FIRST_SPI && irq < GIC_LAST_SPI) { /* SPIs in distributor */
		gic_d_write(sc, 4, GICD_ISENABLER(irq >> 5), mask);
		gic_v3_wait_for_rwp(sc, DIST);
	} else if (irq >= GIC_FIRST_LPI) { /* LPIs */
		lpi_unmask_irq(dev, irq);
	} else {
		KASSERT(0, ("%s: Unsupported IRQ number %u", __func__, irq));
	}
}

/*
 * Helper routines
 */
static void
gic_v3_wait_for_rwp(struct gic_v3_softc *sc, enum gic_v3_xdist xdist)
{
	struct resource *res;
	u_int cpuid;
	size_t us_left = 1000000;

	cpuid = PCPU_GET(cpuid);

	switch (xdist) {
	case (DIST):
		res = sc->gic_dist;
		break;
	case (REDIST):
		res = sc->gic_redists.pcpu[cpuid];
		break;
	default:
		KASSERT(0, ("%s: Attempt to wait for unknown RWP", __func__));
		return;
	}

	while ((bus_read_4(res, GICD_CTLR) & GICD_CTLR_RWP) != 0) {
		DELAY(1);
		if (us_left-- == 0) {
			device_printf(sc->dev,
			    "GICD Register write pending for too long");
			return;
		}
	}
}

/* CPU interface. */
static __inline void
gic_v3_cpu_priority(uint64_t mask)
{

	/* Set prority mask */
	gic_icc_write(PMR, mask & 0xFFUL);
}

static int
gic_v3_cpu_enable_sre(struct gic_v3_softc *sc)
{
	uint64_t sre;
	u_int cpuid;

	cpuid = PCPU_GET(cpuid);
	/*
	 * Set the SRE bit to enable access to GIC CPU interface
	 * via system registers.
	 */
	__asm __volatile(
	    "mrs	%0, icc_sre_el1	\n"
	    "orr	%0, %0, %1	\n"
	    "msr	icc_sre_el1, %0	\n"
	    "isb			\n"
	    : "=r" (sre) : "L" (ICC_SRE_EL1_SRE));

	/*
	 * Now ensure that the bit is set.
	 */
	__asm __volatile("mrs	%0, icc_sre_el1" : "=r" (sre));
	if (!(sre & ICC_SRE_EL1_SRE)) {
		/* We are done. This was disabled in EL2 */
		device_printf(sc->dev, "ERROR: CPU%u cannot enable CPU interface "
		    "via system registers\n", cpuid);
		return (ENXIO);
	} else if (bootverbose) {
		device_printf(sc->dev,
		    "CPU%u enabled CPU interface via system registers\n",
		    cpuid);
	}

	return (0);
}

static int
gic_v3_cpu_init(struct gic_v3_softc *sc)
{
	int err;

	/* Enable access to CPU interface via system registers */
	err = gic_v3_cpu_enable_sre(sc);
	if (err)
		goto error;
	/* Priority mask to minimum - accept all interrupts */
	gic_v3_cpu_priority(GIC_PRIORITY_MIN);
	/* Disable EOI mode */
	gic_icc_clear(CTLR, ICC_CTLR_EL1_EOI);
	/* Enable group 1 (insecure) interrups */
	gic_icc_set(IGRPEN1, ICC_IGRPEN0_EL1_EN);

error:
	return (err);
}

/* Distributor */
static int
gic_v3_dist_init(struct gic_v3_softc *sc)
{
	uint64_t aff;
	u_int i;

	/*
	 * 1. Disable the Distributor
	 */
	gic_d_write(sc, 4, GICD_CTLR, 0);
	gic_v3_wait_for_rwp(sc, DIST);

	/*
	 * 2. Configure the Distributor
	 */
	/* Set all global interrupts to be level triggered, active low. */
	for (i = 32; i < sc->gic_nirqs; i += 16) {
		gic_d_write(sc, 4, GICD_ICFGR(i >> 4), 0x00000000);
	}
	/* Set priority to all shared interrupts */
	for (i = 32; i < sc->gic_nirqs; i += 4) {
		/* Set highest priority */
		gic_d_write(sc, 4, GICD_IPRIORITYR(i >> 2), GIC_PRIORITY_MAX);
	}

	/*
	 * Disable all interrupts. Leave PPI and SGIs as they are enabled in
	 * Re-Distributor registers.
	 */
	for (i = 32; i < sc->gic_nirqs; i += 32) {
		gic_d_write(sc, 4, GICD_ICENABLER(i >> 5), 0xFFFFFFFF);
	}

	gic_v3_wait_for_rwp(sc, DIST);

	/*
	 * 3. Enable Distributor
	 */
	/* Enable Distributor with ARE, Group 1 */
	gic_d_write(sc, 4, GICD_CTLR, GICD_CTLR_ARE_NS | GICD_CTLR_G1A |
	    GICD_CTLR_G1);

	/*
	 * 4. Route all interrupts to boot CPU.
	 */
	aff = CPU_AFFINITY(PCPU_GET(cpuid));
	for (i = 32; i < sc->gic_nirqs; i++)
		gic_d_write(sc, 4, GICD_IROUTER(i), aff);

	return (0);
}

/* Re-Distributor */
static int
gic_v3_redist_find(struct gic_v3_softc *sc)
{
	struct resource r_res;
	bus_space_handle_t r_bsh;
	uint64_t aff;
	uint64_t typer;
	uint32_t pidr2;
	u_int cpuid;
	size_t i;

	cpuid = PCPU_GET(cpuid);

	/* Allocate struct resource for this CPU's Re-Distributor registers */
	sc->gic_redists.pcpu[cpuid] =
	    malloc(sizeof(*sc->gic_redists.pcpu), M_GIC_V3, M_WAITOK);
	if (sc->gic_redists.pcpu[cpuid] == NULL) {
		if (bootverbose) {
			device_printf(sc->dev,
			    "Could not allocate memory for CPU%u "
			    "Re-Distributor resource description\n", cpuid);
		}
		return (ENOMEM);
	}

	aff = CPU_AFFINITY(cpuid);
	/* Affinity in format for comparison with typer */
	aff = (CPU_AFF3(aff) << 24) | (CPU_AFF2(aff) << 16) |
	    (CPU_AFF1(aff) << 8) | CPU_AFF0(aff);

	if (bootverbose) {
		device_printf(sc->dev,
		    "Start searching for Re-Distributor\n");
	}
	/* Iterate through Re-Distributor regions */
	for (i = 0; i < sc->gic_redists.nregions; i++) {
		/* Take a copy of the region's resource */
		r_res = *sc->gic_redists.regions[i];
		r_bsh = rman_get_bushandle(&r_res);

		pidr2 = bus_read_4(&r_res, GICR_PIDR2);
		switch (pidr2 & GICR_PIDR2_ARCH_MASK) {
		case GICR_PIDR2_ARCH_GICv3: /* fall through */
		case GICR_PIDR2_ARCH_GICv4:
			break;
		default:
			device_printf(sc->dev,
			    "No Re-Distributor found for CPU%u\n", cpuid);
			free(sc->gic_redists.pcpu[cpuid], M_GIC_V3);
			return (ENODEV);
		}

		do {
			typer = bus_read_8(&r_res, GICR_TYPER);
			if ((typer >> 32) == aff) {
				KASSERT(sc->gic_redists.pcpu[cpuid] != NULL,
				    ("Invalid pointer to per-CPU redistributor"));
				/* Copy res contents to its final destination */
				*sc->gic_redists.pcpu[cpuid] = r_res;
				if (bootverbose) {
					device_printf(sc->dev,
					    "CPU%u Re-Distributor has been found\n",
					    cpuid);
				}
				return (0);
			}

			r_bsh += PAGE_SIZE_64K * 2;
			if (typer & GICR_TYPER_VLPIS)
				r_bsh += PAGE_SIZE_64K * 2;

			rman_set_bushandle(&r_res, r_bsh);
		} while (!(typer & GICR_TYPER_LAST));
	}

	free(sc->gic_redists.pcpu[cpuid], M_GIC_V3);
	device_printf(sc->dev, "No Re-Distributor found for CPU%u\n", cpuid);
	return (ENXIO);
}

static int
gic_v3_redist_wake(struct gic_v3_softc *sc)
{
	uint32_t waker;
	size_t us_left = 1000000;

	waker = gic_r_read(sc, 4, GICR_WAKER);
	/* Wake up Re-Distributor for this CPU */
	waker &= ~GICR_WAKER_PS;
	gic_r_write(sc, 4, GICR_WAKER, waker);
	/*
	 * When clearing ProcessorSleep bit it is required to wait for
	 * ChildrenAsleep to become zero following the processor power-on.
	 */
	while ((gic_r_read(sc, 4, GICR_WAKER) & GICR_WAKER_CA) != 0) {
		DELAY(1);
		if (us_left-- == 0) {
			device_printf(sc->dev,
			    "Could not wake Re-Distributor for CPU%u",
			    PCPU_GET(cpuid));
			return (ENXIO);
		}
	}

	if (bootverbose) {
		device_printf(sc->dev, "CPU%u Re-Distributor woke up\n",
		    PCPU_GET(cpuid));
	}

	return (0);
}

static int
gic_v3_redist_init(struct gic_v3_softc *sc)
{
	int err;
	size_t i;

	err = gic_v3_redist_find(sc);
	if (err)
		goto error;

	err = gic_v3_redist_wake(sc);
	if (err)
		goto error;

	/* Disable SPIs */
	gic_r_write(sc, 4, PAGE_SIZE_64K + GICR_ICENABLER0, 0xFFFF0000);
	/* Enable SGIs */
	gic_r_write(sc, 4, PAGE_SIZE_64K + GICR_ISENABLER0, 0x0000FFFF);

	/* Set priority for SGIs and PPIs */
	for (i = 0; i < 32; i += 4) {
		gic_r_write(sc, 4, PAGE_SIZE_64K + GICD_IPRIORITYR(i >> 2),
		    GIC_PRIORITY_MAX);
	}

	gic_v3_wait_for_rwp(sc, REDIST);

error:
	return (err);
}
