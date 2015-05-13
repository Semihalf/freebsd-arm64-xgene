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
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/endian.h>
#include <sys/kdb.h>

#include <machine/cpu.h>
#include <machine/bus.h>
#include <machine/fdt.h>
#include <machine/intr.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

/* #include "apm_pcie.h" */

#define VIRT_IRQ_OFFSET 1024

#define NR_MSI_REG		16
#define IRQS_PER_MSI_INDEX	32
#define IRQS_PER_MSI_REG	256
#define NR_MSI_IRQS		(NR_MSI_REG * IRQS_PER_MSI_REG)

/* PCIe MSI Index Registers */
#define MSI0IR0			0x000000
#define MSIFIR7			0x7F0000

/* PCIe MSI Interrupt Register */
#define MSI1INT0		0x800000
#define MSI1INTF		0x8F0000

struct apm_msix_softc {
	device_t	dev;
	struct resource *res;
	struct rman	rman;
	bus_addr_t	base_addr;
	uint32_t	irq_min;
	uint32_t	irq_max;
	uint32_t	irq_count;
	char	msi_bitmap[(NR_MSI_IRQS - 1)/sizeof(char) + 1];
	struct mtx	msi_mtx;
};

struct apm_irq_data {
	int offset;
};

static struct apm_irq_data* irq_data_tbl[NR_MSI_REG];
static u_int irq_data_tbl_ptr = 0;

struct apm_msix_softc* g_softc;

static int apm_msix_attach(device_t self);
static int apm_msix_probe(device_t self);
static int apm_msix_setup_hwirq(device_t,int,int);
static void apm_msix_remove(device_t);
int apm_msix_alloc(int,int*);
int apm_msix_map_msi(int,bus_addr_t*,uint32_t*);
int apm_msix_release(int,int*);
int apm_msix_setup_irq(device_t,device_t,struct resource*,int,driver_filter_t*,
		driver_intr_t*,void*,void**);
void apm_msix_handle_irq(void *);

static MALLOC_DEFINE(M_APM_MSIX, "msix", "APM MSI-X data structures");

#if 1
#define pr_debug(fmt,args...) do { printf("%s(): ", __func__); \
								   printf(fmt,##args); } while (0)
#define bootverbose 1
#else
#define pr_debug(fmt,args...)
#endif

static int
apm_msix_probe(device_t self)
{
	if (0 != ofw_bus_status_okay(self))
		return (ENXIO);

	if (0 != ofw_bus_is_compatible(self, "xgene,gic-msi"))
		return (ENXIO);

	device_set_desc(self, "APM MSI-X Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
apm_msix_attach(device_t self)
{
	int err;
	struct apm_msix_softc* sc;
	struct resource_list rl;
	struct resource_list_entry* rle;
	phandle_t node;
	bus_addr_t reg_base = 0, reg_size = 0;
	int interrupts[2], interrupts_count = 0, rid;
	u_int irq_index, j, offset, count;

	sc = device_get_softc(self);
	node = ofw_bus_get_node(self);
	sc->dev = self;

	if ((err = fdt_regsize(node, (u_long*)&reg_base, (u_long*)&reg_size))) {
		device_printf(self, "ERROR: Failed to resolve address space (err: %d)\n", err);
		return (ENXIO);
	}

	/* allocate resource */
	sc->res = bus_alloc_resource_any(self, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(self, "ERROR: Failed to allocate device resource (err=%d)\n", err);
		return (ENXIO);
	}

	/* initialize and manage memory region */
	sc->rman.rm_type = RMAN_ARRAY;
	sc->rman.rm_descr = "PCI MSI-X resource";
	err = rman_init(&sc->rman);
	if (err) {
		device_printf(self, "ERROR: rman_init() for %s failed (err: %d)\n",
				sc->rman.rm_descr, err);
		return (err);
	}

	err = rman_manage_region(&sc->rman, reg_base, reg_base + reg_size - 1);
	if (err) {
		device_printf(self, "ERROR: rman_manage_region() for %s failed (err: %d)\n",
				sc->rman.rm_descr, err);
		rman_fini(&sc->rman);
		return (err);
	}
	/* store base addr */
	sc->base_addr = reg_base;

	/* set structures' memory */
	memset(irq_data_tbl, 0, sizeof(irq_data_tbl));
	memset(interrupts, 0, sizeof(interrupts));
	memset(&rl, 0, sizeof(rl));

	/* get resource list from FDT/OFW */
	resource_list_init(&rl);
	if ((err = ofw_bus_intr_to_rl(self, node, &rl))) {
		device_printf(self, "ERROR: Failed to get interrupt data (err: %d)\n", err);
		return (ENXIO);
	}

	STAILQ_FOREACH(rle, &rl, link) {
		if (interrupts_count >= sizeof(interrupts)/sizeof(interrupts[0])) {
			device_printf(self, "ERROR: Unable to parse IRQ assignment\n");
			/* free resource list */
			resource_list_free(&rl);
			return (ENOMEM);
		}
		interrupts[interrupts_count] = rle->start;
		interrupts_count++;
	}
	resource_list_free(&rl);

	sc->irq_min = interrupts[0];
	sc->irq_max = interrupts[1];
	sc->irq_count = sc->irq_max - sc->irq_min + 1;

	offset = interrupts[0] / IRQS_PER_MSI_REG;
	count = interrupts[1] / IRQS_PER_MSI_REG;

	/* set global software context */
	g_softc = sc;

	/* initialize mutex */
	mtx_init(&sc->msi_mtx, "msi_mtx", NULL, MTX_DEF);

	/* set up hardware IRQ handling */
	for (irq_index = 0, j = 0; j < count; j++, irq_index++) {
		err = apm_msix_setup_hwirq(self, offset + j, irq_index);
		if (err)
			goto error;
	}
	device_printf(self, "MSI-X: addr 0x%lx, size 0x%lx, IRQ %d-%d\n",
			sc->base_addr, reg_size, sc->irq_min, sc->irq_max);

	return (bus_generic_attach(self));

error:
	apm_msix_remove(self);
	return (err);
}

int
apm_msix_alloc(int count, int *irq)
{
	u_int start, i, irq_min;

	/* count must be a power of 2 */
	if (powerof2(count) == 0 || count > 8) {
		return (EINVAL);
	}

	irq_min = g_softc->irq_min;
	mtx_lock(&g_softc->msi_mtx);

	/* find free space in bitmap */
	for (start = 0; (start + count) < g_softc->irq_count; start++) {
		for (i = start; i < start + count; i++)
			if (isset(&g_softc->msi_bitmap, i))
				break;
		if (i == start + count)
			break;
	}

	if ((start + count) >= g_softc->irq_max) {
		mtx_unlock(&g_softc->msi_mtx);
		return (ENXIO);
	}

	/* allocate IRQ in bitmap */
	for (i = start; i < start + count; i++) {
		setbit(&g_softc->msi_bitmap, i);
		*irq++ = irq_min + i;
	}
	mtx_unlock(&g_softc->msi_mtx);

	device_printf(g_softc->dev, "MSI-X: allocated IRQ %d-%d\n",
			irq_min + start,
			irq_min + start + count-1);

	return (0);
}

int
apm_msix_map_msi(int irq, bus_addr_t* addr, uint32_t* data)
{
	device_t dev;
	u_int virt_irq;

	dev = g_softc->dev;

	/* validate IRQ number */
	if ((irq > g_softc->irq_max) || (irq < g_softc->irq_min) ||
			isclr(&g_softc->msi_bitmap, irq - g_softc->irq_min)) {
		device_printf(dev, "ERROR: invalid IRQ %d\n", irq);
		return (EINVAL);
	}

	*addr = g_softc->base_addr + irq;
	*data = 0;

	virt_irq = VIRT_IRQ_OFFSET + irq;
	if (arm_config_intr(virt_irq, INTR_TRIGGER_EDGE, INTR_POLARITY_HIGH)) {
		device_printf(dev, "ERROR: unable to config interrupt\n");
		return (EINVAL);
	}

	device_printf(dev, "MSI mapping: IRQ %d, addr %#lx, data %#x\n",
			irq, *addr, *data);

	return (0);
}

int
apm_msix_release(int count, int *irq)
{
	int i;

	mtx_lock(&g_softc->msi_mtx);

	for (i = 0; i < count; i++)
		clrbit(&g_softc->msi_bitmap, irq[i] - g_softc->irq_min);

	mtx_unlock(&g_softc->msi_mtx);

	return (0);
}

static inline uint32_t apm_msi_intr_read(bus_addr_t base,
				      u_int reg)
{
	uint32_t irq_reg = MSI1INT0 + (reg << 16);
	pr_debug("base = %lu, irq_reg = 0x%x, reg = 0x%x\n", base, irq_reg, reg);
	return (bus_read_4(g_softc->res, irq_reg));
}

static inline uint32_t apm_msir_read(bus_addr_t base, u_int group,
		u_int reg)
{
	uint32_t irq_reg = MSI0IR0 + (group << 19) + (reg << 16);
	pr_debug("base = %lu irq_reg = 0x%x, group = 0x%x, reg = 0x%x\n",
		base, irq_reg, group, reg);
	return (bus_read_4(g_softc->res, irq_reg));
}

static void
apm_msix_dispatch_virt_irq(u_int virt_irq)
{
	u_int irq;
	struct trapframe frame;

	/* Check if this IRQ is mapped */
	mtx_lock(&g_softc->msi_mtx);
	if (!isset(&g_softc->msi_bitmap, virt_irq)) {
		mtx_unlock(&g_softc->msi_mtx);
		device_printf(g_softc->dev, "Invalid IRQ%d", virt_irq);
		return;
	}
	mtx_unlock(&g_softc->msi_mtx);

	irq = VIRT_IRQ_OFFSET + virt_irq;
	arm_dispatch_intr(irq, &frame);
}

void
apm_msix_handle_irq(void *arg)
{
	struct apm_irq_data *irq_data;
	int msir_index = -1;
	bus_addr_t msi_intr_reg;
	uint32_t msir_value, intr_index, msi_intr_reg_value;
	u_int virt_irq;

	if (arg == NULL) {
		panic("Interrupt controller error\n");
	}
	irq_data = (struct apm_irq_data*)arg;

	/* get MSI-X IRQ register */
	msi_intr_reg = irq_data->offset;
	/* read value from MSI-X IRQ register */
	msi_intr_reg_value = apm_msi_intr_read(g_softc->base_addr, msi_intr_reg);

	pr_debug("MSI-X: read value %#x from register %#lx\n", msi_intr_reg_value, msi_intr_reg);

	/* handle all set interrupts */
	while (msi_intr_reg_value != 0) {
		/*  */
		msir_index = ffs(msi_intr_reg_value) - 1;

		msir_value = apm_msir_read(g_softc->base_addr, msi_intr_reg, msir_index);

		while (msir_value != 0) {
			/* proceed with first set bit */
			intr_index = ffs(msir_value) - 1;
			/* calculate 'virtual' interrupt */
			virt_irq = msir_index * IRQS_PER_MSI_INDEX * NR_MSI_REG +
				intr_index * NR_MSI_REG + msi_intr_reg;
			/* dispatch interrupt to proper device */
			if (virt_irq != 0)
				apm_msix_dispatch_virt_irq(virt_irq);
			/* reset current IRQ bit */
			msir_value &= ~(1 << intr_index);
		}
		/* reset current MSI-X register bit*/
		msi_intr_reg_value &= ~(1 << msir_index);
	}
	return;
}

static int
apm_msix_setup_hwirq(device_t dev, int offset, int irq)
{
	int flags = INTR_TYPE_AV, err;
	struct apm_irq_data *irq_data = NULL;

	if (irq < 0) {
		device_printf(dev, "ERROR: Cannot translate IRQ index %d\n", irq);
		return (EINVAL);
	}

	/* allocate memory for IRQ data and store it for further release */
	if (irq_data_tbl_ptr < NR_MSI_REG) {
		irq_data_tbl[irq_data_tbl_ptr] =
				malloc(sizeof (struct apm_irq_data), M_APM_MSIX, M_NOWAIT | M_ZERO);
		irq_data = irq_data_tbl[irq_data_tbl_ptr++];
	}
	if (irq_data == NULL) {
		device_printf(dev, "ERROR: No memory for MSI-X interrupt data structure\n");
		return (ENOMEM);
	}

	/* store this data for later usage */
	irq_data->offset = offset;

	/* setup HW interrupt with own handler */
	err = arm_setup_intr(device_get_nameunit(dev), NULL, apm_msix_handle_irq, (void*)irq_data, irq, flags, NULL);
	if (err)
		device_printf(dev, "ERROR: IRQ%d setup failed (err=%d)\n", irq, err);
	else
		device_printf(dev, "MSI-X: hw setup of IRQ%d\n", irq);

	/* config HW IRQ to be triggered by edge */
	err = arm_config_intr(irq, INTR_TRIGGER_EDGE, INTR_POLARITY_HIGH);
	if (err)
		device_printf(dev, "ERROR: unable to configure IRQ%d (err=%d)\n", irq, err);

	return (err);
}

int
apm_msix_setup_irq(device_t dev, device_t child, struct resource *res, int flags,
	    driver_filter_t *filt, driver_intr_t *intr, void *arg, void **cookiep)
{
	int virt_irq, error;

	if ((rman_get_flags(res) & RF_SHAREABLE) == 0)
		flags |= INTR_EXCL;

	/* We depend here on rman_activate_resource() being idempotent. */
	error = rman_activate_resource(res);
	if (error)
		return (error);

	/* This IRQ will be considered as 'virtual' */
	virt_irq = VIRT_IRQ_OFFSET + rman_get_start(res);

	error = arm_setup_intr(device_get_nameunit(child), filt, intr,
		arg, virt_irq, flags, cookiep);

	return (error);
}

static void
apm_msix_remove(device_t dev)
{
	int i;

	for (i = 0; i < NR_MSI_REG; i++) {
		if (irq_data_tbl[i] != NULL)
			free(irq_data_tbl[i], M_APM_MSIX);
	}
}
