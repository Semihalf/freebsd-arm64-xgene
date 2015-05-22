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

#define MSI_IRQ_BASE	0x7A000000
/* PCIe MSI Index Registers */
#define MSI0IR0			0x000000
#define MSIFIR7			0x7F0000

/* PCIe MSI Interrupt Register */
#define MSI1INT0		0x800000
#define MSI1INTF		0x8F0000

struct apm_msix_softc {
	device_t	dev;
	struct resource *res;
	struct resource *irq_res;
	struct rman	rman;
	struct rman irq_rman;
	bus_addr_t	base_addr;
	int	irq_min;
	int	irq_max;
	int	irq_count;
	char	msi_bitmap[(NR_MSI_IRQS - 1)/sizeof(char) + 1];
	struct mtx	msi_mtx;
};

/* this structure will be passed to GIC as an argument
 * which will be considered with physical interrupt */
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
static int apm_msix_parse_fdt(struct apm_msix_softc*);

static MALLOC_DEFINE(M_APM_MSIX, "msix", "APM MSI-X data structures");

#if 1
#define pr_debug(fmt,args...) do { printf("%s(): ", __func__); \
								   printf(fmt,##args); } while (0)
#define bootverbose 1
#else
#define pr_debug(fmt,args...)
#endif

static device_method_t apm_msix_methods[] = {
	DEVMETHOD(device_probe,			apm_msix_probe),
	DEVMETHOD(device_attach,		apm_msix_attach),
	DEVMETHOD_END
};

static driver_t apm_msix_driver = {
	"msix",
	apm_msix_methods,
	sizeof(struct apm_msix_softc),
};

static devclass_t apm_msix_devclass;

DRIVER_MODULE(msix, simplebus, apm_msix_driver,
		apm_msix_devclass, 0, 0);

static int
apm_msix_probe(device_t self)
{
	if (!ofw_bus_status_okay(self))
		return (ENXIO);

	if (!ofw_bus_is_compatible(self, "xgene,gic-msi"))
		return (ENXIO);

	device_set_desc(self, "APM MSI-X Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
apm_msix_attach(device_t self)
{
	pr_debug("enter\n");
	int err;
	struct apm_msix_softc* sc;
	struct resource_list rl;
	struct resource_list_entry* rle;
	phandle_t node;
	bus_addr_t reg_base = 0, reg_size = 0;
	int interrupts[NR_MSI_REG], interrupts_count = 0, rid;
	u_int irq_index, j, offset, count;

	sc = device_get_softc(self);
	node = ofw_bus_get_node(self);
	sc->dev = self;

	err = fdt_regsize(node, (u_long*)&reg_base, (u_long*)&reg_size);
	if (err) {
		device_printf(self, "ERROR: Failed to resolve address space (err: %d)\n", err);
		return (ENXIO);
	}

	/* allocate resources */
	sc->res = bus_alloc_resource_any(self, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(self, "ERROR: Failed to allocate device resources\n");
		return (ENXIO);
	}

	rid = 0;
	sc->irq_res = bus_alloc_resource_any(self, SYS_RES_IRQ, &rid,
											RF_SHAREABLE | RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(self, "ERROR: Failed to allocate IRQ resources\n");
		return (ENXIO);
	}

	/* initialize and manage memory region */
	sc->rman.rm_type = RMAN_ARRAY;
	sc->rman.rm_descr = "PCI MSI-X IRQ terminate registers";
	err = rman_init(&sc->rman);
	if (err) {
		device_printf(self, "ERROR: rman_init() for %s failed (err: %d)\n",
				sc->rman.rm_descr, err);
		return (err);
	}

	sc->irq_rman.rm_type = RMAN_ARRAY;
	sc->rman.rm_descr = "PCI MSI-X IRQ trigger registers";
	err = rman_init(&sc->irq_rman);
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

	err = rman_manage_region(&sc->irq_rman, MSI_IRQ_BASE, 0xcf0000);
	if (err) {
		device_printf(self, "ERROR: rman_manage_region() for %s failed (err: %d)\n",
					  sc->irq_rman.rm_descr, err);
		rman_fini(&sc->irq_rman);
		return (err);
	}

	/* store base addr */
	sc->base_addr = reg_base;

	err = apm_msix_parse_fdt(sc);
	if (err)
		return (ENXIO);

	/* set structures' memory */
	memset(irq_data_tbl, 0, sizeof(irq_data_tbl));
	memset(interrupts, 0, sizeof(interrupts));
	memset(&rl, 0, sizeof(rl));

	/* get resource list from FDT/OFW */
	resource_list_init(&rl);
	err = ofw_bus_intr_to_rl(self, node, &rl);
	if (err) {
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

	offset = sc->irq_min / IRQS_PER_MSI_REG;
	count = sc->irq_max / IRQS_PER_MSI_REG;

	/* set global software context */
	g_softc = sc;

	/* initialize mutex */
	mtx_init(&sc->msi_mtx, "msi_mtx", NULL, MTX_DEF);

	/* set up hardware IRQ handling */
	for (irq_index = 48, j = 0; j < count; j++, irq_index++) {
		err = apm_msix_setup_hwirq(self, offset + j, irq_index);
		if (err)
			goto error;
	}
	device_printf(self, "base address 0x%lx, size 0x%lx, MSI(X) IRQs %d-%d\n",
			sc->base_addr, reg_size, sc->irq_min, sc->irq_max);

	return (bus_generic_attach(self));

error:
	apm_msix_remove(self);
	return (err);
}

int
apm_msix_alloc(int count, int *irq)
{
	printf("%s(): enter\n", __func__);
	bus_write_4(g_softc->res, 0x0, 0x1);
	printf("%s(): leave\n", __func__);
	return (103923);

	u_int start, i, irq_min;
	pr_debug("enter\n");

	/* count must be a power of 2 */
	if (powerof2(count) == 0 || count > 8)
		return (EINVAL);

	if (g_softc == NULL)
		return (ENXIO);

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

	if (count == 1)
		device_printf(g_softc->dev, "allocated MSI(X) IRQ%d\n",
			irq_min + start);
	else
		device_printf(g_softc->dev, "allocated MSI(X) IRQ%d-%d\n",
			irq_min + start,
			irq_min + start + count-1);

	return (0);
}

int
apm_msix_map_msi(int irq, bus_addr_t* addr, uint32_t* data)
{
	device_t dev;
	u_int virt_irq;
	pr_debug("enter\n");

	if (g_softc == NULL)
		return (ENXIO);

	dev = g_softc->dev;

	/* validate IRQ number */
	if ((irq > g_softc->irq_max) || (irq < g_softc->irq_min) ||
			isclr(&g_softc->msi_bitmap, irq - g_softc->irq_min)) {
		device_printf(dev, "ERROR: invalid MSI(X) IRQ%d\n", irq);
		return (EINVAL);
	}

	*addr = MSI_IRQ_BASE + irq * 0x10000;
	*data = 1; /* writting 1 to register triggers interrupt */

	virt_irq = VIRT_IRQ_OFFSET + irq;
	if (arm_config_intr(virt_irq, INTR_TRIGGER_EDGE, INTR_POLARITY_HIGH)) {
		device_printf(dev, "ERROR: unable to config interrupt\n");
		return (EINVAL);
	}

	device_printf(dev, "MSI(X) mapping: IRQ%d, addr %#lx, data %#x\n",
			irq, *addr, *data);

	return (0);
}

int
apm_msix_release(int count, int *irq)
{
	int i;
	pr_debug("enter\n");

	if (g_softc == NULL)
		return (ENXIO);

	mtx_lock(&g_softc->msi_mtx);

	for (i = 0; i < count; i++)
		clrbit(&g_softc->msi_bitmap, irq[i] - g_softc->irq_min);

	mtx_unlock(&g_softc->msi_mtx);

	pr_debug("end\n");
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
	pr_debug("enter\n");

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
	pr_debug("end\n");
}

void
apm_msix_handle_irq(void *arg)
{
	struct apm_irq_data *irq_data;
	int msir_index = -1;
	bus_addr_t msi_intr_reg;
	uint32_t msir_value, intr_index, msi_intr_reg_value;
	u_int virt_irq;
	pr_debug("enter\n");

	if (arg == NULL || g_softc == NULL) {
		printf("%s() ERROR: received interrupt w/o data or"
				"software context is not set\n", __func__);
		return;
	}
	irq_data = (struct apm_irq_data*)arg;

	/* get HW IRQ number: 0-16 */
	msi_intr_reg = irq_data->offset;
	/* read which registers in group was set */
	msi_intr_reg_value = apm_msi_intr_read(g_softc->base_addr, msi_intr_reg);

	pr_debug("MSI-X: read value %#x from register %#lx\n", msi_intr_reg_value, msi_intr_reg);

	/* handle all set interrupts */
	while (msi_intr_reg_value != 0) {
		/* get concrete register from registers' group */
		msir_index = ffs(msi_intr_reg_value) - 1;

		/* read value from register -> what interrupt number was written into it */
		msir_value = apm_msir_read(g_softc->base_addr, msi_intr_reg, msir_index);

		while (msir_value != 0) {
			/* proceed with first set bit */
			intr_index = ffs(msir_value) - 1;
			/* calculate 'virtual' interrupt */
			virt_irq =
				intr_index * NR_MSI_REG + /* Message Interrupt number */
				msir_index * IRQS_PER_MSI_INDEX * NR_MSI_REG + /* group */
				msi_intr_reg; /* hw_irq */
			/* dispatch interrupt to proper device */
			if (virt_irq != 0)
				apm_msix_dispatch_virt_irq(virt_irq);
			/* reset current IRQ bit */
			msir_value &= ~(1 << intr_index);
		}
		/* reset current MSI-X register bit*/
		msi_intr_reg_value &= ~(1 << msir_index);
	}
	pr_debug("end\n");
	return;
}

static int
apm_msix_setup_hwirq(device_t dev, int offset, int irq)
{
	int flags = INTR_TYPE_AV, err;
	struct apm_irq_data *irq_data = NULL;
	pr_debug("enter\n");

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
	else {
		device_printf(dev, "MSI-X: hw setup of IRQ%d\n", irq);

		/* config HW IRQ to be triggered by edge */
		err = arm_config_intr(irq, INTR_TRIGGER_EDGE, INTR_POLARITY_HIGH);
		if (err)
			device_printf(dev, "ERROR: unable to configure IRQ%d (err=%d)\n", irq, err);
	}
	pr_debug("end\n");
	return (err);
}

int
apm_msix_setup_irq(device_t dev, device_t child, struct resource *res, int flags,
	    driver_filter_t *filt, driver_intr_t *intr, void *arg, void **cookiep)
{
	int virt_irq, error;
	pr_debug("enter\n");

	if ((rman_get_flags(res) & RF_SHAREABLE) == 0)
		flags |= INTR_EXCL;

	/* We depend here on rman_activate_resource() being idempotent. */
	error = rman_activate_resource(res);
	if (error)
		return (error);

	/* This IRQ will be considered as 'virtual' */
	virt_irq = rman_get_start(res); /* + VIRT_IRQ_OFFSET */
	pr_debug("irq_req = %d\n", virt_irq);

	error = arm_setup_intr(device_get_nameunit(child), filt, intr,
		arg, virt_irq, flags, cookiep);

	pr_debug("end (err=%d)\n", error);

	return (error);
}

static void
apm_msix_remove(device_t dev)
{
	int i;
	pr_debug("enter\n");

	for (i = 0; i < NR_MSI_REG; i++) {
		if (irq_data_tbl[i] != NULL)
			free(irq_data_tbl[i], M_APM_MSIX);
			irq_data_tbl[i] = NULL;
	}

	pr_debug("end\n");
	return;
}

static int
apm_msix_parse_fdt(struct apm_msix_softc *sc)
{
	int cells_count;
	pcell_t *addr_buf;
	phandle_t node;
	pr_debug("enter\n");

	node = ofw_bus_get_node(sc->dev);

	/*
	 * Get 'msi-available-ranges' property
	 */
	cells_count = OF_getprop_alloc(node, "msi-available-ranges",
		sizeof(pcell_t), (void **)&addr_buf);
	if (cells_count == -1 || cells_count > 2) {
		device_printf(sc->dev, "ERROR: wrong FDT 'msi-available-ranges' property\n");
		return (ENXIO);
	}

	/* get min IRQ number */
	sc->irq_min = fdt_data_get((void *)addr_buf++, 1);
	/* get max IRQ number*/
	sc->irq_max = fdt_data_get((void *)addr_buf, 1);

	/* sanity check of data */
	if ((sc->irq_min < 0) ||
		(sc->irq_max - sc->irq_min > NR_MSI_IRQS) ||
		(sc->irq_max < sc->irq_min)) {
		device_printf(sc->dev, "ERROR: Incorrect range of IRQ numbers in FDT"
				"'msi-available-ranges' property\n");
		return (EINVAL);
	}
	sc->irq_count = sc->irq_max - sc->irq_min + 1;

	if (bootverbose) {
		device_printf(sc->dev, "Read from FDT: irq_min = %d, irq_max = %d, count = %d\n",
				sc->irq_min, sc->irq_max, sc->irq_count);
	}

	return (0);
}
