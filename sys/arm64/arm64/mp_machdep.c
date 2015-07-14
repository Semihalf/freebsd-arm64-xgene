/*-
 * Copyright (c) 2015 The FreeBSD Foundation
 * All rights reserved.
 *
 * This software was developed by Andrew Turner under
 * sponsorship from the FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
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
 *
 */

#include "opt_platform.h"

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/sched.h>
#include <sys/smp.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_extern.h>
#include <vm/vm_kern.h>

#include <machine/intr.h>
#include <machine/smp.h>
#include <machine/bus.h>
#include <machine/fdt.h>
#include <machine/psci.h>
#ifdef VFP
#include <machine/vfp.h>
#endif

#ifdef FDT
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_cpu.h>
#include <dev/ofw/ofw_bus.h>
#endif

extern struct pcpu __pcpu[];

static enum {
	CPUS_UNKNOWN,
#ifdef FDT
	CPUS_FDT,
#endif
} cpu_enum_method;

enum ap_release_method {
	RELEASE_UNKNOWN,
	RELEASE_PSCI,
	RELEASE_SPINTABLE,
};

static device_identify_t arm64_cpu_identify;
static device_probe_t arm64_cpu_probe;
static device_attach_t arm64_cpu_attach;

static int ipi_handler(void *arg);

struct mtx ap_boot_mtx;
struct pcb stoppcbs[MAXCPU];

#ifdef INVARIANTS
static uint32_t cpu_reg[MAXCPU][2];
#endif
static device_t cpu_list[MAXCPU];
static phandle_t cpu_node_list[MAXCPU];

void mpentry(unsigned long cpuid);
void init_secondary(uint64_t);

uint8_t secondary_stacks[MAXCPU - 1][PAGE_SIZE * KSTACK_PAGES] __aligned(16);

/* # of Applications processors */
volatile int mp_naps;
/* # of CPU sockets */
int mp_cpu_sockets;
/* # of CPU clusters per socket */
int mp_cpu_clusters;
/* # of all CPUs in FDT */
int mp_allcpus;
/* Set to 1 once we're ready to let the APs out of the pen. */
volatile int aps_ready = 0;

/* Temporary variables for init_secondary()  */
void *dpcpu[MAXCPU - 1];

static device_method_t arm64_cpu_methods[] = {
	/* Device interface */
	DEVMETHOD(device_identify,	arm64_cpu_identify),
	DEVMETHOD(device_probe,		arm64_cpu_probe),
	DEVMETHOD(device_attach,	arm64_cpu_attach),

	DEVMETHOD_END
};

static devclass_t arm64_cpu_devclass;
static driver_t arm64_cpu_driver = {
	"arm64_cpu",
	arm64_cpu_methods,
	0
};

DRIVER_MODULE(arm64_cpu, cpu, arm64_cpu_driver, arm64_cpu_devclass, 0, 0);

static void
arm64_cpu_identify(driver_t *driver, device_t parent)
{

	if (device_find_child(parent, "arm64_cpu", -1) != NULL)
		return;
	if (BUS_ADD_CHILD(parent, 0, "arm64_cpu", -1) == NULL)
		device_printf(parent, "add child failed\n");
}

static int
arm64_cpu_probe(device_t dev)
{
	u_int cpuid;

	cpuid = device_get_unit(dev);
	if (cpuid >= MAXCPU || cpuid > mp_maxid)
		return (EINVAL);

	return (0);
}

static int
arm64_cpu_attach(device_t dev)
{
	const uint32_t *reg;
	size_t reg_size;
	u_int cpuid;
	int i;

	cpuid = device_get_unit(dev);

	if (cpuid >= MAXCPU || cpuid > mp_maxid)
		return (EINVAL);
	KASSERT(cpu_list[cpuid] == NULL, ("Already have cpu %u", cpuid));

	reg = cpu_get_cpuid(dev, &reg_size);
	if (reg == NULL)
		return (EINVAL);

	device_printf(dev, "Found register:");
	for (i = 0; i < reg_size; i++)
		printf(" %x", reg[i]);
	printf("\n");

	/* Set the device to start it later */
	cpu_list[cpuid] = dev;

	return (0);
}

static enum ap_release_method
check_release_method_fdt(device_t dev, u_int cpuid, bus_addr_t *release_addr)
{
	phandle_t node;
	pcell_t	cpu_release_addr[2];
	pcell_t addr_cells;
	char enable_method[16];
	enum ap_release_method rel_method;
	int rv;

	rel_method = RELEASE_UNKNOWN;
	node = cpu_node_list[cpuid];

	/* Get the release method for this cpu from FDT */
	if (!OF_hasprop(node, "enable-method")) {
		printf("No enable-method property in FDT\n");
		return (rel_method);
	}

	OF_getprop(node, "enable-method", enable_method, sizeof(enable_method));
	if (strcmp(enable_method, "psci") == 0)
		rel_method = RELEASE_PSCI;
	else if (strcmp(enable_method, "spin-table") == 0)
		rel_method = RELEASE_SPINTABLE;
	else {
		printf("Invalid enable-method property in FDT\n");
		return (rel_method);
	}

	if (rel_method == RELEASE_SPINTABLE) {
		/* Retrieve release address for this CPU */
		addr_cells = 2;
		OF_getencprop(OF_parent(node), "#address-cells", &addr_cells,
		    sizeof(addr_cells));
		rv = OF_getencprop(node, "cpu-release-addr", cpu_release_addr,
		    addr_cells * sizeof(pcell_t));
		if (rv != addr_cells * sizeof(pcell_t)) {
			printf("Invalid cpu-release-addr property in FDT\n");
			return (RELEASE_UNKNOWN);
		}
		*release_addr = ((bus_addr_t)cpu_release_addr[0] << 32) |
		    cpu_release_addr[1];
	}

	return (rel_method);

}

#define DO_SEV_LATER
static void
spintable_release_cpu(u_int cpuid, vm_paddr_t pa, bus_addr_t
    release_addr)
{
	bus_space_handle_t handle;

	if (bootverbose)
		printf("Writing to release address 0x%lx\n",
		    release_addr);
	if (bus_space_map(fdtbus_bs_tag, release_addr, 8, 0, &handle))
		panic("Could not map CPU release address");
	dsb();
	__asm __volatile(
	    "ic ialluis");
	bus_space_write_8(fdtbus_bs_tag, handle, 0, pa);
	dsb();
	__asm __volatile(
	    "ic ialluis");
#ifndef DO_SEV_LATER
	__asm __volatile(
	    "sev" : : : "memory");
#endif
	__asm __volatile(
	    "nop");
	bus_space_unmap(fdtbus_bs_tag, handle, 8);
}

static void
spintable_do_sev(void)
{
	dsb();
	__asm __volatile(
	    "ic ialluis");
	__asm __volatile(
	    "sev" : : : "memory");
	__asm __volatile(
	    "nop");
	dsb();
}

static boolean_t
arm64_cpu_start(device_t dev, u_int cpuid)
{
	const uint32_t *reg;
	uint64_t target_cpu;
	enum ap_release_method rel_method;
	bus_addr_t release_addr;
	vm_paddr_t pa;
	size_t count;

	KASSERT(cpuid < MAXCPU, ("Trying to start too many CPUs"));

	if (dev == NULL) {
		printf("No device for cpu %u\n", cpuid);
		return (false);
	}

	KASSERT(cpuid == device_get_unit(dev),
	    ("cpuid inconsistency detected"));
	reg = cpu_get_cpuid(dev, &count);
	if (reg == NULL) {
		printf("No CPU register for CPU %u\n", cpuid);
		return (false);
	}

	KASSERT(reg[0] == cpu_reg[cpuid][0],
	    ("CPU register inconsistency detected"));
	KASSERT(count == 1 || reg[1] == cpu_reg[cpuid][1],
	    ("CPU register inconsistency detected"));

	if (cpuid > 0) {
		target_cpu = reg[0];
		if (count == 2) {
			target_cpu <<= 32;
			target_cpu |= reg[1];
		}

		printf("Starting CPU %u (%lx)\n", cpuid, target_cpu);
		pa = pmap_extract(kernel_pmap, (vm_offset_t)mpentry);

		rel_method = check_release_method_fdt(dev, cpuid, &release_addr);
		if (rel_method == RELEASE_PSCI)
			psci_cpu_on(target_cpu, pa, cpuid);
		else if (rel_method == RELEASE_SPINTABLE)
			spintable_release_cpu(cpuid, pa, release_addr);
		else
			printf("Failed to read release method from FDT\n");
	}

	return (true);
}

static void
release_aps(void *dummy __unused)
{
	int i, running, timeout;

	running = 0;
	for (i = 0; i <= mp_maxid; i++)
		if (arm64_cpu_start(cpu_list[i], i))
			running++;

#ifdef	DO_SEV_LATER
	/* Wake up all secondary cores */
	spintable_do_sev();
#endif

	/* TODO: We will hit when some cores are disabled */
	KASSERT(running == mp_ncpus, ("Unable to start all cores %u != %u",
	    running, mp_ncpus));
	if (running > 1) {
		int started = running - 1;

		for (timeout = 0; timeout < 2000; timeout++) {
			if (mp_naps == started)
				break;
			DELAY(10);
		}
		if (mp_naps != started)
			panic("Some AP's failed to start");
		else
			/* TODO: This assumes there are no disabled cores */
			for (i = 1; i < running; i++)
				CPU_SET(i, &all_cpus);
	}

	/* Setup the IPI handler */
	for (i = 0; i < COUNT_IPI; i++) {
		arm_setup_ipihandler(ipi_handler, i);
	}

	atomic_store_rel_int(&aps_ready, 1);

	printf("Running on %u CPUs\n", running);
}
SYSINIT(start_aps, SI_SUB_SMP, SI_ORDER_FIRST, release_aps, NULL);

void
init_secondary(uint64_t cpu)
{
	struct pcpu *pcpup;
	uint64_t mpidr;
	int i;

	DELAY(cpu * 500);

	pcpup = &__pcpu[cpu];
	/*
	 * Set the pcpu pointer with a backup in tpidr_el1 to be
	 * loaded when entering the kernel from userland.
	 */
	__asm __volatile(
	    "mov x18, %0 \n"
	    "msr tpidr_el1, %0" :: "r"(pcpup));
	/*
	 * Save affinity for the secondary CPU.
	 */
	mpidr = get_mpidr();
	CPU_AFFINITY(cpu) = mpidr & CPU_AFF_MASK;
	/*
	 * pcpu_init() updates queue, so it should not be executed in parallel
	 * on several cores
	 */
	while(mp_naps < (cpu - 1))
		;

	/* Signal our startup to BSP */
	atomic_add_rel_32(&mp_naps, 1);

	/* Spin until the BSP releases the APs */
	while (!aps_ready)
		;

	/* Initialize curthread */
	KASSERT(PCPU_GET(idlethread) != NULL, ("no idle thread"));
	pcpup->pc_curthread = pcpup->pc_idlethread;
	pcpup->pc_curpcb = pcpup->pc_idlethread->td_pcb;

	/* Configure the interrupt controller */
	arm_init_secondary();

	for (i = 0; i < COUNT_IPI; i++)
		arm_unmask_ipi(i);

	/* Start per-CPU event timers. */
	cpu_initclocks_ap();

#ifdef VFP
	vfp_init();
#endif

	/* Enable interrupts */
	intr_enable();

	mtx_lock_spin(&ap_boot_mtx);

	atomic_add_rel_32(&smp_cpus, 1);

	if (smp_cpus == mp_ncpus) {
		/* enable IPI's, tlb shootdown, freezes etc */
		atomic_store_rel_int(&smp_started, 1);
	}

	mtx_unlock_spin(&ap_boot_mtx);

	/* Enter the scheduler */
	sched_throw(NULL);

	panic("scheduler returned us to init_secondary");
	/* NOTREACHED */
}

static int
ipi_handler(void *arg)
{
	u_int cpu, ipi;

	arg = (void *)((uintptr_t)arg & ~(1 << 16));
	KASSERT((uintptr_t)arg < COUNT_IPI,
	    ("Invalid IPI %ju", (uintptr_t)arg));

	cpu = PCPU_GET(cpuid);
	ipi = (uintptr_t)arg;

	switch(ipi) {
	case IPI_AST:
		printf("TODO: AST\n");
		break;
	case IPI_PREEMPT:
		CTR1(KTR_SMP, "%s: IPI_PREEMPT", __func__);
		sched_preempt(curthread);
		break;
	case IPI_RENDEZVOUS:
		CTR0(KTR_SMP, "IPI_RENDEZVOUS");
		smp_rendezvous_action();
		break;
	case IPI_STOP:
	case IPI_STOP_HARD:
		CTR0(KTR_SMP, (ipi == IPI_STOP) ? "IPI_STOP" : "IPI_STOP_HARD");
		savectx(&stoppcbs[cpu]);

		/* Indicate we are stopped */
		CPU_SET_ATOMIC(cpu, &stopped_cpus);

		/* Wait for restart */
		while (!CPU_ISSET(cpu, &started_cpus))
			cpu_spinwait();

		CPU_CLR_ATOMIC(cpu, &started_cpus);
		CPU_CLR_ATOMIC(cpu, &stopped_cpus);
		CTR0(KTR_SMP, "IPI_STOP (restart)");
		break;
	case IPI_HARDCLOCK:
		CTR1(KTR_SMP, "%s: IPI_HARDCLOCK", __func__);
		hardclockintr();
		break;
	default:
		panic("ipi_handler %u", ipi);
	}

	return (FILTER_HANDLED);
}

struct cpu_group *
cpu_topo(void)
{

	return (smp_topo_none());
}

/* Determine if we running MP machine */
int
cpu_mp_probe(void)
{

	/* ARM64TODO: Read the u bit of mpidr_el1 to determine this */
	return (1);
}

#ifdef FDT
static boolean_t
cpu_set_topo_fdt(u_int id, phandle_t node, u_int addr_size, pcell_t *reg)
{
	int aff2, aff1;

	/* Find the highest Aff2 and Aff1 numbers among the cores */
	aff2 = (reg[1] >> 16) & 0xFF;
	aff1 = (reg[1] >> 8) & 0xFF;
	mp_cpu_sockets = max(mp_cpu_sockets, aff2 + 1);
	mp_cpu_clusters = max(mp_cpu_clusters, aff1 + 1);

	return (1);
}

static boolean_t
cpu_init_fdt(u_int id, phandle_t node, u_int addr_size, pcell_t *reg)
{
	struct pcpu *pcpup;

	/* Check we are able to start this cpu */
	if (id > mp_maxid)
		return (0);

	KASSERT(id < MAXCPU, ("Too mant CPUs"));

	KASSERT(addr_size == 1 || addr_size == 2, ("Invalid register size"));
#ifdef INVARIANTS
	cpu_reg[id][0] = reg[0];
	if (addr_size == 2)
		cpu_reg[id][1] = reg[1];
#endif

	/* We are already running on cpu 0 */
	if (id == 0)
		return (1);

	CPU_SET(id, &all_cpus);

	pcpup = &__pcpu[id];
	pcpu_init(pcpup, id, sizeof(struct pcpu));

	dpcpu[id - 1] = (void *)kmem_malloc(kernel_arena, DPCPU_SIZE,
	    M_WAITOK | M_ZERO);
	dpcpu_init(dpcpu[id - 1], id);

	/* Store CPU phandle to allow FDT parsing later */
	cpu_node_list[id] = node;

	return (1);
}
#endif

/* Initialize and fire up non-boot processors */
void
cpu_mp_start(void)
{

	mtx_init(&ap_boot_mtx, "ap boot", NULL, MTX_SPIN);

	CPU_SET(0, &all_cpus);

	switch(cpu_enum_method) {
#ifdef FDT
	case CPUS_FDT:
		ofw_cpu_early_foreach(cpu_init_fdt, true);
		break;
#endif
	case CPUS_UNKNOWN:
		break;
	}
}

/* Introduce rest of cores to the world */
void
cpu_mp_announce(void)
{
#ifdef FDT
	if (bootverbose) {
		int cpus_in_cluster;
		cpus_in_cluster = mp_allcpus / (mp_cpu_sockets * mp_cpu_clusters);

		printf("Sockets: %d, clusters per socket: %d, CPUs per cluster: %d\n",
		    mp_cpu_sockets, mp_cpu_clusters, cpus_in_cluster);
	}
#endif
}

void
cpu_mp_setmaxid(void)
{
#ifdef FDT
	int cores;

	cores = ofw_cpu_early_foreach(cpu_set_topo_fdt, false);
	if (cores > 0) {
		mp_allcpus = cores;
		cores = MIN(cores, MAXCPU);
		if (bootverbose)
			printf("Found %d CPUs in the device tree\n", cores);
		mp_ncpus = cores;
		mp_maxid = cores - 1;
		cpu_enum_method = CPUS_FDT;
		return;
	}
#endif

	if (bootverbose)
		printf("No CPU data, limiting to 1 core\n");
	mp_allcpus = 1;
	mp_ncpus = 1;
	mp_maxid = 0;
}
