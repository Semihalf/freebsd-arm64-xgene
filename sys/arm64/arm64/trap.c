/*-
 * Copyright (c) 2014 Andrew Turner
 * All rights reserved.
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/pioctl.h>
#include <sys/proc.h>
#include <sys/ptrace.h>
#include <sys/syscall.h>
#include <sys/sysent.h>
#ifdef KDB
#include <sys/kdb.h>
#endif

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_kern.h>
#include <vm/vm_map.h>
#include <vm/vm_extern.h>

#include <machine/frame.h>
#include <machine/pcb.h>

#ifdef VFP
#include <machine/vfp.h>
#endif

#ifdef KDB
#include <machine/db_machdep.h>
#endif

#ifdef DDB
#include <ddb/db_output.h>
#endif



/* Called from exception.S */
void do_el1h_sync(struct trapframe *);
void do_el0_sync(struct trapframe *);
void do_el0_error(struct trapframe *);

static __inline void
call_trapsignal(struct thread *td, int sig, u_long code)
{
	ksiginfo_t ksi;

	ksiginfo_init_trap(&ksi);
	ksi.ksi_signo = sig;
	ksi.ksi_code = (int)code;
	trapsignal(td, &ksi);
}

int
cpu_fetch_syscall_args(struct thread *td, struct syscall_args *sa)
{
	struct proc *p;
	register_t *ap;
	int nap;

	nap = 8;
	p = td->td_proc;
	ap = td->td_frame->tf_x;

	sa->code = td->td_frame->tf_x[8];

	if (sa->code == SYS_syscall || sa->code == SYS___syscall) {
		sa->code = *ap++;
		nap--;
	}

	if (p->p_sysent->sv_mask)
		sa->code &= p->p_sysent->sv_mask;
	if (sa->code >= p->p_sysent->sv_size)
		sa->callp = &p->p_sysent->sv_table[0];
	else
		sa->callp = &p->p_sysent->sv_table[sa->code];

	sa->narg = sa->callp->sy_narg;
	memcpy(sa->args, ap, nap * sizeof(register_t));
	if (sa->narg > nap)
		panic("TODO: Could we have more then 8 args?");

	td->td_retval[0] = 0;
	td->td_retval[1] = 0;

	return (0);
}

#include "../../kern/subr_syscall.c"

static void
svc_handler(struct trapframe *frame)
{
	struct syscall_args sa;
	struct thread *td;
	int error;

	td = curthread;
	td->td_frame = frame;

	error = syscallenter(td, &sa);
	syscallret(td, error, &sa);
}

static void
data_abort(struct trapframe *frame, uint64_t esr, int lower)
{
	struct vm_map *map;
	struct thread *td;
	struct proc *p;
	struct pcb *pcb;
	vm_prot_t ftype;
	vm_offset_t va;
	uint64_t far;
	int error, sig;

	far = READ_SPECIALREG(far_el1);

	td = curthread;
	p = td->td_proc;
	pcb = td->td_pcb;

	if (lower)
		map = &td->td_proc->p_vmspace->vm_map;
	else {
		/* The top bit tells us which range to use */
		if ((far >> 63) == 1)
			map = kernel_map;
		else
			map = &td->td_proc->p_vmspace->vm_map;
	}

	va = trunc_page(far);
	ftype = ((esr >> 6) & 1) ? VM_PROT_READ | VM_PROT_WRITE : VM_PROT_READ;

	if (map != kernel_map) {
		/*
		 * Keep swapout from messing with us during this
		 *	critical time.
		 */
		PROC_LOCK(p);
		++p->p_lock;
		PROC_UNLOCK(p);

		/* Fault in the user page: */
		error = vm_fault(map, va, ftype, VM_FAULT_NORMAL);

		PROC_LOCK(p);
		--p->p_lock;
		PROC_UNLOCK(p);
	} else {
		/*
		 * Don't have to worry about process locking or stacks in the
		 * kernel.
		 */
		error = vm_fault(map, va, ftype, VM_FAULT_NORMAL);
	}

	if (error != 0) {
		if (lower) {
			if (error == ENOMEM)
				sig = SIGKILL;
			else
				sig = SIGSEGV;
			call_trapsignal(td, sig, 0);
		} else {
			if (td->td_intr_nesting_level == 0 &&
			    pcb->pcb_onfault != 0) {
				frame->tf_x[0] = error;
				frame->tf_elr = pcb->pcb_onfault;
				return;
			}
			panic("vm_fault failed: %lx", frame->tf_elr);
		}
	}

	if (lower)
		userret(td, frame);
}

static void
print_registers(struct trapframe *frame)
{
	u_int reg;

	for (reg = 0; reg < 31; reg++) {
		printf(" %sx%d: %lx\n", (reg < 10) ? " " : "", reg, frame->tf_x[reg]);
	}
	printf("  sp: %lx\n", frame->tf_sp);
	printf("  lr: %lx\n", frame->tf_lr);
	printf(" elr: %lx\n", frame->tf_elr);
	printf("spsr: %lx\n", frame->tf_spsr);
}

void
do_el1h_sync(struct trapframe *frame)
{
	uint32_t exception;
	uint64_t esr;

	/* Read the esr register to get the exception details */
	esr = READ_SPECIALREG(esr_el1);
	exception = ESR_ELx_EXCEPTION(esr);

	/*
	 * Sanity check we are in an exception er can handle. The IL bit
	 * is used to indicate the instruction length, except in a few
	 * exceptions described in the ARMv8 ARM.
	 *
	 * It is unclear in some cases if the bit is implementation defined.
	 * The Foundation Model and QEMU disagree on if the IL bit should
	 * be set when we are in a data fault from the same EL and the ISV
	 * bit (bit 24) is also set.
	 */
	KASSERT((esr & ESR_ELx_IL) == ESR_ELx_IL ||
	    (exception == EXCP_DATA_ABORT && ((esr & ISS_DATA_ISV) == 0)),
	    ("Invalid instruction length in exception"));

	if (0) {
		printf("In do_el1h_sync\n");
		printf(" esr: %lx\n", esr);
		printf("excp: %x\n", exception);
		print_registers(frame);
	}

	switch(exception) {
	case EXCP_FP_SIMD:
	case EXCP_TRAP_FP:
		panic("VFP exception in the kernel");
	case EXCP_DATA_ABORT:
		data_abort(frame, esr, 0);
		break;
	case EXCP_BRK:
		printf("Breakpoint %x\n", (uint32_t)(esr & ESR_ELx_ISS_MASK));
#ifdef KDB
		kdb_trap(T_BREAKPOINT, 0, frame);
#else
		panic("No debugger in kernel.\n");
#endif
		break;
	default:
		print_registers(frame);
		panic("Unknown kernel exception %x esr_el1 %lx\n", exception,
		    esr);
	}
}

void
do_el0_sync(struct trapframe *frame)
{
	uint32_t exception;
	uint64_t esr;

	esr = READ_SPECIALREG(esr_el1);
	exception = ESR_ELx_EXCEPTION(esr);

	if (0)
	{
		printf("In do_el0_sync\n");
		printf(" esr: %lx\n", esr);
		printf("excp: %x\n", exception);
		print_registers(frame);
	}

	switch(exception) {
	case EXCP_FP_SIMD:
	case EXCP_TRAP_FP:
#ifdef VFP
		vfp_restore_state();
#else
		panic("VFP exception in userland");
#endif
		break;
	case EXCP_SVC:
		svc_handler(frame);
		break;
	case EXCP_INSN_ABORT_L:
	case EXCP_DATA_ABORT_L:
		data_abort(frame, esr, 1);
		break;
	default:
		print_registers(frame);
		panic("Unknown userland exception %x esr_el1 %lx\n", exception,
		    esr);
	}
}

void
do_el0_error(struct trapframe *frame)
{
	u_int reg;

	for (reg = 0; reg < 31; reg++) {
		printf(" %sx%d: %lx\n", (reg < 10) ? " " : "", reg, frame->tf_x[reg]);
	}
	printf("  sp: %lx\n", frame->tf_sp);
	printf("  lr: %lx\n", frame->tf_lr);
	printf(" elr: %lx\n", frame->tf_elr);
	printf("spsr: %lx\n", frame->tf_spsr);
	panic("do_el0_error");
}

