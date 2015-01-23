/*-
 * Mach Operating System
 * Copyright (c) 1991,1990 Carnegie Mellon University
 * All Rights Reserved.
 *
 * Permission to use, copy, modify and distribute this software and its
 * documentation is hereby granted, provided that both the copyright
 * notice and this permission notice appear in all copies of the
 * software, derivative works or modified versions, and any portions
 * thereof, and that both notices appear in supporting documentation.
 *
 * CARNEGIE MELLON ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS"
 * CONDITION.  CARNEGIE MELLON DISCLAIMS ANY LIABILITY OF ANY KIND FOR
 * ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
 *
 * Carnegie Mellon requests users of this software to return to
 *
 *  Software Distribution Coordinator  or  Software.Distribution@CS.CMU.EDU
 *  School of Computer Science
 *  Carnegie Mellon University
 *  Pittsburgh PA 15213-3890
 *
 * any improvements or extensions that they make and grant Carnegie Mellon
 * the rights to redistribute these changes.
 *
 *	from: FreeBSD: src/sys/i386/include/db_machdep.h,v 1.16 1999/10/04
 * $FreeBSD$
 */

#ifndef	_MACHINE_DB_MACHDEP_H_
#define	_MACHINE_DB_MACHDEP_H_

#include <machine/armreg.h>
#include <machine/frame.h>
#include <machine/trap.h>

#define T_BREAKPOINT	(EXCP_BRK)
#define T_WATCHPOINT	(EXCP_WATCHPT_EL1)

typedef vm_offset_t	db_addr_t;
typedef long		db_expr_t;

#define	PC_REGS()	((db_addr_t)kdb_thrctx->pcb_pc)

#define	BKPT_INST	(0xd4200000)
#define	BKPT_SIZE	(4)
#define	BKPT_SET(inst)	(BKPT_INST)

#define	BKPT_SKIP do {							\
	kdb_frame->tf_elr += BKPT_SIZE; \
} while (0)

#define db_clear_single_step	kdb_cpu_clear_singlestep
#define db_set_single_step	kdb_cpu_set_singlestep

#define	IS_BREAKPOINT_TRAP(type, code)	(type == T_BREAKPOINT)
#define	IS_WATCHPOINT_TRAP(type, code)	(type == T_WATCHPOINT)

#define	inst_trap_return(ins)	(0)
/* ret */
#define	inst_return(ins)	(((ins) & 0xfffffc1fu) == 0xd65f0000)
#define	inst_call(ins)		(((ins) & 0xfc000000u) == 0x94000000u || /* BL */ \
				 ((ins) & 0xfffffc1fu) == 0xd63f0000u) /* BLR */
/* b, b.cond, br. TODO: b.cond & br */
#define	inst_branch(ins)	(((ins) & 0xfc000000u) == 0x14000000u)

#define inst_load(ins) ({							\
	uint32_t tmp_instr = db_get_value(PC_REGS(), sizeof(uint32_t), FALSE);	\
	is_load_instr(tmp_instr);						\
})

#define inst_store(ins) ({							\
	uint32_t tmp_instr = db_get_value(PC_REGS(), sizeof(uint32_t), FALSE);	\
	is_store_instr(tmp_instr);						\
})

#define	is_load_instr(ins)	((((ins) & 0x3b000000u) == 0x18000000u) || /* literal */ \
				 (((ins) & 0x3f400000u) == 0x08400000u) ||  /* exclusive */ \
				 (((ins) & 0x3bc00000u) == 0x28400000u) || /* no-allocate pair */ \
				 ((((ins) & 0x3b200c00u) == 0x38000400u) && \
				  (((ins) & 0x3be00c00u) != 0x38000400u) && \
				  (((ins) & 0xffe00c00u) != 0x3c800400u)) || /* immediate post-indexed */ \
				 ((((ins) & 0x3b200c00u) == 0x38000c00u) && \
				  (((ins) & 0x3be00c00u) != 0x38000c00u) && \
				  (((ins) & 0xffe00c00u) != 0x3c800c00u)) || /* immediate pre-indexed */ \
				 ((((ins) & 0x3b200c00u) == 0x38200800u) && \
				  (((ins) & 0x3be00c00u) != 0x38200800u) && \
				  (((ins) & 0xffe00c00u) != 0x3ca00c80u)) || /* register offset */ \
				 ((((ins) & 0x3b200c00u) == 0x38000800u) && \
				  (((ins) & 0x3be00c00u) != 0x38000800u)) || /* unprivileged */ \
				 ((((ins) & 0x3b200c00u) == 0x38000000u) && \
				  (((ins) & 0x3be00c00u) != 0x38000000u) && \
				  (((ins) & 0xffe00c00u) != 0x3c800000u)) ||  /* unscaled immediate */ \
				 ((((ins) & 0x3b000000u) == 0x39000000u) && \
				  (((ins) & 0x3bc00000u) != 0x39000000u) && \
				  (((ins) & 0xffc00000u) != 0x3d800000u)) &&  /* unsigned immediate */ \
				 (((ins) & 0x3bc00000u) == 0x28400000u) || /* pair (offset) */ \
				 (((ins) & 0x3bc00000u) == 0x28c00000u) || /* pair (post-indexed) */ \
				 (((ins) & 0x3bc00000u) == 0x29800000u)) /* pair (pre-indexed) */

#define	is_store_instr(ins)	((((ins) & 0x3f400000u) == 0x08000000u) || /* exclusive */ \
				 (((ins) & 0x3bc00000u) == 0x28000000u) || /* no-allocate pair */ \
				 ((((ins) & 0x3be00c00u) == 0x38000400u) || \
				  (((ins) & 0xffe00c00u) == 0x3c800400u)) || /* immediate post-indexed */ \
				 ((((ins) & 0x3be00c00u) == 0x38000c00u) || \
				  (((ins) & 0xffe00c00u) == 0x3c800c00u)) || /* immediate pre-indexed */ \
				 ((((ins) & 0x3be00c00u) == 0x38200800u) || \
				  (((ins) & 0xffe00c00u) == 0x3ca00800u)) || /* register offset */ \
				 (((ins) & 0x3be00c00u) == 0x38000800u) ||  /* unprivileged */ \
				 ((((ins) & 0x3be00c00u) == 0x38000000u) || \
				  (((ins) & 0xffe00c00u) == 0x3c800000u)) ||  /* unscaled immediate */ \
				 ((((ins) & 0x3bc00000u) == 0x39000000u) || \
				  (((ins) & 0xffc00000u) == 0x3d800000u)) ||  /* unsigned immediate */ \
				 (((ins) & 0x3bc00000u) == 0x28000000u) || /* pair (offset) */ \
				 (((ins) & 0x3bc00000u) == 0x28800000u) || /* pair (post-indexed) */ \
				 (((ins) & 0x3bc00000u) == 0x29800000u)) /* pair (pre-indexed) */

#define next_instr_address(pc, bd)	((bd) ? (pc) : ((pc) + 4))

#define	DB_SMALL_VALUE_MAX	(0x7fffffff)
#define	DB_SMALL_VALUE_MIN	(-0x40001)

#define	DB_ELFSIZE		64

u_int branch_taken (u_int insn, u_int pc);

#endif /* !_MACHINE_DB_MACHDEP_H_ */
