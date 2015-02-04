/*-
 * Copyright (c) 2014 Andrew Turner
 * Copyright (c) 2014 The FreeBSD Foundation
 * All rights reserved.
 *
 * Portions of this software were developed by Andrew Turner
 * under sponsorship from the FreeBSD Foundation
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

#include <machine/asm.h>
__FBSDID("$FreeBSD$");

#include <machine/setjmp.h>

#include "assym.s"

/*
 * One of the fu* or su* functions failed, return -1.
 */
ENTRY(fsu_fault)
	SET_FAULT_HANDLER(xzr, x1)	/* Reset the handler function */
	mov	x0, #-1
	ret
END(fsu_fault)

/*
 * int fubyte(volatile const void *)
 */
ENTRY(fubyte)
	adr	x6, fsu_fault		/* Load the fault handler */
	SET_FAULT_HANDLER(x6, x1)	/* And set it */
	ldrb	w0, [x0]		/* Try loading the data */
	SET_FAULT_HANDLER(xzr, x1)	/* Reset the fault handler */
	ret				/* Return */
END(fubyte)

/*
 * int fuword(volatile const void *)
 */
ENTRY(fuword16)
	adr	x6, fsu_fault		/* Load the fault handler */
	SET_FAULT_HANDLER(x6, x1)	/* And set it */
	ldrh	w0, [x0]		/* Try loading the data */
	SET_FAULT_HANDLER(xzr, x1)	/* Reset the fault handler */
	ret				/* Return */
END(fuword16)

/*
 * int32_t fuword32(volatile const void *)
 */
ENTRY(fuword32)
	adr	x6, fsu_fault		/* Load the fault handler */
	SET_FAULT_HANDLER(x6, x1)	/* And set it */
	ldr	w0, [x0]		/* Try loading the data */
	SET_FAULT_HANDLER(xzr, x1)	/* Reset the fault handler */
	ret				/* Return */
END(fuword32)

/*
 * long fuword(volatile const void *)
 * int64_t fuword64(volatile const void *)
 */
ENTRY(fuword)
EENTRY(fuword64)
	adr	x6, fsu_fault		/* Load the fault handler */
	SET_FAULT_HANDLER(x6, x1)	/* And set it */
	ldr	x0, [x0]		/* Try loading the data */
	SET_FAULT_HANDLER(xzr, x1)	/* Reset the fault handler */
	ret				/* Return */
EEND(fuword64)
END(fuword)

ENTRY(setjmp)
	/* Store the stack pointer */
	mov	x8, sp
	str	x8, [x0]

	/* Store the general purpose registers and lr */
	stp	x19, x20, [x0], #16
	stp	x21, x22, [x0], #16
	stp	x23, x24, [x0], #16
	stp	x25, x26, [x0], #16
	stp	x27, x28, [x0], #16
	stp	x29, lr, [x0], #16

	/* Return value */
	mov	x0, #0
	ret
END(setjmp)

ENTRY(longjmp)
	/* Restore the stack pointer */
	ldr	x8, [x0], #8
	mov	sp, x8

	/* Restore the general purpose registers and lr */
	ldp	x19, x20, [x0], #16
	ldp	x21, x22, [x0], #16
	ldp	x23, x24, [x0], #16
	ldp	x25, x26, [x0], #16
	ldp	x27, x28, [x0], #16
	ldp	x29, lr, [x0], #16

	/* Load the return value */
	mov	x0, x1
	ret
END(longjmp)
