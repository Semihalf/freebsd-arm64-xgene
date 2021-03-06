/*-
 * Copyright (c) 2008-2010 Rui Paulo <rpaulo@FreeBSD.org>
 * Copytight (c) 2014 The FreeBSD Foundation
 * All rights reserved.
 *
 * Portions of this software were developed by Andrew Turner
 * under sponsorship from the FreeBSD Foundation.
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/types.h>

#include <stdlib.h>

#include "debug.h"
#include "rtld.h"
#include "rtld_printf.h"

/*
 * It is possible for the compiler to emit relocations for unaligned data.
 * We handle this situation with these inlines.
 */
#define	RELOC_ALIGNED_P(x) \
	(((uintptr_t)(x) & (sizeof(void *) - 1)) == 0)

/*
 * This is not the correct prototype, but we only need it for
 * a function pointer to a simple asm function.
 */
void *_rtld_tlsdesc(void *);

/*
 * Perform early relocation of the run-time linker image
 */
void
reloc_non_plt_self(Elf64_Dyn *dynamic, Elf_Addr relocbase)
{
	unsigned long relent, relcnt;
	unsigned long *newaddr;
	Elf64_Rela *rel;
	Elf64_Dyn *dynp;

	/*
	 * Find the relocation address, its size and the relocation entry.
	 */
	relent = 0;
	relcnt = 0;
	for (dynp = dynamic; dynp->d_tag != DT_NULL; dynp++) {
		switch (dynp->d_tag) {
		case DT_RELA:
			rel = (Elf64_Rela *)((caddr_t)dynp->d_un.d_ptr +
			    relocbase);
			break;
		case DT_RELACOUNT:
			relcnt = dynp->d_un.d_val;
			break;
		case DT_RELAENT:
			relent = dynp->d_un.d_val;
			break;
		default:
			break;
		}
	}

	/*
	 * Perform the actual relocation.
	 */
	for (; relcnt != 0; relcnt--) {
		switch (ELF64_R_TYPE(rel->r_info)) {
		case R_AARCH64_RELATIVE:
			newaddr = (unsigned long *)(relocbase + rel->r_offset);
			*newaddr = relocbase + rel->r_addend;
			break;
		default:
			/* XXX: do we need other relocations ? */
			break;
		}
		rel = (Elf64_Rela *)((caddr_t)rel + relent);
	}
}

void _exit(int);

void
init_pltgot(Obj_Entry *obj)
{

	if (obj->pltgot != NULL) {
		obj->pltgot[1] = (Elf_Addr) obj;
		obj->pltgot[2] = (Elf_Addr) &_rtld_bind_start;
	}
}

int
do_copy_relocations(Obj_Entry *dstobj)
{
	const Obj_Entry *srcobj, *defobj;
	const Elf_Rela *relalim;
	const Elf_Rela *rela;
	const Elf_Sym *srcsym;
	const Elf_Sym *dstsym;
	const void *srcaddr;
	const char *name;
	void *dstaddr;
	SymLook req;
	size_t size;
	int res;

	/*
	 * COPY relocs are invalid outside of the main program
	 */
	assert(dstobj->mainprog);

	relalim = (const Elf_Rela *)((char *)dstobj->rela +
	    dstobj->relasize);
	for (rela = dstobj->rela; rela < relalim; rela++) {
		if (ELF_R_TYPE(rela->r_info) != R_AARCH64_COPY)
			continue;

		dstaddr = (void *)(dstobj->relocbase + rela->r_offset);
		dstsym = dstobj->symtab + ELF_R_SYM(rela->r_info);
		name = dstobj->strtab + dstsym->st_name;
		size = dstsym->st_size;

		symlook_init(&req, name);
		req.ventry = fetch_ventry(dstobj, ELF_R_SYM(rela->r_info));
		req.flags = SYMLOOK_EARLY;

		for (srcobj = dstobj->next; srcobj != NULL;
		     srcobj = srcobj->next) {
			res = symlook_obj(&req, srcobj);
			if (res == 0) {
				srcsym = req.sym_out;
				defobj = req.defobj_out;
				break;
			}
		}
		if (srcobj == NULL) {
			_rtld_error(
"Undefined symbol \"%s\" referenced from COPY relocation in %s",
			    name, dstobj->path);
			return (-1);
		}

		srcaddr = (const void *)(defobj->relocbase + srcsym->st_value);
		memcpy(dstaddr, srcaddr, size);
	}

	return (0);
}

/*
 * Process the PLT relocations.
 */
int
reloc_plt(Obj_Entry *obj)
{
	RtldLockState lockstate;
	const Elf_Rela *relalim;
	const Elf_Rela *rela;
	const Elf_Sym *def;
	const Obj_Entry *defobj;

	relalim = (const Elf_Rela *)((char *)obj->pltrela + obj->pltrelasize);
	for (rela = obj->pltrela; rela < relalim; rela++) {
		Elf_Addr *where;

		where = (Elf_Addr *)(obj->relocbase + rela->r_offset);

		switch(ELF_R_TYPE(rela->r_info)) {
		case R_AARCH64_JUMP_SLOT:
			def = find_symdef(ELF_R_SYM(rela->r_info), obj,
			    &defobj, SYMLOOK_IN_PLT, NULL, &lockstate);
			if (def == NULL) {
				dbg("reloc_plt: sym not found");
				return (-1);
			}

			*where = (Elf_Addr)defobj->relocbase + def->st_value;
			break;
		case R_AARCH64_TLSDESC:
			if (ELF_R_SYM(rela->r_info) == 0) {
				where[0] = (Elf_Addr)_rtld_tlsdesc;
				where[1] = rela->r_addend;
			} else {
				_rtld_error("Unable to handle "
				    "R_AARCH64_TLSDESC with a symbol set");
			}
			break;
		default:
			_rtld_error("Unknown relocation type %u in PLT",
			    (unsigned int)ELF_R_TYPE(rela->r_info));
			return (-1);
		}
	}

	return (0);
}

/*
 * LD_BIND_NOW was set - force relocation for all jump slots
 */
int
reloc_jmpslots(Obj_Entry *obj, int flags, RtldLockState *lockstate)
{
	const Obj_Entry *defobj;
	const Elf_Rela *relalim;
	const Elf_Rela *rela;
	const Elf_Sym *def;

	relalim = (const Elf_Rela *)((char *)obj->pltrela + obj->pltrelasize);
	for (rela = obj->pltrela; rela < relalim; rela++) {
		Elf_Addr *where;

		where = (Elf_Addr *)(obj->relocbase + rela->r_offset);

		switch(ELF_R_TYPE(rela->r_info)) {
		case R_AARCH64_JUMP_SLOT:
			def = find_symdef(ELF_R_SYM(rela->r_info), obj,
			    &defobj, SYMLOOK_IN_PLT | flags, NULL, lockstate);
			if (def == NULL) {
				dbg("reloc_jmpslots: sym not found");
				return (-1);
			}

			*where = (Elf_Addr)(defobj->relocbase + def->st_value);
			break;
		default:
			_rtld_error("Unknown relocation type %x in jmpslot",
			    (unsigned int)ELF_R_TYPE(rela->r_info));
			return (-1);
		}
	}

	return (0);
}

int
reloc_iresolve(Obj_Entry *obj, struct Struct_RtldLockState *lockstate)
{

	/* XXX not implemented */
	return (0);
}

int
reloc_gnu_ifunc(Obj_Entry *obj, int flags,
   struct Struct_RtldLockState *lockstate)
{

	/* XXX not implemented */
	return (0);
}

Elf_Addr
reloc_jmpslot(Elf_Addr *where, Elf_Addr target, const Obj_Entry *defobj,
    const Obj_Entry *obj, const Elf_Rel *rel)
{

	assert(ELF_R_TYPE(rel->r_info) == R_ARM_JUMP_SLOT);

	if (*where != target)
		*where = target;

	return target;
}

/*
 * Process non-PLT relocations
 */
int
reloc_non_plt(Obj_Entry *obj, Obj_Entry *obj_rtld, int flags,
    RtldLockState *lockstate)
{
	const Obj_Entry *defobj;
	const Elf_Rela *relalim;
	const Elf_Rela *rela;
	const Elf_Sym *def;
	SymCache *cache;
	Elf_Addr *where;
	unsigned long symnum;

	/* The relocation for the dynamic loader has already been done. */
	if (obj == obj_rtld)
		return (0);
	if ((flags & SYMLOOK_IFUNC) != 0)
		/* XXX not implemented */
		return (0);

	/*
	 * The dynamic loader may be called from a thread, we have
	 * limited amounts of stack available so we cannot use alloca().
	 */
	cache = calloc(obj->dynsymcount, sizeof(SymCache));
	/* No need to check for NULL here */

	relalim = (const Elf_Rela *)((caddr_t)obj->rela + obj->relasize);
	for (rela = obj->rela; rela < relalim; rela++) {
		where = (Elf_Addr *)(obj->relocbase + rela->r_offset);
		symnum = ELF_R_SYM(rela->r_info);

		switch (ELF_R_TYPE(rela->r_info)) {
		case R_AARCH64_ABS64:
		case R_AARCH64_GLOB_DAT:
			def = find_symdef(symnum, obj, &defobj, flags, cache,
			    lockstate);
			if (def == NULL)
				return (-1);

			if (__predict_true(RELOC_ALIGNED_P(where))) {
				*where = (Elf_Addr)defobj->relocbase +
				    def->st_value;
			} else {
				rtld_printf(
				    "%s: TODO: Relocate unaligned %p %lu\n",
				    obj->path, where, ELF_R_TYPE(rela->r_info));
				return (-1);
			}
			break;
		case R_AARCH64_COPY:
			if (!obj->mainprog) {
				_rtld_error("%s: Unexpected R_AARCH64_COPY "
				    "relocation in shared library", obj->path);
				return (-1);
			}
			break;
		case R_AARCH64_RELATIVE:
			if (__predict_true(RELOC_ALIGNED_P(where))) {
				*where = (Elf_Addr)(obj->relocbase +
				    rela->r_addend);
			} else {
				rtld_printf(
				    "%s: TODO: Relocate unaligned %p %lu\n",
				    obj->path, where, ELF_R_TYPE(rela->r_info));
				return (-1);
			}
			break;
		default:
			rtld_printf("%s: Unhandled relocation %lu\n",
			    obj->path, ELF_R_TYPE(rela->r_info));
			return (-1);
			break;
		}
	}

	return (0);
}

void
allocate_initial_tls(Obj_Entry *objs)
{
	Elf_Addr **tp;

	/*
	* Fix the size of the static TLS block by using the maximum
	* offset allocated so far and adding a bit for dynamic modules to
	* use.
	*/

	tls_static_space = tls_last_offset + tls_last_size +
	    RTLD_STATIC_TLS_EXTRA;

	tp = (Elf_Addr **) allocate_tls(objs, NULL, TLS_TCB_SIZE, 16);

	asm volatile("msr	tpidr_el0, %0" : : "r"(tp));
}
