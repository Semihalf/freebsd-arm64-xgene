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
#include <sys/rman.h>
#include <sys/endian.h>
#include <sys/bus.h>
#include <sys/sockio.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include "xgene_clk_var.h"

/*
 * XXX
 * Since X-Gene clocks share the same SoC registers and we don't reserve
 * these resources for the particular driver instance it is required to
 * lock each register on every access for safety.
 */
#define csr_reg_read(sc, off)	\
    bus_space_read_4(&memmap_bus, (sc)->clk_csr_reg_h, (off))

#define	div_reg_read(sc, off)	\
    bus_space_read_4(&memmap_bus, (sc)->clk_div_reg_h,, (off))

#define	csr_reg_write(sc, off, val)	\
    bus_space_write_4(&memmap_bus, (sc)->clk_csr_reg_h, (off), (val));

#define	div_reg_write(sc, off, val)	\
    bus_space_write_4(&memmap_bus, (sc)->clk_div_reg_h, (off), (val));

/* TODO: global clks list */

int
xgene_clk_attach(device_t dev)
{

	return (0);
}

int
xgene_clk_enable(device_t dev)
{
	struct xgene_clk_softc *sc;

	sc = device_get_softc(dev);
	if (sc->clk_initialized)
		return (0);


	sc->clk_initialized = TRUE;

	return (0);
}

int
xgene_clk_disable(device_t dev)
{

	return (0);
}
