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
#ifndef __XGENE_ENET_COMPAT_H__
#define	__XGENE_ENET_COMPAT_H__

#include <sys/cdefs.h>
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
#include <sys/taskqueue.h>
#include <sys/bitstring.h>

#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_dl.h>

#include <machine/bus.h>

typedef	uint8_t		u8;
typedef	uint16_t	u16;
typedef uint32_t	u32;
typedef uint64_t	u64;

typedef uint64_t	__le64;

typedef bus_addr_t	dma_addr_t;

#define __iomem
#define	udelay		DELAY

#define	netdev_err	device_printf

#define	GENMASK(last, first)				\
({							\
	uint32_t mask;					\
							\
	bit_nset((bitstr_t *)&mask, first, last);	\
	mask;						\
})

#define	GENMASK_ULL(last, first)			\
({							\
	uint64_t mask;					\
							\
	bit_nset((bitstr_t *)&mask, first, last);	\
	mask;						\
})

#define	BIT(n)			(1U << (n))

#define	SPEED_10	10
#define	SPEED_100	100

#define	netdev_priv(dev)				\
({							\
	struct xge_softc *sc;				\
							\
	sc = device_get_softc(dev);			\
	&sc->pdata;					\
})

#endif
