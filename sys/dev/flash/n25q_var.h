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

#ifndef __XGENE_SPI_H_
#define __XGENE_SPI_H_

/* TYPES */
#define N25Q_TYPE_256		0x20BA19
#define N25Q_TYPE_256_DEVSTR	"Micron N25Q256A NOR Flash"

/* COMMANDS */
#define N25Q_INS_RDID		0x9f
#define N25Q_INS_RDID_RXLEN	3
#define N25Q_INS_RDSR		0x05
#define N25Q_INS_RDSR_RXLEN	1
#define N25Q_INS_READ		0x3
#define N25Q_INS_FAST_READ	0xb
#define N25Q_INS_READ_4		0x13
#define N25Q_INS_FAST_READ_4	0xc

/* FLAGS */
#define N25Q_FLAG_FAST_READ	0x1

#define SIZE_128M		0x2000000
#define N25Q_FAST_DUMMY		8

#define WRITE_FIRST_BYTE(x)	((x) & 0xFF)
#define WRITE_SECOND_BYTE(x)	(((x) >> 8) & 0xFF)
#define WRITE_THIRD_BYTE(x)	(((x) >> 16) & 0xFF)
#define WRITE_FOURTH_BYTE(x)	(((x) >> 24) & 0xFF)

#define MAKE_ID(x,y,z)		(((x) << 16) | ((y) << 8) | (z))

DECLARE_CLASS(n25q_driver);

struct n25q_flash_data {
	uint32_t	id;
	uint32_t	page_size;
	uint32_t	page_count;
	uint32_t	flags;
};

static struct n25q_flash_data n25q_flash_devices[] = {
	{N25Q_TYPE_256, 0x100, 0x20000, N25Q_FLAG_FAST_READ}
};

typedef struct n25q_softc {
	device_t		dev;
	struct bio_queue_head	bioq;
	struct disk		*disk;
	struct mtx		mutex;
	struct intr_config_hook	config_intrhook;
	struct proc		*p;
	bool			initialized;
	struct n25q_flash_data	*flash_data;
	unsigned int		addr_bytes;
	uint8_t			*tx_buf;
	unsigned int		tx_buf_len;
} n25q_softc_t;

#endif
