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

#include <sys/param.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/proc.h>
#include <sys/condvar.h>
#include <sys/systm.h>
#include <sys/bio.h>
#include <sys/kthread.h>
#include <sys/sysctl.h>

#include <geom/geom_disk.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/spibus/spi.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include "spibus_if.h"
#include "bus_if.h"

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

MALLOC_DECLARE(M_N25Q);
MALLOC_DEFINE(M_N25Q, "n25q buffers", "buffers for SPI N25Q Flash driver");

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

struct n25q_flash_data {
	uint32_t	id;
	uint32_t	page_size;
	uint32_t	page_count;
	uint32_t	flags;
};

static struct n25q_flash_data n25q_flash_devices[] = {
	{N25Q_TYPE_256, 0x100, 0x20000, N25Q_FLAG_FAST_READ}
};

static struct ofw_compat_data compat_data[] = {
	{"n25q256a", (uintptr_t)N25Q_TYPE_256_DEVSTR},
	{NULL, 0}
};

static int n25q_probe(device_t);
static int n25q_attach(device_t);
static int n25q_detach(device_t);
static void n25q_delayed_attach(void*);

static devclass_t n25q_devclass;

static device_method_t n25q_methods[] = {
	DEVMETHOD(device_probe, n25q_probe),
	DEVMETHOD(device_attach, n25q_attach),
	DEVMETHOD(device_detach, n25q_detach),

	DEVMETHOD_END
};

static driver_t n25q_driver = {
	"n25q",
	n25q_methods,
	sizeof(struct n25q_softc),
};

DRIVER_MODULE(n25q, spibus, n25q_driver, n25q_devclass, NULL, NULL);

static int
n25q_probe(device_t dev)
{
	const struct ofw_compat_data *ocd;

	ocd = ofw_bus_search_compatible(dev, compat_data);
	if (ocd->ocd_str == NULL)
		return (ENXIO);

	device_set_desc(dev, (const char *)ocd->ocd_data);
	return (BUS_PROBE_DEFAULT);
}

static int
n25q_attach(device_t dev)
{
	n25q_softc_t *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->initialized = false;
	mtx_init(&sc->mutex, "N25Q mutex", NULL, MTX_DEF);
	sc->config_intrhook.ich_func = n25q_delayed_attach;
	sc->config_intrhook.ich_arg = sc;
	if (config_intrhook_establish(&sc->config_intrhook) != 0) {
		device_printf(dev, "config_intrhook_establish failed\n");
		return (ENXIO);
	}

	return (0);
}

static int
n25q_detach(device_t dev)
{
	n25q_softc_t *sc;

	sc = device_get_softc(dev);
	free(sc->tx_buf, M_N25Q);
	disk_destroy(sc->disk);
	mtx_destroy(&sc->mutex);
	return (0);
}

static int
n25q_transfer(n25q_softc_t *sc, uint8_t *tx_cmd, uint8_t *rx_cmd,
    uint8_t *tx_data, uint8_t *rx_data, 
    unsigned int cmd_sz, unsigned int data_sz)
{
	device_t dev;
	struct spi_command cmd;

	dev = sc->dev;
	memset(rx_cmd, 0, cmd_sz);
	memset(rx_data, 0, data_sz);
	cmd.tx_cmd = tx_cmd;
	cmd.rx_cmd = rx_cmd;
	cmd.tx_cmd_sz = cmd_sz;
	cmd.rx_cmd_sz = cmd_sz;
	cmd.tx_data = tx_data;
	cmd.rx_data = rx_data;
	cmd.tx_data_sz = data_sz;
	cmd.rx_data_sz = data_sz;
	return (SPIBUS_TRANSFER(device_get_parent(dev), dev, &cmd));
}

static int
n25q_transfer_serial(n25q_softc_t *sc, uint8_t *tx_buf, unsigned int tx_len,
    uint8_t *rx_buf, unsigned int rx_len)
{
	int err;
	uint8_t *tx_data;
	uint8_t *rx_cmd;

	err = 0;
	tx_data = malloc(rx_len, M_N25Q, M_WAITOK);
	rx_cmd = malloc(tx_len, M_N25Q, M_WAITOK);
	/* These buffers are redundant, but needed by API */
	err = n25q_transfer(sc, tx_buf, rx_cmd, tx_data, rx_buf, tx_len,
	    rx_len);
	free(tx_data, M_N25Q);
	free(rx_cmd, M_N25Q);
	return (err);
}

static __inline int
n25q_read_reg(n25q_softc_t *sc, uint32_t instruction, uint8_t *rx_buf,
    unsigned int rx_len)
{
	uint8_t tx_buf;

	tx_buf = instruction;
	return (n25q_transfer_serial(sc, &tx_buf, 1, rx_buf, rx_len));
}

static uint32_t
n25q_get_read_opcode(n25q_softc_t *sc)
{

	if (((sc->flash_data->flags & N25Q_FLAG_FAST_READ) == 0) &&
	    sc->addr_bytes == 3) {
		return (N25Q_INS_READ);
	}
	else if (((sc->flash_data->flags & N25Q_FLAG_FAST_READ) == 0) &&
	    sc->addr_bytes == 4) {
		return (N25Q_INS_READ_4);
	}
	else if (((sc->flash_data->flags & N25Q_FLAG_FAST_READ) != 0) &&
	    sc->addr_bytes == 3) {
		return (N25Q_INS_FAST_READ);
	}
	else if (((sc->flash_data->flags & N25Q_FLAG_FAST_READ) != 0) &&
	    sc->addr_bytes == 4) {
		return (N25Q_INS_FAST_READ_4);
	}
	return (0);
}

static void
n25q_set_addr(n25q_softc_t *sc, uint8_t *tx_buf, uint32_t offset)
{

	if (sc->addr_bytes == 3) {
		tx_buf[1] = WRITE_THIRD_BYTE(offset);
		tx_buf[2] = WRITE_SECOND_BYTE(offset);
		tx_buf[3] = WRITE_FIRST_BYTE(offset);
	} else {
		tx_buf[1] = WRITE_FOURTH_BYTE(offset);
		tx_buf[2] = WRITE_THIRD_BYTE(offset);
		tx_buf[3] = WRITE_SECOND_BYTE(offset);
		tx_buf[4] = WRITE_FIRST_BYTE(offset);
	}
}

static void
n25q_task(void *arg)
{
	n25q_softc_t *sc;
	device_t dev;
	struct bio *bp;
	long bdone;
	uint32_t offset, to_write;
	int err;

	sc = arg;
	dev = sc->dev;
	memset(sc->tx_buf, 0, sc->tx_buf_len);
	sc->tx_buf[0] = n25q_get_read_opcode(sc);
	for (;;) {
		mtx_lock(&sc->mutex);
		do {
			bp = bioq_takefirst(&sc->bioq);
			if (bp == NULL) {
				mtx_sleep(sc, &sc->mutex, PRIBIO,
				    "n25q queue", 0);
			}
		} while (bp == NULL);
		mtx_unlock(&sc->mutex);
		if (bp->bio_cmd == BIO_READ)
			device_printf(dev, "Task - read.\n");
		else if (bp->bio_cmd == BIO_WRITE) {
			device_printf(dev, "Task - write.\n");
			biofinish(bp, NULL, ENOTSUP);
			continue;
		}
		bdone = 0;
		offset = bp->bio_offset;
		while (bdone < bp->bio_bcount) {
			n25q_set_addr(sc, sc->tx_buf, offset);
			to_write = (bp->bio_bcount - bdone) >
			    sc->flash_data->page_size ?
			    sc->flash_data->page_size :
			    (bp->bio_bcount - bdone);
			err = n25q_transfer_serial(sc, sc->tx_buf,
			    sc->tx_buf_len, &bp->bio_data[bdone], to_write);
			if (err != 0) {
				biofinish(bp, NULL, err);
				continue;
			}
			bdone += to_write;
			offset += to_write;
		}
		biodone(bp);
	}
}

static void
n25q_strategy(struct bio *bp)
{
	n25q_softc_t *sc;

	sc = bp->bio_disk->d_drv1;
	mtx_lock(&sc->mutex);
	bioq_disksort(&sc->bioq, bp);
	wakeup(sc);
	mtx_unlock(&sc->mutex);
}

static int
n25q_open(struct disk *disk __unused)
{

	return (0);
}

static int
n25q_close(struct disk *disk __unused)
{

	return (0);
}

static void
n25q_delayed_attach(void *arg)
{
	n25q_softc_t *sc;
	device_t dev;
	uint32_t id;
	uint8_t id_buf[3];
	int i;

	sc = arg;
	dev = sc->dev;
	sc->disk = disk_alloc();
	sc->flash_data = NULL;
	if (sc->disk == 0)
		return;

	n25q_read_reg(sc, N25Q_INS_RDID, id_buf, N25Q_INS_RDID_RXLEN);
	id = MAKE_ID(id_buf[0], id_buf[1], id_buf[2]);
	device_printf(dev, "Flash id is %x\n", id);
	for (i = 0; i < nitems(n25q_flash_devices); i++) {
		if (n25q_flash_devices[i].id == id)
			sc->flash_data = &n25q_flash_devices[i];
	}

	if (sc->flash_data == 0) {
		device_printf(dev, "Flash not attached!\n");
		config_intrhook_disestablish(&sc->config_intrhook);
		return;
	}
	sc->disk->d_name = "flash/n25q";
	sc->disk->d_unit = device_get_unit(dev);
	sc->disk->d_strategy = n25q_strategy;
	sc->disk->d_open = n25q_open;
	sc->disk->d_close = n25q_close;
	sc->disk->d_sectorsize = sc->flash_data->page_size;
	sc->disk->d_mediasize = sc->flash_data->page_size *
	    sc->flash_data->page_count;
	sc->disk->d_maxsize = DFLTPHYS;
	sc->disk->d_drv1 = sc;
	sc->disk->d_dump = NULL;
	sc->addr_bytes = sc->disk->d_mediasize > SIZE_128M ? 4 : 3;
	sc->tx_buf_len = sc->addr_bytes + (sc->flash_data->flags & 
	    N25Q_FLAG_FAST_READ ? N25Q_FAST_DUMMY : 0) + 1;
	sc->tx_buf = malloc(sc->tx_buf_len, M_N25Q, M_NOWAIT);
	kproc_create(&n25q_task, sc, &sc->p, 0, 0, "task: n25q flash");
	disk_create(sc->disk, DISK_VERSION);
	bioq_init(&sc->bioq);
	config_intrhook_disestablish(&sc->config_intrhook);
}
