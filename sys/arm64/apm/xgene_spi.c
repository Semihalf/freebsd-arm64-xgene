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
#include <sys/sysctl.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/spibus/spi.h>
#include <dev/spibus/spibusvar.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include "spibus_if.h"
#include "bus_if.h"

#define	SPI_DEVSTR		"X-Gene SPI integrated controller"
#define	SPI_TIMEOUT		(2 * hz)
#define	SPI_TIMEDWAIT		(10 * hz)
#define	SPI_CLOCK_BASE		3686400
#define	MAX_NUMBER_CHIPS	2

#define	xgene_reg_write(_sc, _off, _val)			\
	bus_write_4(_sc->mem_res, _off, _val)
#define	xgene_reg_read(_sc, _off)				\
	bus_read_4(_sc->mem_res, _off)
#define	xgene_fifo_write(_sc, _val)				\
	bus_write_4(_sc->mem_res, SPI_DR, _val);
#define	xgene_fifo_read(_sc)					\
	bus_read_4(_sc->mem_res, SPI_DR)

#define	SPI_CTRL0	0x0
#define	SPI_CTRL1	0x4
#define	SPI_SSIENR	0x8
#define	SPI_SER		0x10
#define	SPI_BAUDR	0x14
#define	SPI_TXFTLR	0x18
#define	SPI_RXFTLR	0x1c
#define	SPI_TXFLR	0x20
#define	SPI_RXFLR	0x24
#define	SPI_SR		0x28
#define	SPI_IMR		0x2c
#define	SPI_ISR		0x30
#define	SPI_RISR	0x34
#define	SPI_DR		0x60

#define	SPI_CTRL0_MASK		0xffff
#define	SPI_CTRL0_CFS		12
#define	SPI_CTRL0_CFS_MASK	0xf000
#define	SPI_CTRL0_CFS_MIN	1
#define	SPI_CTRL0_CFS_MAX	16
#define	SPI_CTRL0_SRL		11
#define	SPI_CTRL0_SRL_MASK	0x800
#define	SPI_CTRL0_TMOD		8
#define	SPI_CTRL0_TMOD_MASK	0x300
#define	SPI_CTRL0_SCPOL		7
#define	SPI_CTRL0_SCPOL_MASK	0x80
#define	SPI_CTRL0_SCPH		6
#define	SPI_CTRL0_SCPH_MASK	0x40
#define	SPI_CTRL0_DFS		0
#define	SPI_CTRL0_DFS_MASK	0x15
#define	SPI_CTRL0_DFS_MIN	4
#define	SPI_CTRL0_DFS_MAX	16
#define	SPI_CTRL0_TMOD_EEPROM	3

#define	SPI_CTRL1_NDF_MASK	0xffff

#define	SPI_SIZE_8		0x07

#define	SPI_SSIENR_EN		0
#define	SPI_SSIENR_EN_MASK	0x01

#define	SPI_SER_SER		0x0
#define	SPI_SER_SER_MASK	0x07
#define	SPI_SER_MASK		0x07

#define	SPI_BAUDR_SCKDV		0
#define	SPI_BAUDR_SCKDV_MASK	0xffff

#define	SPI_TXFTLR_TFT		0
#define	SPI_TXFTLR_TFT_MASK	0xff

#define	SPI_RXFTLR_RFT		0x0
#define	SPI_RXFTLR_RFT_MASK	0xff

#define	SPI_RXFLR_MASK		0x1ff

#define	SPI_FIFO_DATA_MASK	0xff

#define	SPI_RX_BUFFER_MASK	0xffff

#define	SPI_SR_MASK		0x7f
#define	SPI_SR_TFE_MASK		0x04
#define	SPI_SR_TFNF_MASK	0x02
#define	SPI_SR_BUSY_MASK	0x01
#define	SPI_SR_RFNE_MASK	0x08
#define	SPI_SR_RFF_MASK		0x10

#define	SPI_IMR_MASK		0x3f
#define	SPI_IMR_TXEIM		0
#define	SPI_IMR_TXEIM_MASK	0x01
#define	SPI_IMR_RXFIM		4
#define	SPI_IMR_RXFIM_MASK	0x10
#define	SPI_IMR_RXUIM		2
#define	SPI_IMR_RXUIM_MASK	0x04

#define	SPI_RISR_TXEIR_MASK	0x01
#define	SPI_RISR_TXOIR_MASK	0x02
#define	SPI_RISR_RXUIR_MASK	0x04
#define	SPI_RISR_RXOIR_MASK	0x08
#define	SPI_RISR_RXFIR_MASK	0x10
#define	SPI_RISR_MSTIR_MASK	0x20

#define	AHBC_SRST		0x0
#define	AHBC_CLKEN		0x8
#define	AHBC_CLK_CONFIG		0x10

#define	AHBC_SRST_MASK		0x3ff
#define	AHBC_SRST_SPI0		8
#define	AHBC_SRST_SPI1		7
#define	AHBC_SRST_SPI_MASK	0x180
#define	AHBC_CLKEN_SPI0		8
#define	AHBC_CLKEN_SPI1		7
#define	AHBC_CLKEN_SPI_MASK	0x180
#define	AHBC_CLKEN_APB		9
#define	AHBC_CLKEN_APB_MASK	0x200

/* FLAGS */

#define	SPI_FLAG_BUSY		0x01
#define	SPI_FLAG_RCV		0x08
#define	SPI_FLAG_TR		0x10
#define	SPI_FLAG_EEPROM		0x20
#define	SPI_FLAG_TR_RCV		0x40

#define	SPI_DEFAULT_SCKDV	450

#if defined(DEBUG)
#define debugf(dev, str, args...) do { device_printf(dev, str, ##args); } while (0)
#else
#define debugf(dev, str, args...) do { } while (0)
#endif

typedef struct xgene_spi_softc {
	device_t		dev;
	struct resource		*mem_res;
	struct resource 	*irq_res;
	struct resource		*ahbc_res;
	int 			mem_rid;
	int			irq_rid;
	int			ahbc_rid;
	struct mtx 		mutex;
	struct cv		cv_busy;
	void			*cookie;
	uint32_t		flags;
	int			read;
	int			written;
	int			tx_done;
	int			rx_done;
	struct spi_command	*cmd;
	void			(*tx)(struct xgene_spi_softc *);
	void			(*rx)(struct xgene_spi_softc *);
} xgene_spi_softc_t;

static int xgene_spi_probe(device_t);
static int xgene_spi_attach(device_t);
static int xgene_spi_detach(device_t);
static int xgene_spi_transfer(device_t, device_t, struct spi_command *);
static uint32_t xgene_reg_modify(xgene_spi_softc_t *, bus_size_t, uint32_t,
    uint32_t);
static void xgene_spi_sysctl_init(xgene_spi_softc_t *);
static void xgene_spi_intr(void *);
static uint32_t xgene_ahbc_reg_modify(xgene_spi_softc_t *, bus_size_t,
    uint32_t, uint32_t);
static phandle_t xgene_spi_get_node(device_t, device_t);

static devclass_t xgene_spi_devclass;

static device_method_t xgene_spi_methods[] = {
	DEVMETHOD(device_probe,		xgene_spi_probe),
	DEVMETHOD(device_attach,	xgene_spi_attach),
	DEVMETHOD(device_detach,	xgene_spi_detach),

	DEVMETHOD(spibus_transfer,	xgene_spi_transfer),

	DEVMETHOD(ofw_bus_get_node,	xgene_spi_get_node),

	DEVMETHOD_END
};

static driver_t xgene_spi_driver = {
	"spi",
	xgene_spi_methods,
	sizeof(xgene_spi_softc_t)
};

DRIVER_MODULE(xgene_spi, simplebus, xgene_spi_driver, xgene_spi_devclass, 0, 0);
MODULE_DEPEND(xgene_spi, spibus, 1, 1, 1);

static int
xgene_spi_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev) ||
	    !ofw_bus_is_compatible(dev, "apm,xgene-spi"))
		return (ENXIO);

	device_set_desc(dev, SPI_DEVSTR);

	return (BUS_PROBE_DEFAULT);
}

static int
xgene_spi_attach(device_t dev)
{
	xgene_spi_softc_t *sc;
	uint32_t val;
	int err;
	device_t spibus;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->mem_rid = 0;
	sc->irq_rid = 0;
	sc->flags = 0;

	mtx_init(&sc->mutex, "SPI mutex", NULL, MTX_DEF);
	cv_init(&sc->cv_busy, "SPI condition variable");

	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Failed to allocate memory\n");
		return (ENOMEM);
	}

	sc->ahbc_rid = 0;
	sc->ahbc_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->ahbc_rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->ahbc_res == NULL) {
		device_printf(dev, "Failed to setup AHBC\n");
		return (ENXIO);
	}

	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->irq_rid,
	    RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "Failed to get interrupt line\n");
		goto fail_irq;
	}

	err = bus_setup_intr(dev, sc->irq_res, INTR_MPSAFE | INTR_TYPE_MISC,
	    NULL, xgene_spi_intr, sc, &sc->cookie);
	if (err != 0) {
		device_printf(dev, "Failed to setup interrupt\n");
		goto fail;
	}

	mtx_lock(&sc->mutex);

	xgene_reg_modify(sc, SPI_SSIENR, SPI_SSIENR_EN_MASK, 0);
	if (device_get_unit(dev) == 0) {
		xgene_ahbc_reg_modify(sc, AHBC_SRST, AHBC_SRST_MASK, 0);
		xgene_ahbc_reg_modify(sc, AHBC_CLKEN, AHBC_CLKEN_SPI_MASK,
		    (1 << AHBC_CLKEN_SPI0) | (1 << AHBC_CLKEN_APB) |
		    (1 << AHBC_CLKEN_SPI1));
		xgene_ahbc_reg_modify(sc, AHBC_SRST, AHBC_SRST_MASK,
		    (1 << AHBC_SRST_SPI0) | (1 << AHBC_SRST_SPI1));
		xgene_ahbc_reg_modify(sc, AHBC_SRST, AHBC_SRST_MASK, 0);
	}

	val = (SPI_SIZE_8 << SPI_CTRL0_CFS) | (SPI_SIZE_8 << SPI_CTRL0_DFS);
	sc->flags &= ~(SPI_FLAG_RCV | SPI_FLAG_TR | SPI_FLAG_EEPROM);
	if (device_get_unit(dev) == 0) {
		/* Boot spi0 in EEPROM mode, so as to allow Flash initialization */
		val |= (SPI_CTRL0_TMOD_EEPROM << SPI_CTRL0_TMOD);
		val |= (1 << SPI_CTRL0_SCPH);
		val |= (1 << SPI_CTRL0_SCPOL);
		sc->flags |= SPI_FLAG_EEPROM;
	} else
		sc->flags |= SPI_FLAG_TR_RCV;

	xgene_reg_modify(sc, SPI_CTRL0, SPI_CTRL0_MASK, val);
	xgene_reg_modify(sc, SPI_BAUDR, SPI_BAUDR_SCKDV_MASK,
	    (SPI_DEFAULT_SCKDV << SPI_BAUDR_SCKDV));
	xgene_reg_modify(sc, SPI_TXFTLR, SPI_TXFTLR_TFT_MASK, 0);
	xgene_reg_modify(sc, SPI_RXFTLR, SPI_RXFTLR_RFT_MASK, 0);
	xgene_reg_modify(sc, SPI_IMR, SPI_IMR_MASK, SPI_IMR_MASK);
	xgene_reg_modify(sc, SPI_SER, SPI_SER_SER_MASK, 0);
	
	mtx_unlock(&sc->mutex);
	
	xgene_spi_sysctl_init(sc);
	spibus = device_add_child(dev, "spibus", -1);
	err = bus_generic_attach(dev);
	if (err == 0)
		bus_generic_attach(spibus);
	else
		goto fail;

	return (0);
fail:
	bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->irq_res);
fail_irq:
	bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem_res);
	return (ENOMEM);
}

static int
xgene_spi_detach(device_t dev)
{
	xgene_spi_softc_t *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mutex);
	bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem_res);
	bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->mem_res);
	mtx_unlock(&sc->mutex);
	mtx_destroy(&sc->mutex);

	return (0);
}

static __inline uint32_t
xgene_reg_modify(xgene_spi_softc_t *sc, bus_size_t off, uint32_t mask,
    uint32_t val)
{
	uint32_t tmp, save;

	tmp = xgene_reg_read(sc, off);
	save = tmp;
	tmp &= ~mask;
	tmp |= val;
	xgene_reg_write(sc, off, tmp);
	return (save);
}

static __inline uint32_t 
xgene_ahbc_reg_modify(xgene_spi_softc_t *sc, bus_size_t off, uint32_t mask,
    uint32_t val)
{
	uint32_t tmp;

	tmp = bus_read_4(sc->ahbc_res, off);
	tmp &= ~mask;
	tmp |= val;
	bus_write_4(sc->ahbc_res, off, tmp);
	return (tmp);
}

static void
xgene_spi_tx(xgene_spi_softc_t *sc)
{
	bool not_full;
	uint8_t *data;

	data = sc->cmd->tx_cmd;
	not_full = (xgene_reg_read(sc, SPI_SR) & SPI_SR_TFNF_MASK);
	while (not_full && (sc->written < sc->cmd->tx_cmd_sz)) {
		debugf(sc->dev, "Writing 0x%x from tx_cmd\n",
		    data[sc->written] & SPI_FIFO_DATA_MASK);
		xgene_fifo_write(sc, data[sc->written] & SPI_FIFO_DATA_MASK);
		sc->written++;
		debugf(sc->dev, "xgene_spi_tx, sc->written increased to %i\n",
		    sc->written);
		not_full = (xgene_reg_read(sc, SPI_SR) & SPI_SR_TFNF_MASK);
	}

	data = sc->cmd->tx_data;
	while (not_full && 
	    (sc->written < sc->cmd->tx_cmd_sz + sc->cmd->tx_data_sz)) {
		int i = sc->written - sc->cmd->tx_cmd_sz;
		KASSERT(i >= 0 && i < sc->cmd->tx_data_sz,
		    ("Wrong SPI assertion\n"));
		debugf(sc->dev, "Writing 0x%x from tx_data\n", data[i] &
		    SPI_FIFO_DATA_MASK);
		xgene_fifo_write(sc, data[i] & SPI_FIFO_DATA_MASK);
		sc->written++;
		debugf(sc->dev, "xgene_spi_tx, sc->written increased to %i\n",
		    sc->written);
		not_full = (xgene_reg_read(sc, SPI_SR) & SPI_SR_TFNF_MASK);
	}

	if (sc->written == (sc->cmd->tx_cmd_sz + sc->cmd->tx_data_sz)) {
		debugf(sc->dev, "TXEIM interrupt masked\n");
		sc->tx_done = 1;
		xgene_reg_modify(sc, SPI_IMR, SPI_IMR_TXEIM_MASK, 0);
	}
}

static void
xgene_spi_rx(xgene_spi_softc_t *sc)
{
	bool not_empty;
	uint8_t *data;

	data = sc->cmd->rx_cmd;
	not_empty = (xgene_reg_read(sc, SPI_SR) & SPI_SR_RFNE_MASK);
	while (not_empty && (sc->read < sc->cmd->rx_cmd_sz)) {
		data[sc->read] = xgene_fifo_read(sc) & SPI_FIFO_DATA_MASK;
		sc->read++;
		debugf(sc->dev, "xgene_spi_rx, sc->read increased to %i\n",
		    sc->read);
		not_empty = (xgene_reg_read(sc, SPI_SR) & SPI_SR_RFNE_MASK);
	}

	data = sc->cmd->rx_data;
	while (not_empty &&
	    (sc->read < sc->cmd->rx_cmd_sz + sc->cmd->rx_data_sz)) {
		int i = sc->read - sc->cmd->rx_cmd_sz;
		KASSERT((i >= 0) && (i < sc->cmd->rx_data_sz),
		    ("Wrong SPI assertion\n"));
		data[i] = xgene_fifo_read(sc) & SPI_FIFO_DATA_MASK;
		sc->read++;
		debugf(sc->dev, "xgene_spi_rx, sc->read increased to %i\n",
		    sc->read);
		not_empty = (xgene_reg_read(sc, SPI_SR) & SPI_SR_RFNE_MASK);
	}

	if (sc->read == (sc->cmd->rx_cmd_sz + sc->cmd->rx_data_sz)) {
		debugf(sc->dev, "RXFIM interrupt masked\n");
		sc->rx_done = 1;
		xgene_reg_modify(sc, SPI_IMR, SPI_IMR_RXFIM_MASK, 0);
	}
}

/*
 * In EEPROM mode, cmd->rx_cmd and cmd->rx_data are dropped.
 * Only tx->cmd is sent to hardware, after that rx_data buffer
 * is filled with received data.
 */
static void
xgene_spi_eeprom_tx(xgene_spi_softc_t *sc)
{
	bool not_full;
	uint8_t *data;

	data = sc->cmd->tx_cmd;
	not_full = (xgene_reg_read(sc, SPI_SR) & SPI_SR_TFNF_MASK);
	while (not_full && (sc->written < sc->cmd->tx_cmd_sz)) {
		debugf(sc->dev, "Writing 0x%x from tx_cmd\n",
		    (data[sc->written] & SPI_FIFO_DATA_MASK));
		xgene_fifo_write(sc, (data[sc->written] & SPI_FIFO_DATA_MASK));
		sc->written++;
		debugf(sc->dev, "xgene_spi_tx, sc->written increased to %i\n",
		    sc->written);
		not_full = (xgene_reg_read(sc, SPI_SR) & SPI_SR_TFNF_MASK);
	}

	if (sc->written == sc->cmd->tx_cmd_sz) {
		debugf(sc->dev, "TXEIM interrupt masked\n");
		sc->tx_done = 1;
		xgene_reg_modify(sc, SPI_IMR, SPI_IMR_TXEIM_MASK, 0);
	}
}

static void
xgene_spi_eeprom_rx(xgene_spi_softc_t *sc)
{
	bool not_empty;
	uint8_t *data;

	data = sc->cmd->rx_data;
	not_empty = (xgene_reg_read(sc, SPI_SR) & SPI_SR_RFNE_MASK);
	while (not_empty && (sc->read < sc->cmd->rx_data_sz)) {
		data[sc->read] = xgene_fifo_read(sc) & SPI_FIFO_DATA_MASK;
		printf("%c", data[sc->read]);
		debugf(sc->dev, "Read 0x%x from rx_fifo\n",
		    data[sc->read] & SPI_FIFO_DATA_MASK);
		sc->read++;
		debugf(sc->dev, "xgene_spi_rx, sc->read increased to %i\n",
		    sc->read);
		not_empty = (xgene_reg_read(sc, SPI_SR) & SPI_SR_RFNE_MASK);
	}

	if (sc->read == sc->cmd->rx_data_sz) {
		debugf(sc->dev, "RXFIM interrupt masked\n");
		sc->rx_done = 1;
		xgene_reg_modify(sc, SPI_IMR, SPI_IMR_RXFIM_MASK, 0);
	}
}

static void
xgene_spi_intr(void *arg)
{
	xgene_spi_softc_t *sc;
	uint32_t to_read, to_write;

	sc = arg;
	mtx_lock(&sc->mutex);
	if (sc->tx_done == 0)
		sc->tx(sc);
	if (sc->rx_done == 0)
		sc->rx(sc);
	debugf(sc->dev, "sc->read is %i, sc->written is %i, rx_cmd_sz is %i,"
	    "rx_data_sz is %i, tx_cmd_sz is %i, tx_data_sz is %i\n",
	    sc->read, sc->written, sc->cmd->rx_cmd_sz,
	    sc->cmd->rx_data_sz, sc->cmd->tx_cmd_sz,
	    sc->cmd->tx_data_sz);

	if ((sc->flags & SPI_FLAG_EEPROM) != 0) {
		to_read = sc->cmd->rx_data_sz;
		to_write = sc->cmd->tx_cmd_sz;
	} else {
		to_read = sc->cmd->rx_data_sz + sc->cmd->rx_cmd_sz;
		to_write = sc->cmd->tx_data_sz + sc->cmd->tx_cmd_sz;
	}

	if (((sc->read == to_read) && (sc->written == to_write)) != 0) {
		sc->flags &= ~SPI_FLAG_BUSY;
		mtx_unlock(&sc->mutex);
		cv_signal(&sc->cv_busy);
		return;
	}
	debugf(sc->dev, "SR is 0x%x\n",
	    (xgene_reg_read(sc, SPI_SR) & SPI_SR_MASK));
	mtx_unlock(&sc->mutex);
}

static int
xgene_spi_transfer(device_t dev, device_t child, struct spi_command *cmd)
{
	xgene_spi_softc_t *sc;
	uint32_t ctrl1_save;
	unsigned int cs;
	int err;

	sc = device_get_softc(dev);
	debugf(dev, "xgene_spi_transfer\n");
	mtx_lock(&sc->mutex);
	if ((sc->flags & SPI_FLAG_BUSY) != 0) {
		if ((cv_timedwait(&sc->cv_busy, &sc->mutex,
		    SPI_TIMEOUT)) != 0) {
			mtx_unlock(&sc->mutex);
			device_printf(dev, "Transfer timeout.\n");
			return (ETIMEDOUT);
		}
	}

	sc->tx_done = 0;
	sc->rx_done = 0;
	sc->read = 0;
	sc->written = 0;
	sc->tx = xgene_spi_tx;
	sc->rx = xgene_spi_rx;
	if ((sc->flags & SPI_FLAG_RCV) != 0) {
		if ((cmd->rx_cmd_sz != cmd->rx_data_sz) ||
		    (cmd->rx_cmd_sz != 0)) {
			device_printf(dev, "RX request in no-RX mode.\n");
			goto out_einval;
		}
		sc->tx_done = 1;
		xgene_reg_modify(sc, SPI_IMR, SPI_IMR_MASK,
		    (1 << SPI_IMR_RXFIM));
	} else if ((sc->flags & SPI_FLAG_EEPROM) != 0) {
		if ((cmd->tx_cmd_sz != cmd->rx_cmd_sz) || 
		    (cmd->tx_data_sz != cmd->rx_data_sz)) {
			device_printf(dev, "Not equal RX/TX sizes\n");
			goto out_einval;
		}
		if (cmd->rx_data_sz > SPI_RX_BUFFER_MASK) {
			device_printf(dev, "RX buffer too large.\n");
			goto out_einval;
		}
		sc->tx = xgene_spi_eeprom_tx;
		sc->rx = xgene_spi_eeprom_rx;
		ctrl1_save = xgene_reg_modify(sc, SPI_CTRL1, 
		    SPI_CTRL1_NDF_MASK, ((cmd->rx_data_sz - 1) &
		    SPI_CTRL1_NDF_MASK));
		debugf(dev, "Setting NDF to %i\n", 
		    xgene_reg_read(sc, SPI_CTRL1) & SPI_CTRL1_NDF_MASK);
		xgene_reg_modify(sc, SPI_IMR, SPI_IMR_MASK, 
		    (1 << SPI_IMR_TXEIM) | (1 << SPI_IMR_RXFIM));
	} else if ((sc->flags & SPI_FLAG_TR_RCV) != 0) {
		if ((cmd->tx_cmd_sz != cmd->rx_cmd_sz) || 
		    (cmd->tx_data_sz != cmd->rx_data_sz)) {
			debugf(dev, "Not equal RX/TX sizes\n");
			goto out_einval;
		}
		xgene_reg_modify(sc, SPI_IMR, SPI_IMR_MASK,
		    (1 << SPI_IMR_TXEIM) | (1 << SPI_IMR_RXFIM));
	} else if ((sc->flags & SPI_FLAG_TR) != 0) {
		if ((cmd->tx_cmd_sz != cmd->tx_data_sz) ||
		    (cmd->tx_cmd_sz != 0)) {
			debugf(dev, "RX request in no-RX mode.\n");
			goto out_einval;
		}
		sc->rx_done = 1;
		xgene_reg_modify(sc, SPI_IMR, SPI_IMR_MASK,
		    (1 << SPI_IMR_TXEIM));
	} else
		goto out_einval;

	sc->flags |= SPI_FLAG_BUSY;
	xgene_reg_modify(sc, SPI_IMR, SPI_IMR_MASK, (1 << SPI_IMR_TXEIM) |
	    (1 << SPI_IMR_RXFIM));
	xgene_reg_modify(sc, SPI_SSIENR, 0, (1 << SPI_SSIENR_EN));
	spibus_get_cs(child, &cs);
	if (cs > MAX_NUMBER_CHIPS) {
		device_printf(dev, "Invalid CS.\n");
		goto out_einval;
	}

	sc->cmd = cmd;
	debugf(dev, "Setting cs - interrupt should get trigerred.\n");
	xgene_reg_modify(sc, SPI_SER, 0, (1 << cs));
	err = cv_timedwait(&sc->cv_busy, &sc->mutex, SPI_TIMEDWAIT);
	if (err != 0) {
		device_printf(dev, "Transfer timeout.\n");
		sc->flags &= ~SPI_FLAG_BUSY;
	}

	xgene_reg_modify(sc, SPI_SER, SPI_SER_MASK, 0);
	xgene_reg_modify(sc, SPI_SSIENR, SPI_SSIENR_EN_MASK, 0);
	if ((sc->flags & SPI_FLAG_EEPROM) != 0) {
		xgene_reg_modify(sc, SPI_CTRL1, SPI_CTRL1_NDF_MASK,
		    ctrl1_save);
	}

	debugf(dev, "Transfer end.\n");
	mtx_unlock(&sc->mutex);
	return (err);

out_einval:
	mtx_unlock(&sc->mutex);
	return (EINVAL);
}

static int
xgene_spi_clock_proc(SYSCTL_HANDLER_ARGS)
{
	xgene_spi_softc_t *sc;
	uint32_t sckdv;
	int err;

	sc = arg1;
	mtx_lock(&sc->mutex);
	sckdv = xgene_reg_read(sc, SPI_BAUDR) & SPI_BAUDR_SCKDV_MASK;
	mtx_unlock(&sc->mutex);
	/* If BAUDR is set to 0, clock is disabled */
	sckdv = (sckdv == 0) ? 0 : (SPI_CLOCK_BASE / sckdv);
	err = sysctl_handle_int(oidp, &sckdv, sizeof(sckdv), req);
	if (err != 0)
		return (err);

	sckdv = (sckdv == 0) ? 0 : (SPI_CLOCK_BASE / sckdv);
	sckdv &= SPI_BAUDR_SCKDV_MASK;
	mtx_lock(&sc->mutex);
	xgene_reg_modify(sc, SPI_BAUDR, SPI_BAUDR_SCKDV_MASK, sckdv);
	mtx_unlock(&sc->mutex);
	return (0);
}

static int
xgene_spi_val_proc(SYSCTL_HANDLER_ARGS, bus_size_t off, uint32_t mask,
    int field, uint32_t *new)
{
	xgene_spi_softc_t *sc;
	uint32_t val, err, reg;

	sc = arg1;
	mtx_lock(&sc->mutex);
	val = xgene_reg_read(sc, off);
	mtx_unlock(&sc->mutex);
	val &= mask;
	val = val >> field;
	err = sysctl_handle_int(oidp, &val, sizeof(val), req);
	if (err != 0)
		return (err);

	val = val << field;
	val &= mask;
	mtx_lock(&sc->mutex);
	reg = xgene_reg_read(sc, off);
	reg &= ~mask;
	reg |= val;
	xgene_reg_write(sc, off, reg);
	if (new != NULL)
		*new = (val >> field);

	mtx_unlock(&sc->mutex);
	return (0);
}

static int
xgene_spi_scpol_proc(SYSCTL_HANDLER_ARGS)
{

	return (xgene_spi_val_proc(oidp, arg1, arg2, req, SPI_CTRL0,
	    SPI_CTRL0_SCPOL_MASK, SPI_CTRL0_SCPOL, 0));
}

static int
xgene_spi_scph_proc(SYSCTL_HANDLER_ARGS)
{ 

	return (xgene_spi_val_proc(oidp, arg1, arg2, req, SPI_CTRL0,
	    SPI_CTRL0_SCPH_MASK, SPI_CTRL0_SCPH, 0));
}

static int
xgene_spi_srl_proc(SYSCTL_HANDLER_ARGS)
{

	return (xgene_spi_val_proc(oidp, arg1, arg2, req, SPI_CTRL0,
	    SPI_CTRL0_SRL_MASK, SPI_CTRL0_SRL, 0));
}

static int
xgene_spi_tmode_proc(SYSCTL_HANDLER_ARGS)
{
	xgene_spi_softc_t *sc;
	int err, new_val;

	sc = arg1;
	err = xgene_spi_val_proc(oidp, arg1, arg2, req, SPI_CTRL0,
	    SPI_CTRL0_TMOD_MASK, SPI_CTRL0_TMOD, &new_val);
	if (err != 0)
		return (err);

	mtx_lock(&sc->mutex);
	sc->flags &= ~(SPI_FLAG_RCV | SPI_FLAG_TR | SPI_FLAG_EEPROM |
	    SPI_FLAG_TR_RCV);
	switch (new_val) {
	case 3:
		sc->flags |= SPI_FLAG_EEPROM;
		break;
	case 2:
		sc->flags |= SPI_FLAG_RCV;
		break;
	case 1:
		sc->flags |= SPI_FLAG_TR;
		break;
	case 0:
		sc->flags |= SPI_FLAG_TR_RCV;
		break;
	default:
		return (EINVAL);
		break;
	}

	mtx_unlock(&sc->mutex);
	return (0);
}

static int
xgene_spi_xfs_proc(SYSCTL_HANDLER_ARGS, uint32_t off, uint32_t mask,
    int min, int max)
{
	xgene_spi_softc_t *sc;
	uint32_t xfs;
	int err;

	sc = arg1;
	mtx_lock(&sc->mutex);
	xfs = (xgene_reg_read(sc, SPI_CTRL0) & mask);
	mtx_unlock(&sc->mutex);
	xfs >>= off;
	xfs++;
	err = sysctl_handle_int(oidp, &xfs, sizeof(xfs), req);
	if (err != 0)
		return (err);
	
	if ((xfs < min) || (xfs > max)) {
		device_printf(sc->dev, "Value must follow %i <= val <= %i.\n",
		    min, max);
		return (EINVAL);
	}

	xfs--;
	xfs <<= off;
	mtx_lock(&sc->mutex);
	xgene_reg_modify(sc, SPI_CTRL0, mask, xfs);
	mtx_unlock(&sc->mutex);
	return (0);
}

static int
xgene_spi_cfs_proc(SYSCTL_HANDLER_ARGS)
{

	return (xgene_spi_xfs_proc(oidp, arg1, arg2, req, SPI_CTRL0_CFS,
	    SPI_CTRL0_CFS_MASK, SPI_CTRL0_CFS_MIN, SPI_CTRL0_CFS_MAX));
}

static int
xgene_spi_dfs_proc(SYSCTL_HANDLER_ARGS)
{

	return (xgene_spi_xfs_proc(oidp, arg1, arg2, req, SPI_CTRL0_DFS,
	    SPI_CTRL0_DFS_MASK, SPI_CTRL0_DFS_MIN, SPI_CTRL0_DFS_MAX));
}

static void
xgene_spi_sysctl_init(xgene_spi_softc_t *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	struct sysctl_oid_list *children;

	ctx = device_get_sysctl_ctx(sc->dev);
	tree = device_get_sysctl_tree(sc->dev);
	children = SYSCTL_CHILDREN(tree);
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "clock", CTLFLAG_RW |
	    CTLTYPE_UINT, sc, sizeof(*sc), xgene_spi_clock_proc, "IU",
	    "SPI clock frequency");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "cpol", CTLFLAG_RW |
	    CTLTYPE_UINT, sc, sizeof(*sc), xgene_spi_scpol_proc, "IU",
	    "SPI clock polarity");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "cpha", CTLFLAG_RW |
	    CTLTYPE_UINT, sc, sizeof(*sc), xgene_spi_scph_proc, "IU",
	    "SPI clock phase");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "transmit_mode", CTLFLAG_RW |
	    CTLTYPE_UINT, sc, sizeof(*sc), xgene_spi_tmode_proc, "IU",
	    "SPI transmit mode");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "shift_register_loop",
	    CTLFLAG_RW | CTLTYPE_UINT, sc, sizeof(*sc), xgene_spi_srl_proc,
	    "IU", "SPI shift register loop");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "data_frame_size",
	    CTLFLAG_RW | CTLTYPE_UINT, sc, sizeof(*sc), xgene_spi_dfs_proc,
	    "IU", "SPI data frame size");
	SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "control_frame_size",
	    CTLFLAG_RW | CTLTYPE_UINT, sc, sizeof(*sc), xgene_spi_cfs_proc,
	    "IU", "SPI control frame size");
}

static __inline phandle_t
xgene_spi_get_node(device_t bus, device_t dev)
{

	return (ofw_bus_get_node(bus));
}
