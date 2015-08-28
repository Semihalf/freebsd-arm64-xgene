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
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/resource.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>

#include "iicbus_if.h"

#include "xgene_i2c.h"

static int xgene_i2c_start(device_t, u_char, int);
static int xgene_i2c_stop(device_t);
static int xgene_i2c_reset(device_t, u_char, u_char, u_char *);
static int xgene_i2c_read(device_t, char *, int, int *, int, int);
static int xgene_i2c_write(device_t, const char *, int, int *, int);
static int xgene_i2c_repeated_start(device_t, u_char, int);

static device_probe_t xgene_i2c_probe;
static device_attach_t xgene_i2c_attach;
static device_detach_t xgene_i2c_detach;

static device_method_t i2c_methods[] = {
		DEVMETHOD(device_attach,				xgene_i2c_attach),
		DEVMETHOD(device_detach,				xgene_i2c_detach),
		DEVMETHOD(device_probe,					xgene_i2c_probe),

		DEVMETHOD(iicbus_callback,				iicbus_null_callback),
		DEVMETHOD(iicbus_repeated_start,		xgene_i2c_repeated_start),
		DEVMETHOD(iicbus_start,					xgene_i2c_start),
		DEVMETHOD(iicbus_stop,					xgene_i2c_stop),
		DEVMETHOD(iicbus_reset,					xgene_i2c_reset),
		DEVMETHOD(iicbus_read,					xgene_i2c_read),
		DEVMETHOD(iicbus_write,					xgene_i2c_write),
		DEVMETHOD(iicbus_transfer,				iicbus_transfer_gen),

		DEVMETHOD(bus_print_child, 				bus_generic_print_child),

		DEVMETHOD_END
};

static driver_t i2c_driver = {
		"i2c",
		i2c_methods,
		sizeof(struct xgene_i2c_softc),
};

static devclass_t i2c_devclass;

DRIVER_MODULE(i2c, simplebus, i2c_driver, i2c_devclass, 0, 0);
DRIVER_MODULE(iicbus, i2c, iicbus_driver, iicbus_devclass, 0, 0);
MODULE_DEPEND(i2c, iicbus, 1, 1, 1);

static __inline int
xgene_i2c_reg_read(i2c_softc_t *sc, uint32_t reg)
{

	return (bus_read_4(sc->mem_res, reg));
}

static __inline void
xgene_i2c_reg_write(i2c_softc_t *sc, uint32_t reg, uint32_t data)
{

	bus_write_4(sc->mem_res, reg, data);
}

static int
xgene_i2c_intr(void *xsc __unused)
{

	/*  Interrupts are not needed while in polling mode */
	return (0);
}

static boolean_t
xgene_i2c_wait_bus_not_busy(i2c_softc_t *sc)
{
	uint32_t status;
	ssize_t timeout = XGENE_I2C_TIMEOUT;

	status = xgene_i2c_reg_read(sc, XGENE_I2C_STATUS);

	for (; (status & XGENE_I2C_STATUS_ACTIVITY) != 0;
	    status = xgene_i2c_reg_read(sc, XGENE_I2C_STATUS)) {
		if (timeout < 0)
			return (FALSE);

		timeout--;
		DELAY(XGENE_I2C_DELAY_IF_BUSY);
	}

	return (TRUE);
}

static int
xgene_i2c_wait_for_stop(i2c_softc_t *sc)
{
	uint32_t raw_intr;
	int status;
	ssize_t timeout;

	status = IIC_NOERR;
	timeout = XGENE_I2C_STOP_TIMEOUT;
	raw_intr = xgene_i2c_reg_read(sc, XGENE_I2C_RAW_INTR);

	/* Wait for stop condition on the bus */
	for (; (raw_intr & XGENE_I2C_RAW_INTR_STOP) == 0;
			raw_intr = xgene_i2c_reg_read(sc, XGENE_I2C_RAW_INTR)) {
		if (timeout < 0) {
			status = IIC_EBUSERR;
			break;
		}

		DELAY(i2c_conf.sclk_t);
		timeout--;
	}
	return (status);
}

static int
xgene_i2c_wait_for_flag(i2c_softc_t *sc)
{
	uint32_t status;
	ssize_t timeout = XGENE_I2C_STAT_WAIT_RET;

	/* Wait for start sequence on the bus */
	do {
		status = (xgene_i2c_reg_read(sc, XGENE_I2C_RAW_INTR));
		if (timeout < 0)
			return (IIC_ETIMEOUT);
		timeout--;
		DELAY(i2c_conf.sclk_t);
	} while ((status & XGENE_I2C_RAW_INTR_START) == 0);

	timeout = XGENE_I2C_TX_RX_TIMEOUT;

	/* Check for errors */
	do {
		status = xgene_i2c_reg_read(sc, XGENE_I2C_RAW_INTR);
		if (timeout < 0)
			return (IIC_NOERR);
		timeout--;
		DELAY(i2c_conf.sclk_t);
	} while ((status & XGENE_I2C_RAW_INTR_TX_ABRT) == 0);

	status = xgene_i2c_reg_read(sc, XGENE_I2C_TX_ABRT_SRC);

	/* Check source of error */
	if ((status & XGENE_I2C_TX_ABRT_SRC_NOACK7) != 0) {
		status = IIC_ENOACK;
		goto finish;
	}

	if ((status & XGENE_I2C_TX_ABRT_SRC_SBTACK) != 0) {
		status = IIC_EBUSERR;
		goto finish;
	}

	if ((status & XGENE_I2C_TX_ABRT_SRC_ARBLOST) != 0) {
		status = IIC_EBUSERR;
		goto finish;
	}

	status = IIC_EBUSERR;

finish:
	/* Clear interrupts */
	xgene_i2c_reg_read(sc, XGENE_I2C_CLR_I);

	return (status);
}

static void
xgene_i2c_init(i2c_softc_t *sc)
{
	uint32_t config, i2c_en, rx_buffer, tx_buffer, param, hcnt, lcnt;

	mtx_lock(&sc->mutex);

	/* Disable the controller */
	i2c_en = xgene_i2c_reg_read(sc, XGENE_I2C_ENABLE);
	if ((i2c_en & XGENE_I2C_ENABLE_EN) != 0) {
		xgene_i2c_reg_write(sc, XGENE_I2C_ENABLE,
		    (i2c_en & (~XGENE_I2C_ENABLE_EN)));
	}

	/* Config buffer lenghts */
	param = xgene_i2c_reg_read(sc, XGENE_I2C_COMP_P1);
	rx_buffer = XGENE_I2C_RX_LENGTH(param);
	tx_buffer = XGENE_I2C_TX_LENGTH(param);
	xgene_i2c_reg_write(sc, XGENE_I2C_RX_TL, rx_buffer);
	xgene_i2c_reg_write(sc, XGENE_I2C_TX_TL, tx_buffer);

	/* Set speed clock SCL */
	/* Standard speed clock mode */
	hcnt = XGENE_I2C_SCL_HCNT(i2c_conf.i2c_clk,
			xgene_i2c_scl[I2C_SS][I2C_SCL_HIGH],
			xgene_i2c_scl[I2C_SS][I2C_SCL_TF]);
	lcnt = XGENE_I2C_SCL_LCNT(i2c_conf.i2c_clk,
			xgene_i2c_scl[I2C_SS][I2C_SCL_LOW],
			xgene_i2c_scl[I2C_SS][I2C_SCL_TF]);
	xgene_i2c_reg_write(sc, XGENE_I2C_SS_SCL_H, hcnt);
	xgene_i2c_reg_write(sc, XGENE_I2C_SS_SCL_L, lcnt);

	/* Fast speed clock mode */
	hcnt = XGENE_I2C_SCL_HCNT(i2c_conf.i2c_clk,
			xgene_i2c_scl[I2C_FS][I2C_SCL_HIGH],
			xgene_i2c_scl[I2C_FS][I2C_SCL_TF]);
	lcnt = XGENE_I2C_SCL_LCNT(i2c_conf.i2c_clk,
			xgene_i2c_scl[I2C_FS][I2C_SCL_LOW],
			xgene_i2c_scl[I2C_FS][I2C_SCL_TF]);
	xgene_i2c_reg_write(sc, XGENE_I2C_FS_SCL_H, hcnt);
	xgene_i2c_reg_write(sc, XGENE_I2C_FS_SCL_L, lcnt);

	/* High speed clock mode */
	hcnt = XGENE_I2C_SCL_HCNT(i2c_conf.i2c_clk,
			xgene_i2c_scl[I2C_HS][I2C_SCL_HIGH],
			xgene_i2c_scl[I2C_HS][I2C_SCL_TF]);
	lcnt = XGENE_I2C_SCL_LCNT(i2c_conf.i2c_clk,
			xgene_i2c_scl[I2C_HS][I2C_SCL_LOW],
			xgene_i2c_scl[I2C_HS][I2C_SCL_TF]);
	xgene_i2c_reg_write(sc, XGENE_I2C_HS_SCL_H, hcnt);
	xgene_i2c_reg_write(sc, XGENE_I2C_HS_SCL_L, lcnt);

	/* Disable all interrupts */
	xgene_i2c_reg_write(sc, XGENE_I2C_INTR_MASK, XGENE_I2C_INTR_MASK_DIS);

	/* Configure the I2C master */
	config = XGENE_I2C_CON_MM | XGENE_I2C_CON_10BAS |
			XGENE_I2C_CON_REST_EN | XGENE_I2C_CON_SD;

	/* Set speed mode */
	if ((XGENE_I2C_BUS_SPEED >= 0) &&
	    (XGENE_I2C_BUS_SPEED <= XGENE_I2C_MAX_STANDARD_SPEED)) {
		config |= XGENE_I2C_CON_MSM_S;
		i2c_conf.sclk_t = xgene_i2c_sclk_t[I2C_SS];
	}

	else if ((XGENE_I2C_BUS_SPEED > XGENE_I2C_MAX_STANDARD_SPEED) &&
	    (XGENE_I2C_BUS_SPEED <= XGENE_I2C_MAX_FAST_SPEED)) {
		config |= XGENE_I2C_CON_MSM_F;
		i2c_conf.sclk_t = xgene_i2c_sclk_t[I2C_FS];
	}

	xgene_i2c_reg_write(sc, XGENE_I2C_CON, config);

	/* Enable the controller */
	i2c_en = xgene_i2c_reg_read(sc, XGENE_I2C_ENABLE);
	if ((i2c_en & XGENE_I2C_ENABLE_EN) == 0) {
		xgene_i2c_reg_write(sc, XGENE_I2C_ENABLE,
		    (i2c_en | XGENE_I2C_ENABLE_EN));
	}

	mtx_unlock(&sc->mutex);
}

static int
xgene_i2c_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "snps,designware-i2c"))
		return (ENXIO);

	device_set_desc(dev, "APM X-Gene I2C Controller");

	return (BUS_PROBE_DEFAULT);
}

static int
xgene_i2c_attach(device_t dev)
{
	int err, rid;
	phandle_t node = ofw_bus_get_node(dev);
	i2c_softc_t *sc = device_get_softc(dev);

	sc->dev = dev;
	/* Allocate memory */
	rid = XGENE_I2C_REG_RID;
	sc->mem_res = bus_alloc_resource_any(dev,
	    SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Unable to allocate resource\n");
		xgene_i2c_detach(dev);
		return (ENXIO);
	}

	/* Get bus properties */
	if (OF_getprop(node, "clock-frequency", (void *)&i2c_conf.i2c_clk,
	    sizeof(i2c_conf.i2c_clk)) <= 0) {
		device_printf(dev, "Unknown clock frequency\n");
		xgene_i2c_detach(dev);
		return (ENXIO);
	}
	
	i2c_conf.i2c_clk /= XGENE_I2C_CLOCK_PRESC; 	/* I2C clock in kHz */

	/* Allocate IRQ */
	rid = XGENE_I2C_REG_RID;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
		   RF_ACTIVE | RF_SHAREABLE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "Could not allocate irq\n");
		xgene_i2c_detach(dev);
		return (ENXIO);
	}

	/* Setup interrupt */
	err = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
		NULL, (driver_intr_t *) xgene_i2c_intr, sc, &sc->intr_hdl);
	if (err) {
		device_printf(dev, "Could not setup irq, %d\n", err);
		sc->intr_hdl = NULL;
		xgene_i2c_detach(dev);
		return (err);
	}

	sc->iicbus = device_add_child(dev, "iicbus", -1);
	if (sc->iicbus == NULL) {
		device_printf(dev, "Unable to add child\n");
		xgene_i2c_detach(dev);
		return (ENXIO);
	}

	/* Initialize mutex */
	mtx_init(&sc->mutex, device_get_nameunit(dev), "i2c", MTX_DEF);
	/* Initialize controller */
	xgene_i2c_init(sc);
	/* Attach the iicbus*/
	bus_generic_attach(dev);

	return (IIC_NOERR);
}

static int
xgene_i2c_detach(device_t dev)
{
	i2c_softc_t *sc = device_get_softc(dev);

	if ((sc->irq_res != NULL) && (sc->intr_hdl != NULL)) {
		bus_teardown_intr(dev, sc->irq_res, sc->intr_hdl);
		sc->intr_hdl = NULL;
	}

	if (sc->mem_res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY, rman_get_rid(sc->mem_res),
		    sc->mem_res);
		sc->mem_res = NULL;
	}

	if (sc->irq_res != NULL) {
		bus_release_resource(dev, SYS_RES_IRQ, rman_get_rid(sc->irq_res),
		    sc->irq_res);
		sc->irq_res = NULL;
	}

	return (0);
}

static int
xgene_i2c_repeated_start(device_t dev __unused, u_char slave __unused,
    int time_out __unused)
{

	/* The operation is not available for this controller */
	return (IIC_ENOTSUPP);
}

static int
xgene_i2c_start(device_t dev, u_char slave, int timeout __unused)
{
	uint32_t status, i2c_en;
	int error;
	i2c_softc_t *sc  = device_get_softc(dev);

	mtx_lock(&sc->mutex);

	/* Check if bus is busy */
	if (xgene_i2c_wait_bus_not_busy(sc) == FALSE) {
		mtx_unlock(&sc->mutex);
		xgene_i2c_stop(dev);
		return (IIC_EBUSBSY);
	}

	/* Set start condition */

	/* Disable the controller */
	i2c_en = xgene_i2c_reg_read(sc, XGENE_I2C_ENABLE);
	if ((i2c_en & XGENE_I2C_ENABLE_EN) != 0) {
		xgene_i2c_reg_write(sc, XGENE_I2C_ENABLE,
		    (i2c_en & (~XGENE_I2C_ENABLE_EN)));
	}

	/* Ensure that mode settings are correct */
	status = xgene_i2c_reg_read(sc, XGENE_I2C_CON);
	if (((status & XGENE_I2C_CON_MM) && (status & XGENE_I2C_CON_SD)) == 0) {
		xgene_i2c_reg_write(sc, XGENE_I2C_CON, status |
				XGENE_I2C_CON_MM | XGENE_I2C_CON_SD);
	}

	/* Write target address and call attributes */
	xgene_i2c_reg_write(sc, XGENE_I2C_TAR, XGENE_I2C_TAR_SPECIAL |
	    XGENE_I2C_TAR_START | GET_SLAVE_ADDRESS(slave));
	/* Clear all interrupts */
	xgene_i2c_reg_read(sc, XGENE_I2C_CLR_I);
	/* Enable the controller */
	i2c_en = xgene_i2c_reg_read(sc, XGENE_I2C_ENABLE);
	if ((i2c_en & XGENE_I2C_ENABLE_EN) == 0) {
		xgene_i2c_reg_write(sc, XGENE_I2C_ENABLE,
		    (i2c_en | XGENE_I2C_ENABLE_EN));
	}
	/* Write data direction */
	if (CHECK_CMD_READ(slave) != 0)
		xgene_i2c_reg_write(sc, XGENE_I2C_DATA_CMD, READ_DIRECTION);  /*read*/
	else
		xgene_i2c_reg_write(sc, XGENE_I2C_DATA_CMD, WRITE_DIRECTION); /*write*/

	DELAY(XGENE_I2C_DELAY_AFT_START);

	error = xgene_i2c_wait_for_flag(sc);
	mtx_unlock(&sc->mutex);

	if (error)
		return (error);

	return (IIC_NOERR);
}

static int
xgene_i2c_stop(device_t dev)
{
	/*
	 * Controller handles this procedure automatically after each
	 * successful and unsuccessful transfer.
	 */
	i2c_softc_t *sc = device_get_softc(dev);
	if (xgene_i2c_reg_read(sc, XGENE_I2C_RXFLR) == 0) {
		if ((xgene_i2c_reg_read(sc, XGENE_I2C_RAW_INTR) &
		    XGENE_I2C_RAW_INTR_STOP) != 0)
			return (IIC_NOERR);
	}

	return (IIC_EBUSERR);
}

static int
xgene_i2c_reset(device_t dev, u_char speed, u_char addr,
    u_char *oldadr __unused)
{
	uint32_t state, i2c_en;
	uint8_t baud_rate;
	ssize_t timeout;
	i2c_softc_t *sc = device_get_softc(dev);

	/* Longest transmission period - 4 bytes */
	timeout = XGENE_I2C_TX_RX_TIMEOUT_RST;

	switch (speed) {
	case IIC_SLOW:
		baud_rate = XGENE_I2C_CON_MSM_S;
		break;
	case IIC_FAST:
		baud_rate = XGENE_I2C_CON_MSM_F;
		break;
	case IIC_UNKNOWN:
	default:
		baud_rate = XGENE_I2C_CON_MSM_S;
		break;
	}

	mtx_lock(&sc->mutex);
	/* If controller is inactive carelessly apply configuration */
	if ((xgene_i2c_reg_read(sc, XGENE_I2C_STATUS) &
	    XGENE_I2C_STATUS_ACTIVITY) != 0) {
		while ((xgene_i2c_reg_read(sc, XGENE_I2C_RAW_INTR) &
		    XGENE_I2C_RAW_INTR_STOP) == 0) {
			if (timeout < 0) {
				mtx_unlock(&sc->mutex);
				return (IIC_EBUSBSY);
			}

			DELAY(i2c_conf.sclk_t);
			timeout--;
		}
	}
	/* Disable the controller */
	i2c_en = xgene_i2c_reg_read(sc, XGENE_I2C_ENABLE);
	if ((i2c_en & XGENE_I2C_ENABLE_EN) != 0) {
		xgene_i2c_reg_write(sc, XGENE_I2C_ENABLE,
		    (i2c_en & (~XGENE_I2C_ENABLE_EN)));
	}
	/* Setup new baudrate */
	state = xgene_i2c_reg_read(sc, XGENE_I2C_CON);
	xgene_i2c_reg_write(sc, XGENE_I2C_CON, state | baud_rate);
	/* Clear all interrupts */
	xgene_i2c_reg_read(sc, XGENE_I2C_CLR_I);
	/* Write target address and call attributes */
	xgene_i2c_reg_write(sc, XGENE_I2C_TAR, XGENE_I2C_TAR_SPECIAL |
	    XGENE_I2C_TAR_START | GET_SLAVE_ADDRESS(addr));
	/* Enable the controller */
	i2c_en = xgene_i2c_reg_read(sc, XGENE_I2C_ENABLE);
	if ((i2c_en & XGENE_I2C_ENABLE_EN) == 0) {
		xgene_i2c_reg_write(sc, XGENE_I2C_ENABLE,
		    (i2c_en | XGENE_I2C_ENABLE_EN));
	}

	mtx_unlock(&sc->mutex);

	return (IIC_NOERR);
}

static int
xgene_i2c_read(device_t dev, char *buf, int len, int *read, int last __unused,
    int delay __unused)
{
	uint32_t reg;
	int status;
	ssize_t timeout;
	i2c_softc_t *sc = device_get_softc(dev);

	*read = 0;
	mtx_lock(&sc->mutex);

	while (*read < len) {
		timeout = XGENE_I2C_TX_RX_TIMEOUT;
		/* Wait for data in RX buffer */
		do {
			reg = xgene_i2c_reg_read(sc, XGENE_I2C_RXFLR);
			if (((xgene_i2c_reg_read(sc, XGENE_I2C_RAW_INTR) &
			    XGENE_I2C_RAW_INTR_TX_ABRT) != 0) || timeout < 0) {
				mtx_unlock(&sc->mutex);
				return (IIC_EBUSERR);
			}
			DELAY(i2c_conf.sclk_t);
			timeout--;
		} while (reg == 0);

		(*read)++;
	}
	*buf++ = (xgene_i2c_reg_read(sc, XGENE_I2C_DATA_CMD) & MASK_CMD_DATA);
	status = xgene_i2c_wait_for_stop(sc);
	mtx_unlock(&sc->mutex);

	return (status);
}

static int
xgene_i2c_write(device_t dev, const char *buf, int len, int *sent,
    int time_out __unused)
{
	int status, error;
	ssize_t timeout;
	i2c_softc_t *sc = device_get_softc(dev);

	*sent = 0;
	timeout = XGENE_I2C_TX_RX_TIMEOUT;
	mtx_lock(&sc->mutex);

	while (*sent < len) {
		/* 
		 * Check status register and send data if the transmit 
		 * buffer is at or below the threshold value  
		 */		
		status = (xgene_i2c_reg_read(sc, XGENE_I2C_RAW_INTR) &
		    MASK_STATUS_DATA);
		if ((status & XGENE_I2C_RAW_INTR_TX_EMPT) != 0) {
			xgene_i2c_reg_write(sc, XGENE_I2C_DATA_CMD,
			    ((*buf++) & MASK_CMD_DATA));
		} else
			DELAY(i2c_conf.sclk_t);
		(*sent)++;
	}
	if ((xgene_i2c_reg_read(sc, XGENE_I2C_RAW_INTR) &
	    XGENE_I2C_RAW_INTR_TX_ABRT) != 0) {
		mtx_unlock(&sc->mutex);
		return (IIC_EBUSERR);
	}

	error = xgene_i2c_wait_for_stop(sc);
	mtx_unlock(&sc->mutex);

	return (error);
}
