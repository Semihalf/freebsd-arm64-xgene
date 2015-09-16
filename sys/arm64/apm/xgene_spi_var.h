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

DECLARE_CLASS(xgene_spi_driver);

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

#endif
