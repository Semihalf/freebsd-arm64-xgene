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
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/rman.h>


#include <arm64/include/bus.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ahci/ahci.h>

#define AHCI_DEVSTR "APM XGENE AHCI"


/* PORTCFG register of SATA Control and Status Register block */
#define	PORTCFG										0x000000a4
#define		PORTCFG_POSTSCAL_MASK						0xfff00000
#define 	PORTCFG_PORTSCAL_SHIFT						20
#define		PORTCFG_PRESCAL_MASK						0x000ff000
#define		PORTCFG_PRESCAL_SHIFT						12
#define		PORTCFG_CISE								0x00000100
#define		PORTCFG_PORTADDR_MASK						0x0000003f

#define PORTCFG_PORTS 2
#define PORTCFG_PORT_OFFSET 2

#define PORTCFG_PORTADDR(channel) ((channel) + PORTCFG_PORT_OFFSET)
#define PORTCFG_PORTADDR_SET(dst, channel) 							\
		(((dst) & ~PORTCFG_PORTADDR_MASK) | 						\
		 ((channel) + PORTCFG_PORT_OFFSET))


/* PORTPHY1CFG register of SATA Control and Status Register block */
#define	PORTPHY1CFG									0x000000a8
#define 	PORTPHY1CFG_FIXSPDEN						0x80000000
#define		PORTPHY1CFG_CMDSLMBREN						0x40000000
#define		PORTPHY1CFG_OOBSEL							0x20000000
#define		PORTPHY1CFG_SERSLMBRSEL						0x10000000
#define		PORTPHY1CFG_PCCLK							0x08000000
#define		PORTPHY1CFG_BISTPATTNA						0x04000000
#define		PORTPHY1CFG_BISTCLRERR						0x02000000
#define		PORTPHY1CFG_BISTPATTEN						0x01000000
#define		PORTPHY1CFG_BISTPATTSEL_MASK				0x00e00000
#define		PORTPHY1CFG_BISTPATTSEL_LBP						0x00000000
#define		PORTPHY1CFG_BISTPATTSEL_LFTP					0x00200000
#define		PORTPHY1CFG_BISTPATTSEL_MFTP					0x00400000
#define		PORTPHY1CFG_BISTPATTSEL_HFTP					0x00600000
#define		PORTPHY1CFG_BISTPATTSEL_PRBS					0x00800000
#define		PORTPHY1CFG_BISTPATTSEL_DEFAULT					0x00e00000
#define		PORTPHY1CFG_FRCPHYRDY						0x00100000
#define		PORTPHY1CFG_ALIGNWAIT_MASK					0x0001ffff

#define	PORTPHY1CFG_BISTPATTSEL_GET(data)							\
		((((data) & PORTPHY1CFG_BISTPATTSEL_MASK)) < PORTPHY1CFG_BISTPATTSE_DEFAULT ? \
		   (data) & PORTPHY1CFG_BISTPATSEL_MASK : 					\
		   PORTPHY1CFG_BISTPATTSEL_DEFAULT)

#define	PORTPHY1CFG_BISTPATTSEL_SET(dst, bist) 						\
		(((dst) & ~PORTPHY1CFG_BISTPATSEL_MASK) | bist & PORTPHY1CFG_BISTPATSEL_MASK)

#define PORTPHY1CFG_ALIGNWAIT(wait) ((wait) & PORTPHY1CFG_ALIGNWAIT_MASK)
#define PORTPHY1CFG_ALIGNWAIT_SET(dst, wait) 						\
		((dst) & ~PORTPHY1CFG_ALIGNWAIT_MASK | PORTPHY1CFG_ALIGNWAIT(wait))


/* PORTPHY2CFG register of SATA Control and Status Register block */
#define	PORTPHY2CFG									0x000000ac
#define		PORTPHY2CFG_COMINITNEG_MASK					0xff000000
#define		PORTPHY2CFG_COMINITNEG_SHIFT 				24
#define		PORTPHY2CFG_COMINITGAPNOM_MASK				0x00ff0000
#define		PORTPHY2CFG_COMINITGAPNOM_SHIFT				16
#define		PORTPHY2CFG_COMINITGAPMAX_MASK				0x0000ff00
#define		PORTPHY2CFG_COMINITGAPMAX_SHIFT 			8
#define		PORTPHY2CFG_COMINITGAPMIN_MASK				0x000000ff
#define 	PORTPHY2CFG_COMINITGAPMIN_SHIFT 			0

#define PORTPHY2CFG_INIT(neg, nom, max, min) \
		(((neg) << PORTPHY2CFG_COMINITNEG_SHIFT) & PORTPHY2CFG_COMINITNEG_MASK | \
		 ((nom) << PORTPHY2CFG_COMINITGAPNOM_SHIFT) & PORTPHY2CFG_COMINITGAPNOM_MASK | \
		 ((max) << PORTPHY2CFG_COMINITGAPMAX_SHIFT) & PORTPHY2CFG_COMINITGAPMAX_MASK | \
		 ((min) << PORTPHY2CFG_COMINITGAPMIN_SHIFT) & PORTPHY2CFG_COMINITGAPMIN_MASK)

#define PORTPHY2CFG_SET(dst, neg, nom, max, min) \
		((((dst) & ~(PORTPHY2CFG_COMINITNEG__MASK | PORTPHY2CFG_COMINITGAPNOM_MASK | \
		  PORTPHY2CFG_COMINITGAPMAX_MASK | PORTPHY2CFG_COMINITGAPMIN_MASK)) | \
		 PORTPHY2CFG_INIT(neg, nom, max, min))


/* PORTPHY3CFG register of SATA Control and Status Register block */
#define	PORTPHY3CFG									0x000000b0
#define		PORTPHY3CFG_COMWAKENEG_MASK					0xff000000
#define		PORTPHY3CFG_COMWAKENEG_SHIFT				24
#define		PORTPHY3CFG_COMWAKEGAPNOM_MASK				0x00ff0000
#define		PORTPHY3CFG_COMWAKEGAPNOM_SHIFT				16
#define		PORTPHY3CFG_COMWAKEGAPMAX_MASK				0x0000ff00
#define		PORTPHY3CFG_COMWAKEGAPMAX_SHIFT				8
#define		PORTPHY3CFG_COMWAKEGAPMIN_MASK				0x000000ff
#define		PORTPHY3CFG_COMWAKEGAPMIN_SHIFT				0

#define PORTPHY3CFG_INIT(neg, nom, max, min) \
		(((neg) << PORTPHY3CFG_COMWAKENEG_SHIFT) & PORTPHY3CFG_COMWAKENEG_MASK | \
		 ((nom) << PORTPHY3CFG_COMWAKEGAPNOM_SHIFT) & PORTPHY3CFG_COMWAKEGAPNOM_MASK | \
		 ((max) << PORTPHY3CFG_COMWAKEGAPMAX_SHIFT) & PORTPHY3CFG_COMWAKEGAPMAX_MASK | \
		 ((min) << PORTPHY3CFG_COMWAKEGAPMIN_SHIFT) & PORTPHY3CFG_COMWAKEGAPMIN_MASK)

#define PORTPHY3CFG_SET(dst, neg, nom, max, min) \
		((((dst) & ~(PORTPHY3CFG_COMWAKENEG_MASK | PORTPHY3CFG_COMWAKEGAPNOM_MASK | \
		  PORTPHY3CFG_COMWAKEGAPMAX_MASK | PORTPHY3CFG_COMWAKEGAPMIN_MASK)) | \
		 PORTPHY3CFG_INIT(neg, nom, max, min))


/* PORTPHY4CFG register of SATA Control and Status Register block */
#define	PORTPHY4CFG									0x000000b4
#define		PORTPHY4CFG_COMBURSTNOM_MASK				0x0000ff00
#define		PORTPHY4CFG_COMBURSTNOM_SHIFT				8
#define		PORTPHY4CFG_COMBURSTMAX_MASK				0x000000ff
#define		PORTPHY4CFG_COMBURSTMAX_SHIFT 				0

#define PORTPHY4CFG_INIT(nom, max) \
		(((nom) << PORTPHY4CFG_COMBURSTNOM_SHIFT) & PORTPHY4CFG_COMBURSTNOM_MASK | \
		 ((max) << PORTPHY4CFG_COMBURSTMAX_SHIFT) & PORTPHY4CFG_COMBURSTMAX_MASK)

#define PORTPHY4CFG_SET(dst, nom, max) \
		((((dst) & 0x0000ffff) &											\
		  ~(PORTPHY4CFG_COMBURSTNOM_MASK | PORTPHY4CFG_COMBURSTMAX_MASK)) | 	\
		 PORTPHY4CFG_INIT(nom, max))


/* PORTPHY5CFG register of SATA Control and Status Register block */
#define	PORTPHY5CFG									0x000000b8
#define		PORTPHY5CFG_RTCHG_MASK						0xfff00000
#define 	PORTPHY5CFG_RTCHG_SHIFT						20
#define		PORTPHY5CFG_RTRYINT_MASK					0x000fffff

#define PORPTHY5CFG_INIT(rtchg, rtryint) \
		(((rtchg) << PORTPHY5CFG_RTCHG_SHIFT) & PORTPHY5CFG_RTCHG_MASK | \
		 ((rtryint) << PORTPHY5CFG_RTRYINT_SHIFT) & PORTPHY5CFG_RTRYINT_MASK)

#define PORTPHY5CFG_SET(dst, rtchg, rtryint) \
		(((dst) & ~(PORTPHY5CFG_RTCHG_MASK | PORTPHY5CFG_RTRINT_MASK)) | \
		 PORTPHY5CFG_INIT(rtchg, rtryint))

#define PORTPHY5CFG_RTCHG_SET(dst, rtchg) \
		((((dst) & ~PORTPHY5CFG_RTCHG_MASK) | ((rtchg) << PORTPHY5CFG_RTCHG_SHIFT) & PORTPHY5CFG_RTCHG_MASK))


/* PORTAXICFG register of SATA Control and Status Register block */
#define	PORTAXICFG									0x000000bc
#define		PORTAXICFG_ADDR_OVR							0x02000000
#define		PORTAXICFG_EN_CONTEXT						0x01000000
#define		PORTAXICFG_OUTTRANS_MASK					0x00f00000
#define		PORTAXICFG_OUTTRANS_SHIFT					20
#define 	PORTAXICFG_DATAMARID_MASK 					0x000f0000
#define		PORTAXICFG_DATAMARID_SHIFT					16
#define		PORTAXICFG_NDATAMARID_MASK					0x0000f000
#define		PORTAXICFG_NDATAMARID_SHIFT					12
#define		PORTAXICFG_DATAMAWID_MASK					0x00000f00
#define		PORTAXICFG_DATAMAWID_SHIFT					8
#define		PORTAXICFG_NDATAMAWID_MASK					0x000000f0
#define 	PORTAXICFG_NDATAMAWID_SHIFT					4
#define		PORTAXICFG_AXIDATAWID_MASK					0x00000003
#define		PORTAXICFG_AXIDATAWID_32						0x00000000
#define		PORTAXICFG_AXIDATAWID_64						0x00000001
#define		PORTAXICFG_AXIDATAWID_128						0x00000002

#define PORTAXICFG_DATAMARID(marid) \
		(((marid) << PORTAXICFG_DATAMARID_SHIFT) & PORTAXICFG_DATAMARID_MASK)
#define PORTAXICFG_DATAMARID_SET(dst, marid) \
		((((dst) & PORTAXICFG_DATAMARID_MASK) | PORTAXICFG_DATAMARID(marid))

#define PORTAXICFG_NDATAMARID(marid) \
		(((marid) << PORTAXICFG_NDATAMARID_SHIFT) & PORTAXICFG_NDATAMARID_MASK)
#define PORTAXICFG_NDATAMARID_SET(dst, marid) \
		((((dst) & PORTAXICFG_NDATAMARID_MASK) | PORTAXICFG_NDATAMARID(marid))

#define PORTAXICFG_DATAMAWID(mawid) \
		(((mawid) << PORTAXICFG_DATAMAWID_SHIFT) & PORTAXICFG_DATAMAWID_MASK)
#define PORTAXICFG_DATAMAWID_SET(dst, mawid) \
		((((dst) & PORTAXICFG_DATAMAWID_MASK) | PORTAXICFG_DATAMAWID(mawid))

#define PORTAXICFG_NDATAMAWID(mawid) \
		(((mawid) << PORTAXICFG_NDATAMAWID_SHIFT) & PORTAXICFG_NDATAMAWID_MASK)
#define PORTAXICFG_NDATAMAWID_SET(dst, mawid) \
		((((dst) & PORTAXICFG_NDATAMAWID_MASK) | PORTAXICFG_NDATAMAWID(mawid))

#define PORTAXICFG_OUTTRANS(outtrans) \
		(((outtrans) << PORTAXICFG_OUTTRANS_SHIFT) & PORTAXICFG_OUTTRANS_MASK)
#define PORTAXICFG_OUTTRANS_SET(dst, outtrans) \
		(((dst) & PORTAXICFG_OUTTRANS_MASK) | PORTAXICFG_OUTTRANS(outtrans))


/* PORTTRANSCDG register of SATA Control and Status Register block */
#define PORTRANSCFG									0x000000c8
/* Reserved but peculiarly set to 1 */
#define		PORTRANSCFG_UNKNOWN							0x08000000
#define 	PORTRANSCFG_RXWATERMARK_MASK				0x0000003f
#define		PORTRANSSTAT								0x000000cc
#define		PORTRANSSTAT_TXSM							0x0000ff00
#define		PORTRANSSTAT_TXSM_IDLE							0
#define		PORTRANSSTAT_TXSM_RESET							1
#define		PORTRANSSTAT_TXSM_WAITRESETCOMPLETE				2
#define		PORTRANSSTAT_TXSM_NONDATAINITIAL				3
#define		PORTRANSSTAT_TXSM_NONDATAABORT					4
#define		PORTRANSSTAT_TXSM_NONDATASEND					5
#define		PORTRANSSTAT_TXSM_NONDATACOMP					6
#define 	PORTRANSSTAT_TXSM_NONDATAOK						7
#define 	PORTRANSSTAT_TXSM_NONDATANOTOK					8
#define 	PORTRANSSTAT_TXSM_DATAHEADER					9
#define 	PORTRANSSTAT_TXSM_DATAHEADEROUT					10
#define 	PORTRANSSTAT_TXSM_DATASEND						11
#define 	PORTRANSSTAT_TXSM_DATACOMP						12
#define 	PORTRANSSTAT_TXSM_DATANOTOK						13
#define 	PORTRANSSTAT_TXSM_DATAOK						14
#define 	PORTRANSSTAT_TXSM_CMDRESET						15
#define 	PORTRANSSTAT_TXSM_CMDRESETCOMP					16
#define 	PORTRANSSTAT_TXSM_CMDRESETOVERFLOW				17

#define	PORTRANSCFG_RXWATERMARK_SET(dst, val) 			\
		(((dst) & ~PORTRANSCFG_RXWATERMARK_MASK) |		\
		 ((val) & PORTRANSCFG_RXWATERMARK_MASK))


/* Registers in diagnostic Control and Status Register block */
#define CFG_MEM_RAM_SHUTDOWN 0x070
#define BLOCK_MEM_RDY 0x074


/* Registers in core Control and Status Register block */
#define SLVRDERRATTRIBUTES      				0x00000000
#define SLVWRERRATTRIBUTES						0x00000004
#define MSTRDERRATTRIBUTES						0x00000008
#define MSTWRERRATTRIBUTES						0x0000000c
#define BUSCTLREG 								0x00000014
#define 	BUSCTLREG_MSTAWAUX_COHERENT_BYPASS 			0x00000002
#define 	BUSCTLREG_MSTARAUX_COHERENT_BYPASS			0x00000001
#define IOFMSTRWAUX 							0x00000018
#define 	IOFMSTRWAUX_WR_COHERENT						0x00000200
#define 	IOFMSTRWAUX_RD_COHERENT						0x00000008
#define INTSTATUS 								0x00000028
#define INTSTATUSMASK 							0x0000002c
#define ERRINTSTATUS 							0x00000030
#define ERRINTSTATUSMASK						0x00000034

/* Registersin SATA/ENET common space */
#define SATA_ENET_CONFIG_REG 					0x00000000
#define SATA_ENET_CONFIG_REG_ENET					0x00000001

/* Registers in AXI Control and Status Register block */
#define	INT_SLV_TMOMASK							0x00000010


device_attach_t ahci_attach;
device_detach_t ahci_detach;
static device_probe_t apm_xgene_ahci_probe;
static device_attach_t apm_xgene_ahci_attach;
static device_suspend_t apm_xgene_ahci_suspend;
static device_resume_t apm_xgene_ahci_resume;

static device_method_t ahci_methods[] = {
	DEVMETHOD(device_probe,				apm_xgene_ahci_probe),
	DEVMETHOD(device_attach,			apm_xgene_ahci_attach),
	DEVMETHOD(device_detach,			ahci_detach),
	DEVMETHOD(device_suspend,			apm_xgene_ahci_suspend),
	DEVMETHOD(device_resume,			apm_xgene_ahci_resume),
	DEVMETHOD(bus_print_child,			ahci_print_child),
	DEVMETHOD(bus_alloc_resource, 		ahci_alloc_resource),
	DEVMETHOD(bus_release_resource,		ahci_release_resource),
	DEVMETHOD(bus_setup_intr,			ahci_setup_intr),
	DEVMETHOD(bus_teardown_intr,		ahci_teardown_intr),
	DEVMETHOD(bus_child_location_str,	ahci_child_location_str),
	DEVMETHOD(bus_get_dma_tag,			ahci_get_dma_tag),
	DEVMETHOD_END
};

static driver_t ahci_driver = {
	.name = "ahci",
	.methods = ahci_methods,
	.size = sizeof(struct ahci_controller)
};

devclass_t ahci_devclass;
DRIVER_MODULE(ahci, simplebus, ahci_driver, ahci_devclass, NULL, NULL);

static int
apm_xgene_ahci_suspend(device_t self)
{
	int err;

	err = bus_generic_suspend(self);

	return (err);
}

static int
apm_xgene_ahci_resume(device_t self)
{
	int err = 0;

	err = bus_generic_resume(self);

	return (err);
}

static int
apm_xgene_ahci_phy_config(device_t dev, struct ahci_controller *ctl)
{
	struct resource *ahci = ctl->r_mem;
	int	channel = 0;

	while (channel < PORTCFG_PORTS) {
		uint32_t val;

		val = ATA_INL(ahci, PORTCFG);
		PORTCFG_PORTADDR_SET(val, channel);
		ATA_OUTL(ahci, PORTCFG, val);
		ATA_INL(ahci, PORTCFG);


		val = PORTPHY1CFG_ALIGNWAIT_SET(0, 0x0001fffe);
		ATA_OUTL(ahci, PORTPHY1CFG, val);
		ATA_INL(ahci, PORTPHY1CFG);

		val = PORTPHY2CFG_INIT(0x28, 0x18, 0x32, 0x19);
		ATA_OUTL(ahci, PORTPHY2CFG, val);
		ATA_INL(ahci, PORTPHY2CFG);

		val = PORTPHY3CFG_INIT(0x13, 0x08, 0x10, 0x08);
		ATA_OUTL(ahci, PORTPHY3CFG, val);
		ATA_INL(ahci, PORTPHY3CFG);

		val = PORTPHY4CFG_INIT(0x08, 0x15);
		ATA_OUTL(ahci, PORTPHY4CFG, val);
		ATA_INL(ahci, PORTPHY4CFG);

		val = ATA_INL(ahci, PORTPHY5CFG);
		PORTPHY5CFG_RTCHG_SET(val, 0x300);

		ATA_OUTL(ahci, PORTPHY5CFG, val);
		ATA_INL(ahci, PORTPHY5CFG);


		val = ATA_INL(ahci, PORTAXICFG);
		val |= PORTAXICFG_EN_CONTEXT;
		PORTAXICFG_OUTTRANS_SET(val, 0x0e);
		ATA_OUTL(ahci, PORTAXICFG, val);
		ATA_INL(ahci, PORTAXICFG);

		val = ATA_INL(ahci, PORTRANSCFG);
		PORTRANSCFG_RXWATERMARK_SET(val, 0x30);
		ATA_OUTL(ahci, PORTRANSCFG, val);
		channel++;
	}

	return (0);
}

static int
apm_xgene_ahci_probe(device_t self)
{
	if (!ofw_bus_status_okay(self))
		return (ENXIO);

	if (!ofw_bus_is_compatible(self, "apm,xgene-ahci"))
		return (ENXIO);

	device_set_desc(self, AHCI_DEVSTR);

	return (BUS_PROBE_DEFAULT);
}

static int
apm_xgene_ahci_init_mem(struct resource *r_diag)
{
	uint32_t val = 0;
	/* Try to enable memory */
	val = ATA_INL(r_diag, CFG_MEM_RAM_SHUTDOWN);
	ATA_OUTL(r_diag, CFG_MEM_RAM_SHUTDOWN, 0);
	val = ATA_INL(r_diag, CFG_MEM_RAM_SHUTDOWN);
	DELAY(1000);
	if (val == 0) {
		val = ATA_INL(r_diag, BLOCK_MEM_RDY);
		if (val == ~0)
			return 0;
	}

	return (ENXIO);
}

static int
apm_xgene_ahci_attach(device_t self)
{
	struct ahci_controller *ctl = device_get_softc(self);
	int rid = 0;
	uint32_t val = 0;
	struct resource *r_mux = NULL;
	struct resource *r_diag = NULL;
	struct resource *r_axi = NULL;
	struct resource *r_core = NULL;
	phandle_t node;
	int reg_regs= 0;
	int ret = 0;

//	if (strcmp(device_get_nameunit(self), "ahci0"))
		//return (ENXIO);

	/* Set quirks */
	ctl->quirks = AHCI_Q_XGENE_BUG;

	node = ofw_bus_get_node(self);
	/* Each register region is described by 63bit address and 64bit length,
	   which gives 16 bytes for each region. */
	reg_regs = OF_getproplen(node, "reg") / 16;

	/* rid = 0, first set of configuration registers, page 20-312 */
	ctl->r_mem = bus_alloc_resource_any(self, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (ctl->r_mem == NULL) {
		device_printf(self, "IO memory mapping failed\n");
		ret = ENXIO;
		goto out;
	}

	rid = 1;
	r_core = bus_alloc_resource_any(self, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (r_core == NULL) {
		device_printf(self, "Failed to map CORE registers\n");
		ret = ENXIO;
		goto out;
	}

	rid = 2;
	r_diag = bus_alloc_resource_any(self, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (r_diag == NULL) {
		device_printf(self, "Failed to map DIAG registers\n");
		ret = ENXIO;
		goto out;
	}

	rid = 3;
	r_axi = bus_alloc_resource_any(self, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (r_axi == NULL) {
		device_printf(self, "Failed to map AXI registers\n");
		ret = ENXIO;
		goto out;
	}

	/* Not all AHCI nodes have the SATE/ENET multiplexed region */
	if (reg_regs > 4) {
		rid = 4;
		r_mux = bus_alloc_resource_any(self, SYS_RES_MEMORY, &rid, RF_ACTIVE);
		if (r_mux == NULL) {
			device_printf(self, "Failed to map MUX registers\n");
			ret = ENXIO;
			goto out;
		}

		/* Enable SATA on SATA/ENET multiplexer */
		val = ATA_INL(r_mux, SATA_ENET_CONFIG_REG);
		val &= ~SATA_ENET_CONFIG_REG_ENET;
		ATA_OUTL(r_mux, SATA_ENET_CONFIG_REG, val);
		val = ATA_INL(r_mux, SATA_ENET_CONFIG_REG);

		bus_release_resource(self, SYS_RES_MEMORY, 4, r_mux);
	}

	/* Setting up interrupt */
	ctl->numirqs = 1;

	if (apm_xgene_ahci_init_mem(r_diag) != 0) {
		ret = ENXIO;
		device_printf(self, "Failed to initialize SoC internal RAM\n");
		goto out;
	}

	/* Attempt to setup physical channels before attaching AHCI */
	apm_xgene_ahci_phy_config(self, ctl);

	/* Clear all interrupt requests */
	ATA_OUTL(ctl->r_mem, AHCI_IS, 0xffffffff);
	ATA_INL(ctl->r_mem, AHCI_IS);

	ATA_OUTL(r_core, INTSTATUS, 0x00);
	ATA_OUTL(r_core, INTSTATUSMASK, 0x00);
	ATA_INL(r_core, INTSTATUSMASK);
	ATA_OUTL(r_core, ERRINTSTATUS, 0x00);

	ATA_OUTL(r_core, ERRINTSTATUSMASK, 0x00);
	ATA_INL(r_core, ERRINTSTATUSMASK);
	/* Clear all error flags */
	ATA_OUTL(r_core, SLVRDERRATTRIBUTES, 0xffffffff);
	ATA_OUTL(r_core, SLVWRERRATTRIBUTES, 0xffffffff);
	ATA_OUTL(r_core, MSTRDERRATTRIBUTES, 0xffffffff);
	ATA_OUTL(r_core, MSTWRERRATTRIBUTES, 0xffffffff);

	/* Unmask all timeouts */
	ATA_OUTL(r_axi, INT_SLV_TMOMASK, 0x00);
	ATA_INL(r_axi, INT_SLV_TMOMASK);


	/* Bypass read/write coherency */
	val = ATA_INL(r_core, BUSCTLREG);
	val &= ~(BUSCTLREG_MSTAWAUX_COHERENT_BYPASS | BUSCTLREG_MSTARAUX_COHERENT_BYPASS);
	ATA_OUTL(r_core, BUSCTLREG, val);
	ATA_INL(r_core, BUSCTLREG);

	val = ATA_INL(r_core, IOFMSTRWAUX);
	val |= IOFMSTRWAUX_WR_COHERENT | IOFMSTRWAUX_RD_COHERENT;
	ATA_OUTL(r_core, IOFMSTRWAUX, val);
	ATA_INL(r_core, IOFMSTRWAUX);

	if (ahci_attach(self) != 0) {
		device_printf(self, "Generic AHCI attach failed\n");
		ret = ENXIO;
		goto out;
	}

out:
	if (ret != 0 && ctl->r_mem != NULL)
		bus_release_resource(self, SYS_RES_MEMORY, 0, ctl->r_mem);

	if (r_core != NULL)
		bus_release_resource(self, SYS_RES_MEMORY, 1, r_core);

	if (r_diag != NULL)
		bus_release_resource(self, SYS_RES_MEMORY, 2, r_diag);

	if (r_axi != NULL)
		bus_release_resource(self, SYS_RES_MEMORY, 3, r_axi);

	return (ret);
}
