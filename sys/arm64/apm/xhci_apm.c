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
/*
 * FDT attachment driver for the USB Enhanced Host Controller.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/condvar.h>
#include <sys/malloc.h>
#include <sys/rman.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>

#include <dev/usb/usb_busdma.h>
#include <dev/usb/usb_process.h>

#include <dev/usb/usb_controller.h>
#include <dev/usb/usb_bus.h>
#include <dev/usb/controller/xhci.h>
#include <dev/usb/controller/xhcireg.h>

#define	XHCI_HC_DEVSTR		"USB 3.0 XHCI Platform Integrated controller"
#define	XHCI_HC_VENDOR		"APM"

static device_attach_t apm_xhci_attach;
static device_detach_t apm_xhci_detach;
static device_shutdown_t apm_xhci_shutdown;
static device_suspend_t apm_xhci_suspend;
static device_resume_t apm_xhci_resume;

/* XHCI HC regs start at this offset within USB range */
static int
apm_xhci_suspend(device_t self)
{
	xhci_softc_t *sc = device_get_softc(self);
	int err;

	err = bus_generic_suspend(self);
	if (err != 0)
		return (err);

	return (xhci_halt_controller(sc));
}

static int
apm_xhci_resume(device_t self)
{
	xhci_softc_t *sc = device_get_softc(self);
	int err;

	err = bus_generic_resume(self);
	if (err != 0)
		return (err);

	return (xhci_start_controller(sc));
}

static int
apm_xhci_shutdown(device_t self)
{
	xhci_softc_t *sc = device_get_softc(self);
	int err;

	err = bus_generic_shutdown(self);
	if (err != 0)
		return (err);

	return (xhci_halt_controller(sc));
}

static int
apm_xhci_probe(device_t self)
{
	if (!ofw_bus_status_okay(self) ||
		!ofw_bus_is_compatible(self, "xhci-platform"))
		return (ENXIO);

	device_set_desc(self, XHCI_HC_DEVSTR);

	return (BUS_PROBE_DEFAULT);
}

static int
apm_xhci_attach(device_t self)
{
	xhci_softc_t *sc = device_get_softc(self);
	int err = 0;
	int rid = 0;

	sc->sc_bus.parent = self;
	sc->sc_bus.devices = sc->sc_devices;
	sc->sc_bus.devices_max = XHCI_MAX_DEVICES;

	sc->sc_io_res = bus_alloc_resource_any(self, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->sc_io_res == 0) {
		device_printf(self, "Failed to map IO memory\n");
		apm_xhci_detach(self);
		return (ENXIO);
	}

	sc->sc_io_tag = rman_get_bustag(sc->sc_io_res);
	sc->sc_io_hdl = rman_get_bushandle(sc->sc_io_res);
	sc->sc_io_size = rman_get_size(sc->sc_io_res);

	rid = 0;
	sc->sc_irq_res = bus_alloc_resource_any(self, SYS_RES_IRQ, &rid,
	    RF_SHAREABLE | RF_ACTIVE);
	if (sc->sc_irq_res == NULL) {
		device_printf(self, "Failed to allocate IRQ\n");
		apm_xhci_detach(self);
		return (ENXIO);
	}

	sc->sc_bus.bdev = device_add_child(self, "usbus", -1);
	if (sc->sc_bus.bdev == 0) {
		device_printf(self, "Failed to add USB device\n");
		apm_xhci_detach(self);
		return (ENXIO);
	}

	device_set_ivars(sc->sc_bus.bdev, &sc->sc_bus);

	sprintf(sc->sc_vendor, XHCI_HC_VENDOR);
	device_set_desc(sc->sc_bus.bdev, XHCI_HC_DEVSTR);

	err = bus_setup_intr(self, sc->sc_irq_res, INTR_TYPE_BIO | INTR_MPSAFE,
	    NULL, (driver_intr_t *)xhci_interrupt, sc, &sc->sc_intr_hdl);

	if (err != 0) {
		device_printf(self, "Failed to setup IRQ, with error %d\n", err);
		sc->sc_intr_hdl = NULL;
		apm_xhci_detach(self);
		return (err);
	}

	err = xhci_init(sc, self, 0);
	if (err != 0) {
		device_printf(self, "Failed to init XHCI, with error %d\n", err);
		apm_xhci_detach(self);
		return (ENXIO);
	}

	err = xhci_start_controller(sc);
	if (err != 0) {
		device_printf(self, "Failed to start XHCI controller, with error %d\n", err);
		apm_xhci_detach(self);
		return (ENXIO);
	}

	err = device_probe_and_attach(sc->sc_bus.bdev);
	if (err != 0) {
		device_printf(self, "Failed to initialize USB, with error %d\n", err);
		apm_xhci_detach(self);
		return (ENXIO);
	}

	return (0);
}

static int
apm_xhci_detach(device_t self)
{
	xhci_softc_t *sc = device_get_softc(self);
	device_t bdev;
	int err;

	if (sc->sc_bus.bdev != NULL) {
		bdev = sc->sc_bus.bdev;
		device_detach(bdev);
		device_delete_child(self, bdev);
	}

	/* during module unload there are lots of children leftover */
	device_delete_children(self);

	if (sc->sc_irq_res != NULL && sc->sc_intr_hdl != NULL) {
		/*
		 * only call xhci_detach() after xhci_init()
		 */
		/* TODO: there is no substitution for this */
		//xhci_detach(sc);

		err = bus_teardown_intr(self, sc->sc_irq_res, sc->sc_intr_hdl);

		if (err != 0)
			/* XXX or should we panic? */
			device_printf(self, "Failed to tear down IRQ, with error %d\n",
			    err);
		sc->sc_intr_hdl = NULL;
	}

	if (sc->sc_irq_res != NULL) {
		bus_release_resource(self, SYS_RES_IRQ, 1, sc->sc_irq_res);
		sc->sc_irq_res = NULL;
	}

	if (sc->sc_io_res != NULL) {
		bus_release_resource(self, SYS_RES_MEMORY, 0,
		    sc->sc_io_res);
		sc->sc_io_res = NULL;
	}

	usb_bus_mem_free_all(&sc->sc_bus, &xhci_iterate_hw_softc);

	return (0);
}

static device_method_t xhci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, apm_xhci_probe),
	DEVMETHOD(device_attach, apm_xhci_attach),
	DEVMETHOD(device_detach, apm_xhci_detach),
	DEVMETHOD(device_suspend, apm_xhci_suspend),
	DEVMETHOD(device_resume, apm_xhci_resume),
	DEVMETHOD(device_shutdown, apm_xhci_shutdown),

	/* Bus interface */
	DEVMETHOD(bus_print_child, bus_generic_print_child),

	DEVMETHOD_END
};

static driver_t xhci_driver = {
	.name = "xhci",
	.methods = xhci_methods,
	.size = sizeof(xhci_softc_t)
};

static devclass_t xhci_devclass;

DRIVER_MODULE(xhci, simplebus, xhci_driver, xhci_devclass, 0, 0);
MODULE_DEPEND(xhci, usb, 1, 1, 1);
