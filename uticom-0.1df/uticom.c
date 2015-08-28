/*-
 * Copyright (c) 2005, Dmitry Komissaroff <dxi@mail.ru>.
 * All rights reserved.
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

/*
 * Simple datasheet
 * TODO: 
 * change datashit link...
 *
 */


#include "utifw3410_p.h"
/*#include "utifw5052.h"*/
#include "uticom.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
/*#include <sys/module.h>*/
#include <sys/malloc.h>
#include <sys/bus.h>
#include <sys/ioccom.h>
#include <sys/fcntl.h>
#include <sys/conf.h>
#include <sys/tty.h>
#include <sys/file.h>
#include <sys/select.h>
#include <sys/proc.h>
#include <sys/vnode.h>
#include <sys/poll.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <bus/usb/usb.h>
#include <bus/usb/usbcdc.h>

#include <bus/usb/usbdi.h>
#include <bus/usb/usbdi_util.h>
#include <bus/usb/usbdivar.h>

#include <bus/usb/usbdevs.h>

#include <bus/usb/usb_quirks.h>
#include <bus/usb/usb_port.h>

#include <dev/usbmisc/ucom/ucomvar.h>

SYSCTL_NODE(_hw_usb, OID_AUTO, uticom, CTLFLAG_RW, 0, "USB uticom");

/*#define USB_DEBUG 1*/


#ifdef USB_DEBUG
static int	uticomdebug = 0;
SYSCTL_INT(_hw_usb_uticom, OID_AUTO, debug, CTLFLAG_RW,
	   &uticomdebug, 0, "uticom debug level");

#define DPRINTFN(n, x)	do { \
				if (uticomdebug > (n)) \
					logprintf x; \
			} while (0)
#else
#define DPRINTFN(n, x)
#endif

#define DPRINTF(x) DPRINTFN(0, x)

#define UTICOM_MODVER			1	/* module version */

#define	UTICOM_CONFIG_INDEX		1
#define	UTICOM_ACTIVE_INDEX		2

#define	UTICOM_IFACE_INDEX		0
#define	UTICOM_SECOND_IFACE_INDEX	1

#ifndef UTICOM_INTR_INTERVAL
#define UTICOM_INTR_INTERVAL		100	/* ms */
#endif

/* TI Defines */

#define TI_DRIVER_VERSION	"v0.1dragonflybsd"
#define TI_DRIVER_AUTHOR	"Griffin <dxi@mail.ru>"
#define TI_DRIVER_DESC		"TI USB 3410 Serial Driver"

#define TI_FIRMWARE_BUF_SIZE	16284

#define TI_WRITE_BUF_SIZE	64
#define TI_READ_BUF_SIZE	64

#define TI_TMP_BUF_SIZE		1024

#define TI_TRANSFER_TIMEOUT	2

#define TI_DEFAULT_LOW_LATENCY	0
#define TI_DEFAULT_CLOSING_WAIT	4000		/* in .01 secs */

/* supported setserial flags */
#define TI_SET_SERIAL_FLAGS	(ASYNC_LOW_LATENCY)


#define BMLS_VENDOR_ID		0x1313		// very preliminary !!!	*
#define BMLS_PRODUCT_ID		0x1414		// very preliminary !!!	*

/* TI Structures */

struct ti_buf {
	unsigned int		buf_size;
	char			*buf_buf;
	char			*buf_get;
	char			*buf_put;
};



/*****************************************************************************************************************/
struct	uticom_softc {
	struct ucom_softc	sc_ucom;

	int			sc_iface_number;	/* interface number */

	usbd_interface_handle	sc_intr_iface;	/* interrupt interface */
	int			sc_intr_number;	/* interrupt number */
	usbd_pipe_handle	sc_intr_pipe;	/* interrupt pipe */
	u_char			*sc_intr_buf;	/* interrupt buffer */
	int			sc_isize;

	usb_cdc_line_state_t	sc_line_state;	/* current line state */
	u_char			sc_dtr;		/* current DTR state */
	u_char			sc_rts;		/* current RTS state */
	u_char			sc_status;

	u_char			sc_lsr;		/* Local status register */
	u_char			sc_msr;		/* uplcom status register */
};

/*
 * These are the maximum number of bytes transferred per frame.
 * The output buffer size cannot be increased due to the size encoding.
 */
#define UTICOMIBUFSIZE 64
#define UTICOMOBUFSIZE 64

static	usbd_status uticom_reset(struct uticom_softc *);
static	usbd_status uticom_set_line_coding(struct uticom_softc *,
					   usb_cdc_line_state_t *);
static	usbd_status uticom_set_crtscts(struct uticom_softc *);
static	void uticom_intr(usbd_xfer_handle, usbd_private_handle, usbd_status);

static	void uticom_set(void *, int, int, int);
static	void uticom_dtr(struct uticom_softc *, int);
static	void uticom_rts(struct uticom_softc *, int);
static	void uticom_break(struct uticom_softc *, int);
static	void uticom_set_line_state(struct uticom_softc *);
static	void uticom_get_status(void *, int, u_char *, u_char *);
#if TODO
static	int  uticom_ioctl(void *, int, u_long, caddr_t, int, usb_proc_ptr);
#endif
static	int  uticom_param(void *, int, struct termios *);
static	int  uticom_open(void *, int);
static	void uticom_close(void *, int);

static int ti_download_firmware(struct uticom_softc *sc,unsigned int pipeno,usbd_device_handle dev,
	unsigned char *firmware, unsigned int firmware_size);
static usbd_status ti_reestablish_dpipe(struct uticom_softc *sc,usbd_device_handle dev);

struct ucom_callback uticom_callback = {
	uticom_get_status,
	uticom_set,
	uticom_param,
	NULL, /* uplcom_ioctl, TODO */
	uticom_open,
	uticom_close,
	NULL,
	NULL
};

static const struct uticom_product {
	uint16_t	vendor;
	uint16_t	product;
} uticom_products [] = {
	/* I/O DATA USB-RSAQ */
	{ TI_VENDOR_ID, TI_3410_PRODUCT_ID },
/*	{ TI_VENDOR_ID, BMLS_PRODUCT_ID },
	{ BMLS_VENDOR_ID, BMLS_PRODUCT_ID},*/
	{ 0, 0 }
};

static device_probe_t uticom_match;
static device_attach_t uticom_attach;
static device_detach_t uticom_detach;

static device_method_t uticom_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, uticom_match),
	DEVMETHOD(device_attach, uticom_attach),
	DEVMETHOD(device_detach, uticom_detach),
	{ 0, 0 }
};

static driver_t uticom_driver = {
	"ucom",
	uticom_methods,
	sizeof (struct uticom_softc)
};

DRIVER_MODULE(uticom, uhub, uticom_driver, ucom_devclass, usbd_driver_load, 0);
MODULE_DEPEND(uticom, usb, 1, 1, 1);
MODULE_DEPEND(uticom, ucom, UCOM_MINVER, UCOM_PREFVER, UCOM_MAXVER);
MODULE_VERSION(uticom, UTICOM_MODVER);

static int	uticominterval = UTICOM_INTR_INTERVAL;

static int
sysctl_hw_usb_uticom_interval(SYSCTL_HANDLER_ARGS)
{

	int err, val;

	val = uticominterval;
	err = sysctl_handle_int(oidp, &val, sizeof(val), req);
	if (err != 0 || req->newptr == NULL)
		return (err);
	if (0 < val && val <= 1000)
		uticominterval = val;
	else
		err = EINVAL;

	return (err);

}

SYSCTL_PROC(_hw_usb_uticom, OID_AUTO, interval, CTLTYPE_INT | CTLFLAG_RW,
	    0, sizeof(int), sysctl_hw_usb_uticom_interval,
	    "I", "uticom interrpt pipe interval");

static int	uticomstickdsr = 0;
static int
sysctl_hw_usb_uticom_stickdsr(SYSCTL_HANDLER_ARGS)
{

	int err, val;

	val = uticomstickdsr;
	err = sysctl_handle_int(oidp, &val, sizeof(val), req);
	if (err != 0 || req->newptr == NULL)
		return (err);
	if (0 < val && val <= 1)
		uticomstickdsr = val;
	else
		err = EINVAL;

	return (err);

}

SYSCTL_PROC(_hw_usb_uticom, OID_AUTO, stickdsr, CTLTYPE_INT | CTLFLAG_RW,
	    0, sizeof(int), sysctl_hw_usb_uticom_stickdsr,
	    "I", "uticom sticky dsr level");



USB_MATCH(uticom)
{

	USB_MATCH_START(uticom, uaa);
	int i;

	if (uaa->iface != NULL)
		return (UMATCH_NONE);

	for (i = 0; uticom_products[i].vendor != 0; i++) {
		if (uticom_products[i].vendor == uaa->vendor &&
		    uticom_products[i].product == uaa->product) {
			return (UMATCH_VENDOR_PRODUCT);
		}
	}
	return (UMATCH_NONE);

}

USB_ATTACH(uticom)
{

	USB_ATTACH_START(uticom, sc, uaa);
	usbd_device_handle dev = uaa->device;
	struct ucom_softc *ucom;
	usb_config_descriptor_t *cdesc;
	usb_interface_descriptor_t *id;
	usb_endpoint_descriptor_t *ed;
	char *devinfo;
	const char *devname;
	usbd_status err;
	int i;
	int status;
	usb_device_descriptor_t *dd;


	devinfo = kmalloc(1024, M_USBDEV, M_WAITOK);
	ucom = &sc->sc_ucom;

	bzero(sc, sizeof (struct uticom_softc));

	usbd_devinfo(dev, 0, devinfo);

	/* USB_ATTACH_SETUP; */

	ucom->sc_dev = self;
	device_set_desc_copy(self, devinfo);

	/* USB_ATTACH_SETUP; */

	ucom->sc_udev = dev;
	ucom->sc_iface = uaa->iface;

	devname = USBDEVNAME(ucom->sc_dev);
	DPRINTF(("uticom_attach: %s: %s\n", devname, devinfo));
	
	DPRINTF(("uticom_attach: sc = %p\n", sc));

	/* initialize endpoints */

	ucom->sc_bulkin_no = ucom->sc_bulkout_no = -1;
	sc->sc_intr_number = -1;
	sc->sc_intr_pipe = NULL;


	dd=usbd_get_device_descriptor(dev);
	DPRINTF(("uticom_attach: num of configurations %d\n",dd->bNumConfigurations));
	/* Move the device into the configured state. */

/**/
/*if (uaa->product!=BMLS_PRODUCT_ID) {*/
	if (dd->bNumConfigurations==1) {
		
		/* loading FirmWare */


	DPRINTF(("uticom_attach: starting loading firmware\n"));

	err = usbd_set_config_index(dev, UTICOM_CONFIG_INDEX, 1);
	if (err) {
		kprintf("%s: failed to set configuration: %s\n",
			devname, usbd_errstr(err));
		ucom->sc_dying = 1;
		goto error;
	}

	/* get the config descriptor */

	cdesc = usbd_get_config_descriptor(ucom->sc_udev);

	if (cdesc == NULL) {
		kprintf("%s: failed to get configuration descriptor\n",
			USBDEVNAME(ucom->sc_dev));
		ucom->sc_dying = 1;
		goto error;
	}

	
	err = usbd_device2interface_handle(dev, UTICOM_IFACE_INDEX,
					   &ucom->sc_iface);
	if (err) {
		kprintf("%s: failed to get interface: %s\n",
			devname, usbd_errstr(err));
		ucom->sc_dying = 1;
		goto error;
	}

	/* Find the interrupt endpoints */

	id = usbd_get_interface_descriptor(ucom->sc_iface);
	sc->sc_iface_number = id->bInterfaceNumber;

	for (i = 0; i < id->bNumEndpoints; i++) {
		ed = usbd_interface2endpoint_descriptor(ucom->sc_iface, i);
		if (ed == NULL) {
			kprintf("%s: no endpoint descriptor for %d\n",
				USBDEVNAME(ucom->sc_dev), i);
			ucom->sc_dying = 1;
			goto error;
		}/*if*/




/*Griffin------------*/
		if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_OUT &&
		    UE_GET_XFERTYPE(ed->bmAttributes) == UE_BULK) {
			ucom->sc_bulkout_no = ed->bEndpointAddress;
			/*printf("attach: data bulk out num: %d\n",ed->bEndpointAddress);*/
                }/*if*/

	if (ucom->sc_bulkout_no == -1) {
		kprintf("%s: Could not find data bulk out\n",
			USBDEVNAME(ucom->sc_dev));
		ucom->sc_dying = 1;
		goto error;
	}/*if */

	}/*for*/


	status = ti_download_firmware(sc, ucom->sc_bulkout_no,dev,ti_fw_3410,
				sizeof(ti_fw_3410));

	if (status) {
		kprintf("uticom_attach: error while download firmware...\n");
		ucom->sc_dying = 1;
		goto error;
	}/*if*/
		

	status=usbd_reload_device_desc(dev);
	if (status) {
		kprintf("uticom_attach: error while reload dev desc...\n");
		ucom->sc_dying = 1;
		kfree(devinfo, M_USBDEV);
		return ENXIO;
	}/*if*/


	}/*loading firmware*/

	DPRINTF(("uticom_attach: After firmware\n"));

	devname = USBDEVNAME(ucom->sc_dev);
	kprintf("%s\n", devinfo);

	dd=usbd_get_device_descriptor(dev);
	DPRINTF(("uticom_attach: num of configurations %d\n",dd->bNumConfigurations));

	err = usbd_set_config_index(dev, UTICOM_ACTIVE_INDEX, 1);
	if (err) {
		kprintf("%s: failed to set configuration: %s\n",
			devname, usbd_errstr(err));
		ucom->sc_dying = 1;
		goto error;
	}


	/* get the config descriptor*/

	cdesc = usbd_get_config_descriptor(ucom->sc_udev);

	if (cdesc == NULL) {
		kprintf("%s: failed to get configuration descriptor\n",
			USBDEVNAME(ucom->sc_dev));
		ucom->sc_dying = 1;
		goto error;
	}


	status=usbd_reload_device_desc(dev);
	devname = USBDEVNAME(ucom->sc_dev);
	DPRINTF(("We have configValue=%d\n",cdesc->bConfigurationValue));
	DPRINTF(("After select config: %s: %s\n", devname, devinfo));
	DPRINTF(("We have %d interfaces\n",cdesc->bNumInterface));
	
	/* get the (first/common) interface */

	err = usbd_device2interface_handle(dev, UTICOM_IFACE_INDEX,
					   &ucom->sc_iface);
	if (err) {
		kprintf("%s: failed to get interface: %s\n",
			devname, usbd_errstr(err));
		ucom->sc_dying = 1;
		goto error;
	}

	/* Find the interrupt endpoints */

	id = usbd_get_interface_descriptor(ucom->sc_iface);
	sc->sc_iface_number = id->bInterfaceNumber;
	DPRINTF(("We have %d endpoints for first iface\n",id->bNumEndpoints));

	for (i = 0; i < id->bNumEndpoints; i++) {
		ed = usbd_interface2endpoint_descriptor(ucom->sc_iface, i);
		if (ed == NULL) {
			kprintf("%s: no endpoint descriptor for %d\n",
				USBDEVNAME(ucom->sc_dev), i);
			ucom->sc_dying = 1;
			goto error;
		}


		if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_IN &&
		    UE_GET_XFERTYPE(ed->bmAttributes) == UE_INTERRUPT) {
			sc->sc_intr_number = ed->bEndpointAddress;
			sc->sc_isize = UGETW(ed->wMaxPacketSize);
		}


	}

	if (sc->sc_intr_number == -1) {
		kprintf("%s: Could not find interrupt in\n",
			USBDEVNAME(ucom->sc_dev));
/*We don't have interrupt...*/
/*		ucom->sc_dying = 1;
//		goto error; */
	}
/**/
	/* keep interface for interrupt */

	sc->sc_intr_iface = ucom->sc_iface;

	if (cdesc->bNumInterface == 2) {
	DPRINTF(("We have second interface ?!"));
		err = usbd_device2interface_handle(dev,
						   UTICOM_SECOND_IFACE_INDEX,
						   &ucom->sc_iface);
		if (err) {
			kprintf("%s: failed to get second interface: %s\n",
				devname, usbd_errstr(err));
			ucom->sc_dying = 1;
			goto error;
		}
	}

	/* Find the bulk{in,out} endpoints */

	id = usbd_get_interface_descriptor(ucom->sc_iface);
	sc->sc_iface_number = id->bInterfaceNumber;

	for (i = 0; i < id->bNumEndpoints; i++) {
		ed = usbd_interface2endpoint_descriptor(ucom->sc_iface, i);
		if (ed == NULL) {
			kprintf("%s: no endpoint descriptor for %d\n",
				USBDEVNAME(ucom->sc_dev), i);
			ucom->sc_dying = 1;
			goto error;
		}

		if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_IN &&
		    UE_GET_XFERTYPE(ed->bmAttributes) == UE_BULK) {
			ucom->sc_bulkin_no = ed->bEndpointAddress;
		} else if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_OUT &&
		    UE_GET_XFERTYPE(ed->bmAttributes) == UE_BULK) {
			ucom->sc_bulkout_no = ed->bEndpointAddress;
		}
	}

	if (ucom->sc_bulkin_no == -1) {
		kprintf("%s: Could not find data bulk in\n",
			USBDEVNAME(ucom->sc_dev));
		ucom->sc_dying = 1;
		goto error;
	}

	if (ucom->sc_bulkout_no == -1) {
		kprintf("%s: Could not find data bulk out\n",
			USBDEVNAME(ucom->sc_dev));
		ucom->sc_dying = 1;
		goto error;
	}

	sc->sc_dtr = sc->sc_rts = -1;
	ucom->sc_parent = sc;
	ucom->sc_portno = UCOM_UNK_PORTNO;

	/* bulkin, bulkout set above */

	ucom->sc_ibufsize = UTICOMIBUFSIZE;
	ucom->sc_obufsize = UTICOMOBUFSIZE;
	ucom->sc_ibufsizepad = UTICOMIBUFSIZE;
	ucom->sc_opkthdrlen = 0;
	ucom->sc_callback = &uticom_callback;

	err = uticom_reset(sc);

	if (err) {
		kprintf("%s: reset failed: %s\n",
		       USBDEVNAME(ucom->sc_dev), usbd_errstr(err));
		ucom->sc_dying = 1;
		goto error;
	}

	DPRINTF(("uticom: in = 0x%x, out = 0x%x, intr = 0x%x\n",
		 ucom->sc_bulkin_no, ucom->sc_bulkout_no, sc->sc_intr_number));


	ucom_attach(&sc->sc_ucom);

	kfree(devinfo, M_USBDEV);
	USB_ATTACH_SUCCESS_RETURN;

error:
	kfree(devinfo, M_USBDEV);
	USB_ATTACH_ERROR_RETURN;

}

USB_DETACH(uticom)
{

	USB_DETACH_START(uticom, sc);
	int rv = 0;

	DPRINTF(("uticom_detach: sc = %p\n", sc));

	if (sc->sc_intr_pipe != NULL) {
		usbd_abort_pipe(sc->sc_intr_pipe);
		usbd_close_pipe(sc->sc_intr_pipe);
		kfree(sc->sc_intr_buf, M_USBDEV);
		sc->sc_intr_pipe = NULL;
	}

	sc->sc_ucom.sc_dying = 1;

	rv = ucom_detach(&sc->sc_ucom);

	return (rv);

}

static usbd_status
uticom_reset(struct uticom_softc *sc)
{

	usb_device_request_t req;
	usbd_status err;

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = REQUEST_SON;
	USETW(req.wValue, 0);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);

	err = usbd_do_request(sc->sc_ucom.sc_udev, &req, 0);
	if (err){
		kprintf("%s: uticom_reset: %s\n",
		       USBDEVNAME(sc->sc_ucom.sc_dev), usbd_errstr(err));
	return (EIO);
	}

	DPRINTF(("uticom:reset\n"));
	return (0);

}

static void
uticom_set_line_state(struct uticom_softc *sc)
{
return;
	usb_device_request_t req;
	int ls;
	usbd_status err;
/*
	ls = (sc->sc_dtr ? UCDC_LINE_DTR : 0) |
		(sc->sc_rts ? UCDC_LINE_RTS : 0);
*/

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = UCDC_SET_CONTROL_LINE_STATE;
	USETW(req.wValue, ls);
	USETW(req.wIndex, sc->sc_iface_number);
	USETW(req.wLength, 0);

	err = usbd_do_request(sc->sc_ucom.sc_udev, &req, 0);
	if (err)
		kprintf("%s: uticom_set_line_status: %s\n",
		       USBDEVNAME(sc->sc_ucom.sc_dev), usbd_errstr(err));

}

static void
uticom_set(void *addr, int portno, int reg, int onoff)
{

	struct uticom_softc *sc = addr;

	switch (reg) {
	case UCOM_SET_DTR:
		uticom_dtr(sc, onoff);
		break;
	case UCOM_SET_RTS:
		uticom_rts(sc, onoff);
		break;
	case UCOM_SET_BREAK:
		uticom_break(sc, onoff);
		break;
	default:
		break;
	}

}

static void
uticom_dtr(struct uticom_softc *sc, int onoff)
{

	DPRINTF(("uticom_dtr: onoff = %d\n", onoff));

	if (sc->sc_dtr == onoff) return;

	sc->sc_dtr = onoff;

	usb_device_request_t req;
	int ls;
	usbd_status err;

	ls = (sc->sc_dtr ? UCDC_LINE_DTR : 0);

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = REQUEST_DTR;
	USETW(req.wValue, ls);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);

	err = usbd_do_request(sc->sc_ucom.sc_udev, &req, 0);
	if (err)
		kprintf("%s: uticom_dtr: %s\n",
		       USBDEVNAME(sc->sc_ucom.sc_dev), usbd_errstr(err));
}

static void
uticom_rts(struct uticom_softc *sc, int onoff)
{

	DPRINTF(("uticom_rts: onoff = %d\n", onoff));

	if (sc->sc_rts == onoff)
		return;
	sc->sc_rts = onoff;

	usb_device_request_t req;
	int ls;
	usbd_status err;

	ls = (sc->sc_dtr ? UCDC_LINE_RTS : 0);

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = REQUEST_RTS;
	USETW(req.wValue, ls);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);

	err = usbd_do_request(sc->sc_ucom.sc_udev, &req, 0);
	if (err)
		kprintf("%s: uticom_dtr: %s\n",
		       USBDEVNAME(sc->sc_ucom.sc_dev), usbd_errstr(err));


}

static void
uticom_break(struct uticom_softc *sc, int onoff)
{

	usb_device_request_t req;
	usbd_status err;

	DPRINTF(("uticom_break: onoff = %s(%d)\n", onoff?"on":"off",onoff));

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = REQUEST_BREAK;
	USETW(req.wValue, onoff ? 1 : 0);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);

	err = usbd_do_request(sc->sc_ucom.sc_udev, &req, 0);
	if (err)
		kprintf("%s: uticom_break: %s\n",
		       USBDEVNAME(sc->sc_ucom.sc_dev), usbd_errstr(err));

}


static usbd_status
uticom_set_crtscts(struct uticom_softc *sc)
{
	usb_device_request_t req;
	usbd_status err;

	DPRINTF(("uticom_set_crtscts: on\n"));

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = REQUEST_CRTSCTS;
	USETW(req.wValue, 1);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);

	err = usbd_do_request(sc->sc_ucom.sc_udev, &req, 0);
	if (err) {
		kprintf("%s: uticom_set_crtscts: %s\n",
		       USBDEVNAME(sc->sc_ucom.sc_dev), usbd_errstr(err));
		return (err);
	}

	return (USBD_NORMAL_COMPLETION);

}

static usbd_status
uticom_set_line_coding(struct uticom_softc *sc, usb_cdc_line_state_t *state)
{
	usb_device_request_t req;
	usbd_status err;

__uint8_t brate=0;

	DPRINTF(("uticom_set_line_coding: rate = %d, fmt = %d, parity = %d bits = %d\n",
		 UGETDW(state->dwDTERate), state->bCharFormat,
		 state->bParityType, state->bDataBits));

	switch UGETDW(state->dwDTERate) {
	case B1200:
		brate = BaudRate1200;
		break;
	case B2400:
		brate = BaudRate2400;
		break;
	case B4800:
		brate = BaudRate4800;
		break;
	case B7200:
		brate = BaudRate7200;
		break;
	case B9600:
		brate = BaudRate9600;
		break;
	case B14400:
		brate = BaudRate14400;
		break;
	case B19200:
		brate = BaudRate19200;
		break;
	case B38400:
		brate = BaudRate38400;
		break;
	case B57600:
		brate = BaudRate57600;
		break;
	case B115200:
		brate = BaudRate115200;
		break;
	default:{
		kprintf("%s: uticom_set_line_coding: unsupported BaudRate %d\n",
		       USBDEVNAME(sc->sc_ucom.sc_dev),UGETDW(state->dwDTERate));
		return (EIO);
		}
	}

    err=0;
	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = REQUEST_BAUD;
/*	USETW(req.wValue, brate); */
	req.wValue[0]=brate;
	req.wValue[1]=brate;
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);

	err = usbd_do_request(sc->sc_ucom.sc_udev, &req, 0);
	if (err) {
		kprintf("%s: uticom_set_line_coding: %s\n",
		       USBDEVNAME(sc->sc_ucom.sc_dev), usbd_errstr(err));
		return (err);
	}

struct line myline;

myline.divh=0;
myline.divl=0;

	req.bmRequestType = UT_READ_VENDOR_DEVICE;
	req.bRequest = REQUEST_LINE;
	USETW(req.wValue, 0);
	USETW(req.wIndex, 0);
	USETW(req.wLength, sizeof(myline));
	err = usbd_do_request_flags(sc->sc_ucom.sc_udev, &req, &myline,USBD_SHORT_XFER_OK,0,USBD_DEFAULT_TIMEOUT);
	if (err) {
		kprintf("%s: uticom_get_line_coding: %s\n",
		       USBDEVNAME(sc->sc_ucom.sc_dev), usbd_errstr(err));
		return (err);
	}

DPRINTF(("brate after get: lo:0x%02x hi:0x%02x\n",myline.divl,myline.divh));
	sc->sc_line_state = *state;

	return (USBD_NORMAL_COMPLETION);
}

static int
uticom_param(void *addr, int portno, struct termios *t)
{

	struct uticom_softc *sc = addr;
	usbd_status err;
	usb_cdc_line_state_t ls;

	DPRINTF(("uticom_param: sc = %p\n", sc));

	USETDW(ls.dwDTERate, t->c_ospeed);
	if (ISSET(t->c_cflag, CSTOPB))
		ls.bCharFormat = UCDC_STOP_BIT_2;
	else
		ls.bCharFormat = UCDC_STOP_BIT_1;
	if (ISSET(t->c_cflag, PARENB)) {
		if (ISSET(t->c_cflag, PARODD))
			ls.bParityType = UCDC_PARITY_ODD;
		else
			ls.bParityType = UCDC_PARITY_EVEN;
	} else
		ls.bParityType = UCDC_PARITY_NONE;
	switch (ISSET(t->c_cflag, CSIZE)) {
	case CS5:
		ls.bDataBits = 5;
		break;
	case CS6:
		ls.bDataBits = 6;
		break;
	case CS7:
		ls.bDataBits = 7;
		break;
	case CS8:
		ls.bDataBits = 8;
		break;
	}

	err = uticom_set_line_coding(sc, &ls);
	if (err)
		return (EIO);

	if (ISSET(t->c_cflag, CRTSCTS)) {
		err = uticom_set_crtscts(sc);
		if (err)
			return (EIO);
	} 

	return (0);

}

static int
uticom_open(void *addr, int portno)
{

	struct uticom_softc *sc = addr;

	if (sc->sc_ucom.sc_dying)
		return (ENXIO);

	DPRINTF(("uticom_open: sc = %p\n", sc));


	

/*	usb_device_request_t req;*/
	usbd_status err;
/*
	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = REQUEST_SON;
	USETW(req.wValue, 0);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);

	err = usbd_do_request(sc->sc_ucom.sc_udev, &req, 0);
	if (err){
		printf("%s: uticom_open: %s\n",
		       USBDEVNAME(sc->sc_ucom.sc_dev), usbd_errstr(err));
	return (EIO);
	}
*/	
	sc->sc_status = 0;
	kprintf("uticom_open\n");

	if (sc->sc_intr_number != -1 && sc->sc_intr_pipe == NULL) {
		sc->sc_status = 0; /* clear status bit */

		sc->sc_intr_buf = kmalloc(sc->sc_isize, M_USBDEV, M_WAITOK);
		err = usbd_open_pipe_intr(sc->sc_intr_iface,
					  sc->sc_intr_number,
					  USBD_SHORT_XFER_OK,
					  &sc->sc_intr_pipe,
					  sc,
					  sc->sc_intr_buf,
					  sc->sc_isize,
					  uticom_intr,
					  uticominterval);
		if (err) {
			kprintf("%s: cannot open interrupt pipe (addr %d)\n",
			       USBDEVNAME(sc->sc_ucom.sc_dev),
			       sc->sc_intr_number);
			return (EIO);
		}
	}

DPRINTF(("port opened\n"));
	return (0);

}

static void
uticom_close(void *addr, int portno)
{

	struct uticom_softc *sc = addr;
	if (sc->sc_ucom.sc_dying)
		return;



	usb_device_request_t req;
	usbd_status err;

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = REQUEST_SON;
	USETW(req.wValue, 0);
	USETW(req.wIndex, 0);
	USETW(req.wLength, 0);

	err = usbd_do_request(sc->sc_ucom.sc_udev, &req, 0); /*Try to reset UART part of chip*/
	if (err){
		kprintf("%s: uticom_close: %s\n",
		       USBDEVNAME(sc->sc_ucom.sc_dev), usbd_errstr(err));
	return;
	}

	DPRINTF(("uticom_close: close\n"));

	if (sc->sc_intr_pipe != NULL) {
		err = usbd_abort_pipe(sc->sc_intr_pipe);
		if (err)
			kprintf("%s: abort interrupt pipe failed: %s\n",
			       USBDEVNAME(sc->sc_ucom.sc_dev),
			       usbd_errstr(err));
		err = usbd_close_pipe(sc->sc_intr_pipe);
		if (err)
			kprintf("%s: close interrupt pipe failed: %s\n",
			       USBDEVNAME(sc->sc_ucom.sc_dev),
			       usbd_errstr(err));
		kfree(sc->sc_intr_buf, M_USBDEV);
		sc->sc_intr_pipe = NULL;
	}

}

static void
uticom_intr(usbd_xfer_handle xfer, usbd_private_handle priv, usbd_status status)
{

	struct uticom_softc *sc = priv;
	u_char *buf = sc->sc_intr_buf;
	/*u_char pstatus;*/

	if (sc->sc_ucom.sc_dying)
		return;


	if (status != USBD_NORMAL_COMPLETION) {
		if (status == USBD_NOT_STARTED || status == USBD_CANCELLED) {
			DPRINTF(("uticom_intr: int status: %s\n",usbd_errstr(status)));
			return;
		}

		DPRINTF(("%s: uticom_intr: abnormal status: %s\n",
			USBDEVNAME(sc->sc_ucom.sc_dev),
			usbd_errstr(status)));
		usbd_clear_endpoint_stall_async(sc->sc_intr_pipe);
		return;
	}

int len = xfer->actlen;
if (!len) return;
/*if (!xfer->actlen) return;*/

/*DPRINTF(("%s: uticom status = 0x%02x\n", USBDEVNAME(sc->sc_ucom.sc_dev), buf[0]));*/

/*for (int i=0;i<sc->sc_isize;i++){*/

#ifdef USB_DEBUG
for (int i=0;i<xfer->actlen;i++){
 printf("buf[%d]:%d ",i,buf[i]);
 }
printf("\n"); 
#endif 
		 
	DPRINTF(("%s: xfer_length = %d\n",
		 USBDEVNAME(sc->sc_ucom.sc_dev), xfer->actlen));


 sc->sc_lsr = sc->sc_msr = 0;
 
 if (buf[0]==0) {
 /*msr registres*/
	if (buf[1] & MCR_CTS )sc->sc_msr |= UMSR_CTS;
	if (buf[1] & MCR_DSR )sc->sc_msr |= UMSR_DSR;
	if (buf[1] & MCR_CD )sc->sc_msr |= UMSR_DCD;		
	if (buf[1] & MCR_RI )sc->sc_msr |= UMSR_RI;		
}else {
/*lsr register*/
	if (buf[0] & LCR_OVR )sc->sc_lsr |= ULSR_OE;
	if (buf[0] & LCR_PTE )sc->sc_lsr |= ULSR_PE;
	if (buf[0] & LCR_FRE )sc->sc_lsr |= ULSR_FE;
	if (buf[0] & LCR_BRK )sc->sc_lsr |= ULSR_BI;	
	}
  
    if (uticomstickdsr) sc->sc_msr |= UMSR_DSR;
 	ucom_status_change(&sc->sc_ucom);
}

static void
uticom_get_status(void *addr, int portno, u_char *lsr, u_char *msr)
{

return;
	
	struct uticom_softc *sc = addr;

	DPRINTF(("uticom_get_status:\n"));

	if (lsr != NULL)
		*lsr = sc->sc_lsr;
	if (msr != NULL)
		*msr = sc->sc_msr;

}

#if TODO
static int
uplcom_ioctl(void *addr, int portno, u_long cmd, caddr_t data, int flag,
	     usb_proc_ptr p)
{
	struct uplcom_softc *sc = addr;
	int error = 0;

	if (sc->sc_ucom.sc_dying)
		return (EIO);

	DPRINTF(("uticom_ioctl: cmd = 0x%08lx\n", cmd));

	switch (cmd) {
	case TIOCNOTTY:
	case TIOCMGET:
	case TIOCMSET:
	case USB_GET_CM_OVER_DATA:
	case USB_SET_CM_OVER_DATA:
		break;

	default:
		DPRINTF(("uticom_ioctl: unknown\n"));
		error = ENOTTY;
		break;
	}

	return (error);
}
#endif


static int ti_download_firmware(struct uticom_softc *sc,unsigned int pipeno,usbd_device_handle dev,
	unsigned char *firmware, unsigned int firmware_size)
{

	int buffer_size;
	int pos;
	__uint8_t cs = 0;
	__uint8_t *buffer;
	usbd_status err;

	struct ti_firmware_header *header;

	buffer_size = TI_FIRMWARE_BUF_SIZE + sizeof(struct ti_firmware_header);
	buffer = kmalloc(buffer_size,M_USBDEV, M_WAITOK);
	if (!buffer) {
		kprintf("%s - out of memory", __FUNCTION__);
		return -ENOMEM;
	}

	memcpy(buffer, firmware, firmware_size);
	memset(buffer+firmware_size, 0xff, buffer_size-firmware_size);

	for(pos = sizeof(struct ti_firmware_header); pos < buffer_size; pos++)
		cs = (__uint8_t)(cs + buffer[pos]);

	header = (struct ti_firmware_header *)buffer;
	header->wLength = (__uint16_t)(buffer_size - sizeof(struct ti_firmware_header));

	header->bCheckSum = cs;

	DPRINTF(("uticom: downloading firmware...\n"));



usbd_xfer_handle oxfer=0;
u_char *obuf;
usbd_status error=0;
usbd_pipe_handle pipe;

		err = usbd_open_pipe(sc->sc_ucom.sc_iface, pipeno,
				     USBD_EXCLUSIVE_USE, &pipe);
		if (err) {
			kprintf("%s: open bulk out error (addr %d): %s\n",
			       USBDEVNAME(sc->sc_ucom.sc_dev), pipeno,
			       usbd_errstr(err));
			error = EIO;
			goto fail;
		}

		oxfer = usbd_alloc_xfer(dev);
		if (oxfer == NULL) {
			error = ENOMEM;
			goto fail;
		}

		obuf = usbd_alloc_buffer(oxfer, buffer_size);
		if (obuf == NULL) {
			error = ENOMEM;
			goto fail;
		}

	memcpy(obuf, buffer, buffer_size);

	usbd_setup_xfer(oxfer, pipe,
			(usbd_private_handle)sc, obuf, buffer_size,
			USBD_NO_COPY || USBD_SYNCHRONOUS, USBD_NO_TIMEOUT, 0);
	err = usbd_sync_transfer(oxfer);
	if (err != USBD_NORMAL_COMPLETION) { 
		kprintf( "uticom_downloading_firmware: error: %s\n",	usbd_errstr(err));
		goto fail;
	}



	usbd_free_buffer(oxfer);
	usbd_free_xfer(oxfer);
	oxfer = NULL;
	usbd_abort_pipe(pipe);
	usbd_close_pipe(pipe);
	kfree(buffer,M_USBDEV);
	DPRINTF(("uticom_downloading_firmware: download successful\n"));	
	return 0;



fail:

	usbd_free_buffer(oxfer);
	usbd_free_xfer(oxfer);
	oxfer = NULL;
	usbd_abort_pipe(pipe);
	usbd_close_pipe(pipe);
	kfree(buffer,M_USBDEV);
	DPRINTF(("uticom: download failed\n"));	
	return error;
}
