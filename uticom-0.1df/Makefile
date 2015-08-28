
.PATH: ${.CURDIR} /usr/src/sys/dev/usbmisc/ucom

KMOD=	uticom
SRCS=	uticom.h utifw3410_p.h
SRCS+= ucomvar.h opt_usb.h bus_if.h device_if.h
SRCS+=	uticom.c
CFALGS+ = -I${WRKSRC}
NOMAN=
KMODDEPS= ucom

.include <bsd.kmod.mk>
