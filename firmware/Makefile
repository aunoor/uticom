#
# Makefile	tusb3410 firmware
#

CC =		/usr/local/bin/sdcc
LD =		/usr/local/bin/sdcc
LFLAGS =	--model-small --idata-loc 0xc0 --xram-loc 0xf800 --xram-size 2048 --code-size 16384
#		                            ^ info for weasels only: the stack grows upwards ...
CFLAGS =	-c

OBJS=	main.rel usb.rel ser.rel

all:	$(OBJS) Makefile
	$(LD) $(LFLAGS) $(OBJS)
	ihex/ihex2flat main.ihx firm.flat
	cat utifw3410head.h >./utifw3410_p.h
#	hexdump -f /1.txt <firm.flat >/firm.hex
	hexdump -f ./1.txt <firm.flat >>./utifw3410_p.h
	cat utifw3410foot.h >>./utifw3410_p.h

clean:	
	rm firm.flat *.asm *.ihx *.lnk *.map *.mem *.lst *.rel *.rst *.sym

main.rel:	main.c Makefile
	$(CC) $(CFLAGS) main.c

usb.rel:	usb.c Makefile
	$(CC) $(CFLAGS) usb.c

ser.rel:	ser.c Makefile
	$(CC) $(CFLAGS) ser.c


