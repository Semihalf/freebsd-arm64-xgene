# $FreeBSD$

.PATH: ${.CURDIR}/../common

SRCS=		crti.S crtn.S
OBJS=		${SRCS:N*.h:R:S/$/.o/g}
OBJS+=		gcrt1.o crt1.o Scrt1.o
CFLAGS+=	-I${.CURDIR}/../common \
		-I${.CURDIR}/../../libc/include

all: ${OBJS}

CLEANFILES=	${OBJS} crt1_c.o crt1_s.o gcrt1_c.o Scrt1_c.o
CLEANFILES+=	crt1_c.s gcrt1_c.s Scrt1_c.s

# See the comment in lib/csu/common/crtbrand.c for the reason crt1_c.c is not
# directly compiled to .o files.

gcrt1_c.s: crt1_c.c
	${CC} ${CFLAGS} -DGCRT -S -o ${.TARGET} ${.CURDIR}/crt1_c.c
	sed ${SED_FIX_NOTE} ${.TARGET}

gcrt1_c.o: gcrt1_c.s
	${CC} ${ACFLAGS} -c -o ${.TARGET} gcrt1_c.s

gcrt1.o: gcrt1_c.o crt1_s.o
	${LD} ${_LDFLAGS} -o gcrt1.o -r crt1_s.o gcrt1_c.o

crt1_c.s: crt1_c.c
	${CC} ${CFLAGS} -S -o ${.TARGET} ${.CURDIR}/crt1_c.c
	sed ${SED_FIX_NOTE} ${.TARGET}

crt1_c.o: crt1_c.s
	${CC} ${ACFLAGS} -c -o ${.TARGET} crt1_c.s

crt1.o:	crt1_c.o crt1_s.o
	${LD} ${_LDFLAGS} -o crt1.o -r crt1_s.o crt1_c.o
	objcopy --localize-symbol _start1 crt1.o

Scrt1_c.s: crt1_c.c
	${CC} ${CFLAGS} -fPIC -DPIC -S -o ${.TARGET} ${.CURDIR}/crt1_c.c
	sed ${SED_FIX_NOTE} ${.TARGET}

Scrt1_c.o: Scrt1_c.s
	${CC} ${ACFLAGS} -c -o ${.TARGET} Scrt1_c.s

Scrt1.o: Scrt1_c.o crt1_s.o
	${LD} ${_LDFLAGS} -o Scrt1.o -r crt1_s.o Scrt1_c.o
	objcopy --localize-symbol _start1 Scrt1.o

realinstall:
	${INSTALL} -o ${LIBOWN} -g ${LIBGRP} -m ${LIBMODE} \
	    ${OBJS} ${DESTDIR}${LIBDIR}

.include <bsd.lib.mk>
