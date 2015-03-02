# $FreeBSD$

MAN=

.include <bsd.own.mk>

# In-tree GCC does not support __attribute__((ms_abi)).
.if ${COMPILER_TYPE} != "gcc"

MK_SSP=		no

PROG=		loader.sym
INTERNALPROG=

# architecture-specific loader code
SRCS=	start.S boot1.c reloc.c

CFLAGS+=	-I.
CFLAGS+=	-I${.CURDIR}/../../efi/include
CFLAGS+=	-I${.CURDIR}/../../efi/include/${MACHINE_CPUARCH}
CFLAGS+=	-I${.CURDIR}/../../../contrib/dev/acpica/include
CFLAGS+=	-I${.CURDIR}/../../..

# Always add MI sources and REGULAR efi loader bits
.PATH:		${.CURDIR}/../efi ${.CURDIR}/../../common \
		${.CURDIR}/../../amd64/boot1.efi
CFLAGS+=	-I${.CURDIR}/../../common

FILES=	boot1.efi boot1.efifat
FILESMODE_boot1.efi=	${BINMODE}

LDSCRIPT=	${.CURDIR}/../efi/ldscript.${MACHINE_CPUARCH}
LDFLAGS=	-Wl,-T${LDSCRIPT} -Wl,-Bsymbolic -shared

${PROG}:	${LDSCRIPT}

OBJCOPY?=	objcopy
OBJDUMP?=	objdump

boot1.efi: loader.sym
	if [ `${OBJDUMP} -t ${.ALLSRC} | fgrep '*UND*' | wc -l` != 0 ]; then \
		${OBJDUMP} -t ${.ALLSRC} | fgrep '*UND*'; \
		exit 1; \
	fi
	${OBJCOPY} -j .data -j .dynamic -j .dynstr -j .dynsym -j .hash \
	    -j .rela.dyn -j .reloc -j .sdata -j .text -j set_Xcommand_set \
	    -j .sbss -j .bss -O binary ${.ALLSRC} ${.TARGET}

CFLAGS+=	-I${.CURDIR}/../../common

boot1.o: ${.CURDIR}/../../common/ufsread.c

# The following inserts out objects into a template FAT file system
# created by generate-fat.sh

.include "${.CURDIR}/Makefile.fat"

boot1.efifat: boot1.efi
	echo ${.OBJDIR}
	uudecode ${.CURDIR}/fat.tmpl.bz2.uu
	mv fat.tmpl.bz2 ${.TARGET}.bz2
	bzip2 -f -d ${.TARGET}.bz2
	dd if=boot1.efi of=${.TARGET} seek=${BOOT1_OFFSET} conv=notrunc

CLEANFILES= boot1.efifat

.endif # ${COMPILER_TYPE} != "gcc"

.include <bsd.prog.mk>

CLEANFILES+=   boot1.efi

