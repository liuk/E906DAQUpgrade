#-----------------------------------------------------------------------------
#  Copyright (c) 1991,1992 Southeastern Universities Research Association,
#                          Continuous Electron Beam Accelerator Facility
# 
#  This software was developed under a United States Government license
#  described in the NOTICE file included as part of this distribution.
# 
#  CEBAF Data Acquisition Group, 12000 Jefferson Ave., Newport News, VA 23606
#  Email: coda@cebaf.gov  Tel: (804) 249-7101  Fax: (804) 249-7363
# -----------------------------------------------------------------------------
#  
#  Description:  Makefile for Trigger Supervisor RPC libraries and Server
# 	
# 	
#  Author:  David Abbott, TJANF Data Acquisition Group
# 

ifndef ARCH
	ifdef LINUXVME_LIB
		ARCH=Linux
	else
		ARCH=VXWORKSPPC
	endif
endif

# Defs and build for VxWorks
ifeq ($(ARCH),VXWORKSPPC)

VXWORKS_ROOT = /site/vxworks/5.5/ppc/target

DEFS   = -mcpu=604 -DCPU=PPC604 -DVXWORKS -D_GNU_TOOL -mlongcall \
		-fno-for-scope -fno-builtin -fvolatile -DVXWORKSPPC
INCS   = -I. -I$(VXWORKS_ROOT)/h -I$(VXWORKS_ROOT)/h/rpc -I$(VXWORKS_ROOT)/h/net
CC     = ccppc $(INCS) $(DEFS)
LD     = ldppc

# explicit targets

all: CR_Read.o

install: 
	@echo "installing tsUtil.o"
	@cp -v tsUtil.o $(CODA)/VXWORKSPPC55/bin/

clean:
	@rm -f tsUtil.o

tsUtil.o: tsUtil.c tsUtil.h tsIntUtil.c
	$(CC) -c tsUtil.c

endif

# Defs and build for Linux
ifeq ($(ARCH),Linux)

LINUXVME_LIB	?= ${CODA}/extensions/linuxvme/lib
LINUXVME_INC	?= ${CODA}/extensions/linuxvme/include

CROSS_COMPILE           = 
CC			= $(CROSS_COMPILE)gcc
AR                      = ar
RANLIB                  = ranlib
CFLAGS			= -fpic -shared  -I. -I${LINUXVME_INC} -I/usr/include \
			  -L. -L${LINUXVME_LIB} -DJLAB -ljvme -ltir
ifdef DEBUG
CFLAGS			+= -Wall -g
else
CFLAGS			+= -O2
endif

OBJS			= CR_Read.o

LIBS			= libCR.a

all: echoarch clean $(LIBS) install

#CR_Read.o: CR_Read.c CR_Read.h
#	$(CC) -c $(CFLAGS) -o $@ CR_Read.c

libCR.a: CR_Read.o
	$(CC) -fpic -shared $(CFLAGS) -o libCR.so CR_Read.c
	$(AR) ruv libCR.a CR_Read.o
	$(RANLIB) libCR.a


install: libCR.a
	@cp -v $(PWD)/libCR.a $(LINUXVME_LIB)/libCR.a
	@cp -v $(PWD)/libCR.so $(LINUXVME_LIB)/libCR.so
	@cp -v $(PWD)/CR_Read.h $(LINUXVME_INC)/CR_Read.h



clean distclean:
	@rm -f $(OBJS) $(LIBS) *.so *~


%: %.c libCR.a
	$(CC) $(CFLAGS) -o $@ $(@:%=%.c) -lrt -ljvme 

echoarch:
	echo "Make for $(ARCH)"

.PHONY: all clean distclean

endif
