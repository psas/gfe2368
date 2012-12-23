
#
# Makefile for olimex status spiflash
#

NAME            := adis-usb-bulk
LPCLIBDIR       := ../../liblpc23xx
LIBDIR          := ../..

INCLUDE         := -I.\
                   -I./include\
                   -I$(LPCLIBDIR)/include\
                   -I$(LIBDIR)/include \
                   -I$(LPCLIBDIR)/lpc23xx-usb/include\
                   -I$(LPCLIBDIR)/external-devices/include

GFE_BOARD_NUM   := -DBOARD_SIX
CSRCS           :=  ADIS16405.c

include $(LIBDIR)/include/common-test-make.mk

