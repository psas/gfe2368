
#
# Makefile for the IMU
#

NAME            := IMU
LIBDIR          := ..
LPCLIBDIR       := $(LIBDIR)/liblpc23xx
CSRCS		:= LSM303DLH.c LIS331HH.c L3G4200D.c

INCLUDE         := -I.\
                   -I./include\
                   -I$(LPCLIBDIR)/include\
                   -I$(LPCLIBDIR)/lpc23xx-uart/include\
                   -I$(LPCLIBDIR)/lpc23xx-util/include\
                   -I$(LPCLIBDIR)/lpc23xx-pll/include\
                   -I$(LPCLIBDIR)/lpc23xx-binsem/include\
                   -I$(LPCLIBDIR)/lpc23xx-i2c/include\
                   -I$(LPCLIBDIR)/lpc23xx-vic/include\
                   -I$(LPCLIBDIR)/lpc23xx-mam/include\
                   -I$(LPCLIBDIR)/lpc23xx-usb/include\
                   -I$(LIBDIR)/gfe2368-info/include\
                   -I$(LIBDIR)/gfe2368-util/include

GFE_BOARD_NUM   := -DBOARD_FOUR
include $(LIBDIR)/include/common-test-make.mk

