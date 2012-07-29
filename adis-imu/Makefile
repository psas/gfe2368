
#
# Makefile for adis-imu
#

NAME            := adis-imu
LPCLIBDIR       := ../liblpc23xx
LIBDIR          := ..

INCLUDE         := -I.\
                   -I./include\
                   -I$(LPCLIBDIR)/include\
                   -I$(LPCLIBDIR)/lpc23xx-spi/include\
                   -I$(LPCLIBDIR)/lpc23xx-binsem/include\
                   -I$(LPCLIBDIR)/lpc23xx-uart/include\
                   -I$(LPCLIBDIR)/lpc23xx-util/include\
                   -I$(LPCLIBDIR)/lpc23xx-pll/include\
                   -I$(LPCLIBDIR)/lpc23xx-vic/include\
                   -I$(LIBDIR)/gfe2368-info/include\
                   -I$(LIBDIR)/gfe2368-util/include

GFE_BOARD_NUM   := -DBOARD_ONE
include $(LIBDIR)/include/common-test-make.mk

