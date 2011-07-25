
#
# Makefile for olimex status led test
#

NAME            := led-test
LPCLIBDIR       := ../../liblpc23xx
LIBDIR          := ../..

INCLUDE         := -I.\
                   -I./include\
                   -I$(LPCLIBDIR)/include\
                   -I$(LPCLIBDIR)/lpc23xx-uart/include\
                   -I$(LPCLIBDIR)/lpc23xx-util/include\
                   -I$(LPCLIBDIR)/lpc23xx-pll/include\
                   -I$(LIBDIR)/gfe2368-info/include\
                   -I$(LIBDIR)/gfe2368-util/include

BOARD_NUM       := -DBOARD_ONE
include $(LIBDIR)/include/common-test-make.mk


