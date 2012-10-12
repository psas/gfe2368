
#
# Makefile for olimex status spiflash
#

NAME            := spiflash
LPCLIBDIR       := ../../liblpc23xx
LIBDIR          := ../..

INCLUDE         := -I.\
                   -I./include\
                   -I$(LPCLIBDIR)/include\
                   -I$(LIBDIR)/gfe2368-info/include\
                   -I$(LIBDIR)/gfe2368-util/include

GFE_BOARD_NUM   := -DBOARD_ONE
CSRCS           :=  at45161D.c

include $(LIBDIR)/include/common-test-make.mk

