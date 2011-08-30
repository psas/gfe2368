
#
# Makefile for libgfe2368
#

NAME            := libgfe2368
LPCLIBDIR       := ./liblpc23xx
CROSS           ?= /opt/cross

CC              := $(CROSS)/bin/arm-elf-gcc
LD              := $(CROSS)/bin/arm-elf-ld
AR              := $(CROSS)/bin/arm-elf-ar
AS              := $(CROSS)/bin/arm-elf-as
CP              := $(CROSS)/bin/arm-elf-objcopy
OD              := $(CROSS)/bin/arm-elf-objdump

TYPE            ?= lpc23xx

GFE_BOARD_NUM   ?= -DBOARD_ONE
#GFE_BOARD_NUM   ?= -DBOARD_THREE

# GFE have at least one circuit error in USB connections
LPC23XX_PART    ?= -DGFE_LPC2368

##
# If you are using port B the LPC2378 uncomment out the next line.
# this is the default, make LPC2378_PORT= will overwrite
#LPC2378_PORT    = -DLPC2378_PORTB

DEBUG           += -g
#DEBUG           = -DDEBUG
 
INCLUDE         := -I$(LPCLIBDIR)/include\
		   -I$(LPCLIBDIR)/lpc23xx-pll/include\
		   -I$(LPCLIBDIR)/lpc23xx-mam/include\
		   -I$(LPCLIBDIR)/lpc23xx-vic/include\
		   -I$(LPCLIBDIR)/lpc23xx-util/include\
		   -I$(LPCLIBDIR)/lpc23xx-uart/include\
                   -I./gfe2368-info/include\
                   -I./gfe2368-util/include
	
HS              :=  $(wildcard ./include/*.h)\
                    $(wildcard ./gfe2368-util/include/*.h)

EXLIBS          = ./liblpc23xx/liblpc23xx.a

LIBS            = $(NAME).a

TESTS           = ./gfe2368-util/led-test/led-test.hex\
	          ./gfe2368-usb/datapath-test/datapath-test.hex
		  
TESTSRCS        = $(wildcard gfe2368-*/*test/*c)
TESTOBJS        = $(TESTSRCS:.c=.o)

ASRCS           := $(wildcard gfe2368-*/*.s)

CSRCS           := $(wildcard gfe2368-*/*.c)

COBJS           = $(CSRCS:.c=.o)

AOBJS           = $(ASRCS:.s=.o)
                  
CFLAGS          = $(INCLUDE) $(DEBUG) $(LPC23XX_PORT) $(LPC23XX_PART) $(GFE_BOARD_NUM) -c -Wall -Werror -mfloat-abi=softfp -fno-common -O0 -mcpu=arm7tdmi-s

ARCHIVEFLAGS    = rvs

ASFLAGS         = -ggdb -ahls -mfloat-abi=softfp $(INCLUDE) 
 
.PHONY: clean allclean rebuild

.SUFFIXES : .c .cpp .s

.c.o :
	@echo "========== COMPILING $@ ========================"
	@$(CC) $(CFLAGS) -o $(<:.c=.o) -c $<

.s.o :
	@echo "========== COMPILING $@ ========================"
	@$(AS) $(ASFLAGS) -o $@ $< > $*.lst
        

all: $(LIBS) $(EXLIBS) Makefile

tests: $(TESTS) 

$(COBJS): $(HS)

$(EXLIBS): 
	@echo "===== Recursive make: $(@D) ========================"
	@$(MAKE) -s USB_PORT=$(USB_PORT) DEBUG=$(DEBUG) -s -C $(@D) $(@F)

$(LIBS): $(AOBJS) $(COBJS) $(EXLIBS)
	@echo "======= Making Library $@ ========================"
	@$(AR) $(ARCHIVEFLAGS) $@ $(AOBJS) $(COBJS)

$(TESTS): $(LIBS) $(ASRCS) $(CSRCS) $(TESTSRCS)
	@echo "===== Recursive make: $(@D) ========================"
	@$(MAKE) -s -C $(@D) $(@F)

clean:
	@$(RM)  $(LIBS) $(AOBJS) $(COBJS) $(COBJS) \
	*.map *.hex *.bin *.lst *~ ./include/*~ a.out 
	@$(MAKE) -s -C gfe2368-util/led-test clean
	@$(MAKE) -s -C gfe2368-usb/datapath-test clean

allclean: clean
	$(MAKE) -s -C liblpc23xx clean
	
rebuild: allclean
	@$(MAKE)

