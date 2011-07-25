
#
# common make command for testing
#

GCC_VERSION     ?= 4.5.2
CROSS           ?= /opt/cross

ARMLINUXGCCLIB  := $(CROSS)/lib/gcc/arm-elf/$(GCC_VERSION)
ARMLINUXLIB     := $(CROSS)/arm-elf/lib

CC              := $(CROSS)/bin/arm-elf-gcc
LD              := $(CROSS)/bin/arm-elf-ld
AR              := $(CROSS)/bin/arm-elf-ar
AS              := $(CROSS)/bin/arm-elf-as
CP              := $(CROSS)/bin/arm-elf-objcopy
OD              := $(CROSS)/bin/arm-elf-objdump

TYPE            ?= lpc23xx

#GFE_BOARD_NUM   ?=
#LPC2378_PORT    = -DLPC2378_PORTB

DEBUG           ?=
#DEBUG           = -DDEBUG
                  
CFLAGS          ?= $(INCLUDE) $(DEBUG) $(USB_PORT) $(GFE_BOARD_NUM) -ggdb -c -Wall -Werror -fno-common -O0 -mfloat-abi=softfp -mcpu=arm7tdmi-s

ASFLAGS         ?= -ggdb -ahls -mfloat-abi=softfp $(INCLUDE)

LDFLAGS         ?= -T $(TYPE).ld -nostartfiles -Map $(NAME).map

CPFLAGS         := -O binary
HEXFLAGS        := -O ihex
ODFLAGS         := --syms -S -l 

ASRCS           := $(ASRCS) boot.s
                  
CSRCS           := $(CSRCS) $(NAME).c hwsys.c

COBJS           = $(CSRCS:.c=.o)

AOBJS           = $(ASRCS:.s=.o)

EXLIBS          = $(LPCLIBDIR)/liblpc23xx.a $(LIBDIR)/libgfe2368.a

PROGS           = $(NAME).out

.NOTPARALLEL: %.a

.PHONY: clean

.SUFFIXES : .c .cpp .s

.c.o :
	@echo "======== COMPILING $@ ========================"
	@$(CC) $(CFLAGS) -c $<

.s.o :
	@echo "======== COMPILING $@ ========================"
	@$(AS) $(ASFLAGS) -o $@ $< > $*.lst
        
all:  $(PROGS) $(EXLIBS) $(NAME).bin $(NAME).hex

$(COBJS): include/*.h

$(EXLIBS):
	@echo "========= Recursive make: $(@D)    ========================"
	$(MAKE) -s -C $(@D) DEBUG=$(DEBUG) USB_PORT=$(USB_PORT) GFE_BOARD_NUM=$(GFE_BOARD_NUM) $(@F)

$(PROGS): $(AOBJS) $(COBJS) $(EXLIBS)
	@echo "========= LINKING $@ ========================"
	@$(LD) $(LDFLAGS) -o $@ $(AOBJS) $(COBJS) $(EXLIBS) -L$(CROSS)/arm-elf/lib -lc -L$(CROSS)/lib/gcc/arm-elf/$(GCC_VERSION) -lgcc

$(NAME).hex: $(NAME).out
	@echo "========= .hex file for $< =================="
	@$(CP) $(HEXFLAGS) $< $@

$(NAME).bin: $(NAME).out
	@echo "========= .bin file for $< =================="
	@$(CP) $(CPFLAGS) $< $@
	@echo "========= .dump file for $< =================="
	@$(OD) $(ODFLAGS) $< > $(NAME).dump

clean:
	@$(RM) $(EXLIBS) $(PROGS) $(AOBJS) $(COBJS) $(NAME).*dump \
	*.map *.hex *.bin *.lst *~ ./include/*~ a.out 

allclean: clean
	$(MAKE)  -s -C $(LIBDIR) allclean
	$(MAKE)  -s -C $(LPCLIBDIR) allclean

rebuild: allclean
	$(MAKE)

