


NAME		= usb-speed-test-host

CC		= gcc

LDLIBS          = -lm $(shell pkg-config --libs glib-2.0 libusb-1.0)
LDPPFLAGS       = -lm

AR		= ar rcvs

INCLUDE         = -I./include $(shell pkg-config --cflags glib-2.0 libusb-1.0)
DEBUG           =

CFLAGS          = $(INCLUDE) -c $(DEBUG) -Wall -O3 -g #-Wextra

###### SOURCES  #################

CSRCS		 = $(NAME).c libusb-source.c imu-device-host-interface.c

COBJS            = $(CSRCS:.c=.o)

PROGS            = $(NAME)

# Are we working in cygwin? If so, then
# complier spits out things with .exe, we
# need to know that....
ifeq ($(TERM),cygwin)
	EXES = {PROGS:=.exe}
else
	EXES = $(PROGS)
endif

.NOTPARALLEL: rebuild 

.PHONY: rebuild clean allclean

.SUFFIXES : .c .cpp .o  

all:  $(EXES) Makefile

$(COBJS): include/$(NAME).h

$(EXES): $(COBJS)

clean:
	$(RM) $(EXES) $(COBJS) \
	*.report\
        *.lst *~ ./include/*~ a.out 

rebuild: rebuild 
	@$(MAKE) -s $(NAME) clean
	@$(MAKE) -s


