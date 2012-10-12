
#todo: vpath

NAME		= aps-host

CC		= gcc

LDLIBS          = -lm $(shell pkg-config --libs glib-2.0 libusb-1.0)

AR		= ar rcvs

INCLUDE         = -I./include $(shell pkg-config --cflags libusb-1.0)
DEBUG           =

CFLAGS          = $(INCLUDE) -c $(DEBUG) -Wall -O0 -g #-Wextra

###### SOURCES  #################

CSRCS		 = $(NAME).c libusb-util.c

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

.PHONY: rebuild clean

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


