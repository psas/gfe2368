


NAME		= imu-host

CC		= gcc
CPP		= g++

LDFLAGS         = -lm -lglib-2.0 -lusb-1.0
LDPPFLAGS       = -lm

LD              = $(CC) $(LDFLAGS)
LDPP            = $(CPP) $(LDPPFLAGS)

AR		= ar rcvs

INCLUDE         = -I.\
                  -I./include\
                  -I/usr/include/glib-2.0\
                  -I/usr/lib/i386-linux-gnu/glib-2.0/include\
 				  -I/usr/include/libusb-1.0
DEBUG           =

CFLAGS          = $(INCLUDE) -c $(DEBUG) -Wall -O3 -g #-Wextra

CPPFLAGS	= $(INCLUDE) -c $(DEBUG) -Wall -Wextra -g  -ansi -pedantic -O3

###### SOURCES  #################

CSRCS		 = $(NAME).c libusb-source.c usb-transfer-source.c
CPPSRCS		 =

COBJS            = $(CSRCS:.c=.o)
CPPOBJS          = $(CPPSRCS:.cpp=.o)

PROGS            = $(NAME)

LIBS            = lib$(NAME).a

LIBSRCS         = $(CPPSRCS)
LIBOBJS         = $(LIBSRCS:.cpp=.o)

EXLIBS          = 

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

.cpp.o:
	@echo "-------------- COMPILING $@ "
	@$(CPP) $(CPPFLAGS) -c $<
.c.o :
	@echo "-------------- COMPILING $@ "
	@$(CC) $(CFLAGS) -c $<

.s.o :
	@echo "-------------- COMPILING $@ "
	@${AS} ${ASFLAGS} -o $@ $< > $*.lst

all:  $(EXES) Makefile

$(COBJS): include/$(NAME).h
$(CPPOBJS): include/*.hpp

$(EXLIBS):
	@echo "------------ Recursive make: $(@D) ------------------------"
	@$(MAKE) -s -C $(@D) $(@F)
	
$(LIBS): $(LIBSRCS) $(LIBOBJS) $(EXLIBS)
	@echo "-------------- Making library: $@ -----------------------"
	@$(AR) $@ $(LIBOBJS)

$(EXES): $(COBJS)
	@echo "---------------- LINKING $@ --------------"
	@$(LD) -o $@ $(COBJS) -lusb-1.0 -lglib-2.0
clean:
	@$(RM) $(EXES) $(COBJS) $(CPPOBJS) \
	lib$(NAME).a *.report\
        *.lst *~ ./include/*~ a.out 

rebuild: rebuild 
	@$(MAKE) -s $(NAME) clean
	@$(MAKE) -s


