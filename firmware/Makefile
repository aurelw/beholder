DEV_SER = /dev/ttyS0
DEV_PAR = /dev/parport0


VERSION := $(shell sed -e 's/\#define VERSION "\(.*\)"/\1/p;d' firmware/version.h )

ARCH_FILES = firmware/main.c firmware/commandline.c firmware/ds1307.c	\
		firmware/itoa.c firmware/main.c firmware/portmap.c				\
		firmware/selftest.c firmware/sht11.c firmware/time.c			\
		firmware/time_noRTC.c firmware/timeswitch.c firmware/tmp101.c	\
		firmware/uart.c firmware/commandline.h firmware/common.h		\
		firmware/ds1307.h firmware/eeprom.h firmware/hd4478.h			\
		firmware/i2c.h firmware/itoa.h firmware/portmap.h				\
		firmware/selftest.h firmware/sht11.h firmware/time.h			\
		firmware/timeswitch.h firmware/tmp101.h firmware/uart.h			\
		firmware/version.h Makefile firmware/TerraControl/Makefile		\
		firmware/Makefile \

all: build

get_version:
	@echo $(VERSION)

build:
	@$(MAKE) -C firmware build

program:
	@$(MAKE) -C firmware program

install_gcc:
	apt-get install gcc-avr uisp avr-libc binutils-avr gtkterm

clean: 
	@$(MAKE) -C firmware clean
#	@$(MAKE) -C pcb clean

arch:
	@tar czf TerraControl-$(VERSION).tar.gz $(ARCH_FILES)

release:
	$(MAKE) -C firmware release

.PHONY: clean rights build program arch geda sync install_gcc remote_program 

