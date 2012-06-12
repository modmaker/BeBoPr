##############################################################################
#                                                                            #
# BeBoPr - 3D printer software for Linux on BeagleBone                       #
#                                                                            #
# 2012-05-29 - ARM version for BeagleBone with BeBoPr cape                   #
#                                                                            #
#           Copyright (C) 2011-2012 Bas Laarhoven aka modmaker               #
#                                                                            #
# Partly based on code by Triffid Hunter, Traumflug, jakepoz, Markus Hitter, #
# and many others.                                                           #
# Previous Copyright: (C) 2009-2010 Michael Moon aka Triffid_Hunter          #
#                                                                            #
# This program is free software; you can redistribute it and/or modify       #
# it under the terms of the GNU General Public License as published by       #
# the Free Software Foundation; either version 2 of the License, or          #
# (at your option) any later version.                                        #
#                                                                            #
# This program is distributed in the hope that it will be useful,            #
# but WITHOUT ANY WARRANTY; without even the implied warranty of             #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              #
# GNU General Public License for more details.                               #
#                                                                            #
# You should have received a copy of the GNU General Public License          #
# along with this program; if not, write to the Free Software                #
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA #
#                                                                            #
##############################################################################

##############################################################################
#                                                                            #
# These defaults should be ok, change if you need to                         #
#                                                                            #
##############################################################################

# Name of this Makefile (used for "make depend").

DEFS=

#PATH=/home/bas/ti-sdk-am335x-evm-05.03.00.00/linux-devkit/bin:$(PATH)
ARCH=arm
CROSS_COMPILE=arm-arago-linux-gnueabi-

MAKEFILE = Makefile

CSTD=gnu99

PROGRAM = mendel

SOURCES = \
	analog.c \
	bebopr_r2.c \
	debug.c \
	gcode_parse.c \
	gcode_process.c \
	gpio.c \
	heater.c \
	home.c \
	limit_switches.c \
	pruss.c \
	pruss_stepper.c \
	pwm.c \
	serial.c \
	sermsg.c \
	temp.c \
	thermistor.c \
	traject.c \
	$(PROGRAM).c

CC = $(CROSS_COMPILE)gcc
OBJDUMP = $(CROSS_COMPILE)objdump
OBJCOPY = $(CROSS_COMPILE)objcopy

#OPTIMIZE = -Os -ffunction-sections -finline-functions-called-once -mcall-prologues
# OPTIMIZE = -O0
CFLAGS = -g -Wall -Wstrict-prototypes $(OPTIMIZE) $(DEFS) -std=${CSTD} -funsigned-char -funsigned-bitfields -fpack-struct -save-temps -pthread
LDFLAGS = -Wl,--as-needed -Wl,--gc-sections
LIBS = -lm -pthread -lrt
LIBDEPS =

SUBDIRS =

OBJ = $(patsubst %.c,%.o,${SOURCES})

.PHONY: all program clean subdirs doc
.PRECIOUS: %.o %.elf

all: $(PROGRAM).elf

$(PROGRAM).elf: $(LIBDEPS)

subdirs:
	@for dir in $(SUBDIRS); do \
	  $(MAKE) -C $$dir; \
	done

clean: clean-subdirs
	-rm -rf *.o *.elf *.lst *.map *.sym *.lss *.eep *.srec *.bin *.hex *.al *.i *.s *~ *fuse

clean-subdirs:
	@for dir in $(SUBDIRS); do \
	  $(MAKE) -C $$dir clean; \
	done

doc: Doxyfile *.c *.h
	doxygen $<

%.o: %.c Makefile
	@echo "  CC        $@"
	@$(CC) -c $(CFLAGS) -Wa,-adhlns=$(<:.c=.al) -o $@ $(subst .o,.c,$@)

%.elf: $(OBJ)
	@echo "  LINK      $@"
	@$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

%.lst: %.elf
	@echo "  OBJDUMP   $@"
	@$(OBJDUMP) -h -S $< > $@

depend:
	@echo "  Appending dependancy information to '$(MAKEFILE)'"
	@if grep '^# DO NOT DELETE' $(MAKEFILE) >/dev/null; then \
		sed -e '/^# DO NOT DELETE/,$$d' $(MAKEFILE) > $(MAKEFILE).$$$$ && \
			mv -f $(MAKEFILE).$$$$ $(MAKEFILE); \
	fi; \
	echo '# DO NOT DELETE THIS LINE -- make depend depends on it.' >> $(MAKEFILE) ; \
	if [ "$(MCU_TARGET)x"=="x" ] ; then \
		$(CC) -MM $(CDEFS) $(CINCS) $(SOURCES) $(ASRC) >> $(MAKEFILE) ; \
	else \
		$(CC) -MM -mmcu=$(MCU_TARGET) $(CDEFS) $(CINCS) $(SOURCES) $(ASRC) >> $(MAKEFILE) ; \
	fi

install:	all
	@echo "  INSTALLING '$(PROGRAM)' TO '$(TARGET_NFS_ROOT)'"
	@if [ -n "$(TARGET_NFS_ROOT)" ] ; then \
		sudo cp $(PROGRAM).elf $(TARGET_NFS_ROOT)/home/root/ ; \
	else \
		echo "INSTALL: no destination specified" ; \
	fi

.PHONY:	all build elf hex eep lss sym program coff extcoff clean depend applet_files install

# DO NOT DELETE THIS LINE -- make depend depends on it.
analog.o: analog.c analog.h beaglebone.h mendel.h debug.h
bebopr_r2.o: bebopr_r2.c analog.h beaglebone.h temp.h thermistor.h \
 bebopr.h heater.h pwm.h traject.h
debug.o: debug.c debug.h
gcode_parse.o: gcode_parse.c gcode_parse.h serial.h sermsg.h debug.h \
 gcode_process.h bebopr.h
gcode_process.o: gcode_process.c bebopr.h gcode_process.h gcode_parse.h \
 serial.h debug.h temp.h beaglebone.h heater.h pwm.h home.h traject.h \
 pruss_stepper.h algo2cmds.h mendel.h limit_switches.h
gpio.o: gpio.c gpio.h
heater.o: heater.c heater.h temp.h beaglebone.h pwm.h debug.h mendel.h
home.o: home.c beaglebone.h home.h bebopr.h limit_switches.h traject.h \
 pruss_stepper.h algo2cmds.h gcode_process.h debug.h
limit_switches.o: limit_switches.c limit_switches.h traject.h bebopr.h \
 mendel.h gpio.h debug.h
pruss.o: pruss.c pruss.h algo2cmds.h beaglebone.h debug.h
pruss_stepper.o: pruss_stepper.c pruss_stepper.h algo2cmds.h pruss.h \
 beaglebone.h debug.h bebopr.h
pwm.o: pwm.c pwm.h beaglebone.h debug.h
serial.o: serial.c serial.h mendel.h
sermsg.o: sermsg.c sermsg.h serial.h
temp.o: temp.c temp.h beaglebone.h analog.h debug.h mendel.h
thermistor.o: thermistor.c beaglebone.h thermistor.h
traject.o: traject.c bebopr.h traject.h pruss_stepper.h algo2cmds.h \
 debug.h beaglebone.h mendel.h
mendel.o: mendel.c serial.h heater.h temp.h beaglebone.h pwm.h bebopr.h \
 mendel.h gcode_process.h gcode_parse.h limit_switches.h traject.h \
 pruss_stepper.h algo2cmds.h
