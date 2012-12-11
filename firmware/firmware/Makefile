########################################################################
# This file is part of TerraControl.								   #
# 																	   #
# TerraControl is free software; you can redistribute it and/or modify #
# it under the terms of the GNU General Public License as published	   #
# by the Free Software Foundation; either version 2 of the License, or #
# (at your option) any later version.								   #
# 																	   #
# TerraControl is distributed in the hope that it will be useful,	   #
# but WITHOUT ANY WARRANTY; without even the implied warranty of	   #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the		   #
# GNU General Public License for more details.						   #
# 																	   #
# You should have received a copy of the GNU General Public License	   #
# along with TerraControl; if not, write to the Free Software		   #
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 #
# USA																   #
# Written and (c) by mru											   #
# Contact <mru@fourier> for comment & bug reports					   #
########################################################################


#F_OSC = 11059201L
#F_OSC = 12288000L
MCU = atmega8
MCU_DUDE = m8

# List C source files here. (C dependencies are automatically generated.)
SRC = main.c uart.c #portmap.c 

RELEASE_DIR = ../releases
BUILD_ID_FILE = build-id.h
VERSION_FILE = version.h


GET_DEFINES = $(CC) $(ALL_CFLAGS) -mmcu=$(MCU) -E -dM $(SRC) -Werror
BUILD := $(shell sed -e 's/\#define BUILD_ID "\(.*\)"/\1/p;d' $(BUILD_ID_FILE))
VERSION := $(shell sed -e 's/\#define VERSION "\(.*\)"/\1/p;d' $(VERSION_FILE))
GET_BRANCH = $(shell git branch | sed -e 's/* \(.*\)/\1/p;d')

br:
	echo $(GET_BRANCH)

DATE = $(shell date +%Y%m%d)
HOST = $(shell uname -n)
RELEASE_FILENAME = $(TARGET)-$(HOST)-$(DATE)-$(VERSION)-b$(BUILD)

TAGS: $(SRC)
	etags *.c *.h /usr/lib/avr/include/*.h /usr/lib/avr/include/util/*.h /usr/lib/avr/include//avr/*.h /usr/lib/avr/include/compat/*.h

get_defines: $(SRC)
	$(call GET_DEFINES)

erase_flash:
	$(UISP) $(UISP_FLAGS) --erase

increase_build_number:
	@touch $(BUILD_ID_FILE)
	@echo "#define BUILD_ID \"$$(( $(BUILD) + 1 ))\"" > $(BUILD_ID_FILE)
	@echo
	@echo " | new build number: " $$(( $(BUILD) + 1))
	@echo " |====================== "

find_dummy:
	@$(call GET_DEFINES) > /dev/null

check_branch:
	@[ $(GET_BRANCH) = "master" ] || ( echo "error: not on branch 'master'!"; false)

release: check_branch find_dummy build sizeafter
	@git commit -a -m "committing everything for TAG $(RELEASE_FILENAME)"
	@git tag -m "TAG for $(RELEASE_FILENAME)" $(RELEASE_FILENAME) master
	@echo
	@echo "Released as $(RELEASE_FILENAME)"
	@echo


# for external, 12MHz XTAL:
#FUSE_L = 0xef  # ... 11101111
#FUSE_H = 0xc9  # ... 11001001
#F_OSC = 11059201L
F_OSC = 8000000L

# for internal, calibrated 1MHz
#FUSE_L = 0xe1   # 11100001
#FUSE_H = 0xd9   # 11011001
#F_OSC = 1e6



# for internal, calibrated 8MHz
#FUSE_L = 0xe4   # 11100100
#FUSE_H = 0xd9   # 11011001
#F_OSC = 8e6




#CFLAGS += -Werror

-include Makefile.atmel

.PHONY: profile_size erase_flash release get_defines
