# Builds our controller into the official Crazyflie firmware as an out-of-tree app.
#   make                 build (firmware in ~/crazyflie-firmware, patched with crazyflie-firmware.patch)
#   make cload           flash it over the Crazyradio (put the drone in bootloader mode first)
# See README.md in this folder.
CRAZYFLIE_BASE ?= $(HOME)/crazyflie-firmware
OOT_CONFIG := $(PWD)/app-config

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
