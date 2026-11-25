# Build the fault-tolerant controller app on top of the Bitcraze firmware.
#   make CRAZYFLIE_BASE=/path/to/crazyflie-firmware        (build)
#   make CRAZYFLIE_BASE=/path/to/crazyflie-firmware cload  (flash over the Crazyradio)
CRAZYFLIE_BASE ?= $(HOME)/crazyflie-firmware

OOT_CONFIG := $(PWD)/app-config

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
