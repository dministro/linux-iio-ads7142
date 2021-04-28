ROOT_DIR = $(shell pwd)
DL_DIR := $(ROOT_DIR)/dl
O := $(ROOT_DIR)/output

OL_BUILD = $(O)/overlay/build

DTC = dtc

OL_VREF_LINE =
OL_BUS ?= 1
OL_ADDR ?= 18
OL_IRQ ?= 20
OL_VREF ?=
OL_MONITORING_MODE ?=

OL_DTS_NAME = ads7142_i2c$(OL_BUS)_$(OL_ADDR)-overlay.dts
OL_DTBO_NAME = ads7142_i2c$(OL_BUS)_$(OL_ADDR)
OL_DTBO_FILE = $(OL_DTBO_NAME).dtbo

ifneq ($(OL_VREF),)
OL_VREF_LINE = vref-supply = <\&$(OL_VREF)>;
endif

ifeq ($(OL_MONITORING_MODE),y)
OL_MONITORING_MODE_LINE = ti,monitoring-mode;
endif

define boot_change_line
	sed -i /boot/config.txt -i -e 's/^${1}/${2}/'
endef

define boot_add_line
	if [ $(shell egrep -c '^${1}' /boot/config.txt) -eq 0 ]; then \
	echo ${1} >> /boot/config.txt; \
	fi
endef

driver:
	make -C /lib/modules/$(shell uname -r)/build M=$(ROOT_DIR)/../src/linux/drivers/iio/adc modules

driver-install: driver
	cp $(ROOT_DIR)/../src/linux/drivers/iio/adc/ads7142.ko /lib/modules/$(shell uname -r)/extra
	depmod

overlay:
	mkdir -p $(OL_BUILD)
	cat '$(ROOT_DIR)/resources/overlay.dts.tmpl' | \
	sed --expression='s/_OL_BUS_/$(OL_BUS)/g' | \
	sed --expression='s/_OL_ADDR_/$(OL_ADDR)/g' | \
	sed --expression='s/_OL_IRQ_/$(OL_IRQ)/g' > \
	sed --expression='s/_OL_VREF_/$(OL_VREF_LINE)/g' > \
	sed --expression='s/_OL_MONITORING_MODE_/$(OL_MONITORING_MODE_LINE)/g' > \
	$(OL_BUILD)/$(OL_DTS_NAME)
	$(DTC) -W no-unit_address_vs_reg -@ -I dts -O dtb -o $(OL_BUILD)/$(OL_DTBO_FILE) $(OL_BUILD)/$(OL_DTS_NAME)

overlay-install: overlay
	cp $(OL_BUILD)/$(OL_DTBO_FILE) /boot/overlays

boot-configure:
	$(call boot_change_line,dtparam=i2c_arm=off,dtparam=i2c_arm=on)
	$(call boot_add_line,dtparam=i2c_arm=on)
	$(call boot_add_line,dtoverlay=$(OL_DTBO_NAME))

install: driver-install overlay-install

all: install boot-configure
