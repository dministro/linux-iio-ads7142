ROOT_DIR = $(shell pwd)
DL_DIR := $(ROOT_DIR)/dl
O := $(ROOT_DIR)/output

OL_BUILD = $(O)/overlay/build

DTC = dtc

OL_AVDD_LINE =
OL_BUS ?= 1
OL_ADDR ?= 18
OL_IRQ ?= 16
OL_AVDD ?=

OL_DTS_NAME = ads7142_i2c$(OL_BUS)_$(OL_ADDR)-overlay.dts
OL_DTBO_NAME = ads7142_i2c$(OL_BUS)_$(OL_ADDR)
OL_DTBO_FILE = $(OL_DTBO_NAME).dtbo

ifneq ($(OL_AVDD),)
OL_AVDD_LINE = avdd-supply = <\&$(OL_AVDD)>;
endif

ifneq ($(OL_DVDD),)
OL_DVDD_LINE = dvdd-supply = <\&$(OL_DVDD)>;
endif

OL_GPIO_LINE = ads7142_$(OL_BUS)_$(OL_ADDR)_pins: ads7142_$(OL_BUS)_$(OL_ADDR)_pins {\n
OL_GPIO_LINE += brcm,pins = <$(OL_IRQ)>; \/* rdy\/busy *\/\n
OL_GPIO_LINE += brcm,function = <0>; \/* input *\/\n
OL_GPIO_LINE += };\n

OL_INTERRUPTS_LINE = interrupt-parent = <\&gpio>;\n
OL_INTERRUPTS_LINE += interrupts = <$(OL_IRQ) 0x2>; \/* GPIO and falling edge *\/\n

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
	mkdir -p /lib/modules/$(shell uname -r)/drivers/iio/adc
	cp $(ROOT_DIR)/../src/linux/drivers/iio/adc/ti-ads7142.ko \
		/lib/modules/$(shell uname -r)/drivers/iio/adc
	depmod

iio-adc-ti-ads7142-test:
	make -C $(ROOT_DIR)/../test/iio-adc-ti-ads7142-test release
	mkdir -p $(ROOT_DIR)/output/test
	cp $(ROOT_DIR)/../test/iio-adc-ti-ads7142-test/build/iio-adc-ti-ads7142-test \
		$(ROOT_DIR)/output/test
	chmod +x $(ROOT_DIR)/output/test/iio-adc-ti-ads7142-test

overlay:
	mkdir -p $(OL_BUILD)
	cat '$(ROOT_DIR)/resources/overlay.dts.tmpl' | \
	sed --expression='s/_OL_BUS_/$(OL_BUS)/g' | \
	sed --expression='s/_OL_ADDR_/$(OL_ADDR)/g' | \
	sed --expression='s/_OL_IRQ_/$(OL_IRQ)/g' > \
	sed --expression='s/_OL_GPIO_LINE_/$(OL_GPIO_LINE)/g' > \
	sed --expression='s/_OL_INTERRUPTS_LINE_/$(OL_INTERRUPTS_LINE)/g' > \
	sed --expression='s/_OL_AVDD_/$(OL_AVDD_LINE)/g' > \
	sed --expression='s/_OL_DVDD_/$(OL_DVDD_LINE)/g' > \
	$(OL_BUILD)/$(OL_DTS_NAME)
	$(DTC) -W no-unit_address_vs_reg -@ -I dts -O dtb -o $(OL_BUILD)/$(OL_DTBO_FILE) $(OL_BUILD)/$(OL_DTS_NAME)

overlay-install: overlay
	cp $(OL_BUILD)/$(OL_DTBO_FILE) /boot/overlays

boot-configure:
	$(call boot_change_line,dtparam=i2c_arm=off,dtparam=i2c_arm=on)
	$(call boot_add_line,dtparam=i2c_arm=on)
	$(call boot_add_line,dtoverlay=$(OL_DTBO_NAME))

install: driver-install overlay-install

all: install boot-configure iio-adc-test
