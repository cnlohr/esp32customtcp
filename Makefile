#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := cnip_example

COMPONENTS := bootloader bt-support driver esp32 esp_adc_cal \
	espcoredump esp_event esp_ringbuf esptool_py heap \
	idf_test freertos jsmn json log micro-ecc newlib nvs_flash \
	pthread soc spiffs spi_flash ulp unity wpa_supplicant \
	app_trace xtensa-debug-module ethernet bootloader_support \
	vfs app_update main partition_table efuse \
	bootloader_support esp_common xtensa esp_rom esp_timer \
	esp_system esp_ipc esp_wifi esp_netif mbedtls

COMPONENTS+=cnip cnhttp tcpip_adapter lwipstub

include $(IDF_PATH)/make/project.mk
include web/component.mk

listing : build/$(PROJECT_NAME).elf
	xtensa-esp32-elf-objdump -S build/$(PROJECT_NAME).elf  > program.lst
	xtensa-esp32-elf-size build/$(PROJECT_NAME).elf

CFLAGS+=-I. -Iinclude  -DNO_LWIP

#-DMBEDTLS_NET_C
