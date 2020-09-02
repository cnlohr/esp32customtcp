# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set( COMPONENTS, bootloader bt-support driver esp32 esp_adc_cal
	espcoredump esp_event esp_ringbuf esptool_py heap
	idf_test freertos jsmn json log micro-ecc newlib nvs_flash
	pthread soc spiffs spi_flash ulp unity wpa_supplicant
	app_trace xtensa-debug-module ethernet bootloader_support
	vfs app_update mbedtls main partition_table efuse
	bootloader_support esp_common xtensa esp_rom esp_timer
	esp_system cnip cnhttp )




include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp82xx_example)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DMBEDTLS_NET_C")

