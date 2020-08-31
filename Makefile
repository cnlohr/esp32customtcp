#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := cnip_example

include $(IDF_PATH)/make/project.mk
include web/component.mk

listing : build/$(PROJECT_NAME).elf
	xtensa-esp32-elf-objdump -S build/$(PROJECT_NAME).elf  > program.lst
	xtensa-esp32-elf-size build/$(PROJECT_NAME).elf

CFLAGS+=-I. -Iinclude -DMBEDTLS_NET_C


