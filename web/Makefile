
include ../esp82xx/common.mf

PATH_TO_SRC = ../esp82xx/web/


PAGE_TITLE = esp82xx-basic
PAGE_SCRIPTS = $(wildcard page/*.js) # include all javascrpts in ./web/page/
PAGE_HEADING = Welcome to the basic Web-GUI
PAGE_INFO = This is the basic web interface for esp82xx-series chips


.PHONY : all clean push
all : mfsmaker page.mpfs burn

CC = gcc
CFLAGS = $(OPTS)
LDLIBS = -lusb-1.0
PAGE_SCRIPT = $(foreach s, $(call uniq, $(notdir $(wildcard page/jquery*gz) menuinterface.js $(PAGE_SCRIPTS))), <script language="javascript" type="text/javascript" src="$(s)"></script>)

mfsmaker : $(PATH_TO_SRC)mfsmaker.c

mfsmaker :
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS) -lusb-1.0

page.mpfs : mfsmaker $(wildcard page/*) Makefile
	$(RM) -r tmp
	cp -r page tmp
	# Inception level over 9000 for poor man's template substitution
	$(info bla$ ${\n} blubb)
	$(foreach p, $(filter PAGE_%,$(.VARIABLES)) $(filter PROJECT_%,$(.VARIABLES)) VERSSTR, \
		sed -i "s/{{$(p)}}/$$(echo -n '$($(p))' | sed -e 's/[\/&"]/\\&/g')/" tmp/index.html && \
	) true
	./mfsmaker tmp page.mpfs

burn :
	esptool.py --baud 921600 --port /dev/ttyUSB0 write_flash 0x110000 page.mpfs

clean :
	$(RM) mfsmaker page.mpfs tmp/*
