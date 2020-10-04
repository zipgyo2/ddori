# Target specific macros
TARGET = DDORI
TARGET_SOURCES = \
	ddori.c
TOPPERS_OSEK_OIL_SOURCE = ./ddori.oil

# Don't modify below part
O_PATH ?= build
include /home/osek/nxtOSEK/ecrobot/ecrobot.mak
