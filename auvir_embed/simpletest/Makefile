CROSS_COMPILE = /home/nikolay/work/dev/auvir/auvir_embed/toolchain/gcc-arm-none-eabi-4_9-2015q2/bin/arm-none-eabi
CC = $(CROSS_COMPILE)-gcc
CXX = $(CROSS_COMPILE)-g++
LD = $(CROSS_COMPILE)-ld
OBJDUMP = $(CROSS_COMPILE)-objdump
OBJCOPY = $(CROSS_COMPILE)-objcopy
 

CFLAGS = -c -Wall 
OUTPUT_DIRECTORY =  /home/nikolay/work/dev/auvir/auvir_embed/simpletest/build/out
OBJ =  /home/nikolay/work/dev/auvir/auvir_embed/simpletest/build/obj

SOURCES=main.c
EXECUTABLE=simpletest

simpletest: main.c 
	$(CC) -o simpletest main.c 
