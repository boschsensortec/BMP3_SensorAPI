all:
	arm-none-eabi-gcc -c -fno-builtin -Wall -Wstrict-prototypes -Wshadow -Wundef -g -mcpu=cortex-m7 -mthumb -mfloat-abi=soft -I. -Werror bmp3.c

clean:
	rm -f *.o

tags:
	etags -a *.c
	etags -a *.h
