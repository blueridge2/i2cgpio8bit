all:i2cgpio8bit i2cledsoff
i2cgpio8bit: i2cgpio8bit.c i2cgpio8bit.h
	gcc -o i2cgpio8bit  i2cgpio8bit.c
i2cledsoff: i2cledsoff.c i2cgpio8bit.h
	gcc -o i2cledsoff  i2cledsoff.c




clean:
	rm -rf  i2cgpio8bit i2cledsoff *.o

