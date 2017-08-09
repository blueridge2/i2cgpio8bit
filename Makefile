all:i2cgpio8bit i2cledsoff readtemp
i2cgpio8bit: i2cgpio8bit.c i2cgpio8bit.h
	gcc -o i2cgpio8bit  i2cgpio8bit.c
i2cledsoff: i2cledsoff.c i2cgpio8bit.h
	gcc -o i2cledsoff  i2cledsoff.c

readtemp: readtemp.c readtemp.h
	gcc -g -o readtemp  readtemp.c



clean:
	rm -rf  i2cgpio8bit i2cledsoff *.o

