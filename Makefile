all:i2cgpio8bit 
i2cgpio8bit: i2cgpio8bit.c i2cgpio8bit.h
	gcc -o i2cgpio8bit  i2cgpio8bit.c




clean:
	rm -rf  i2cgpio8bit *.o

