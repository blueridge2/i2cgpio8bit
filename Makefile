all:i2cgpio8bit i2cledsoff readtemp read_ds3231 read_adxl326
i2cgpio8bit: i2cgpio8bit.c i2cgpio8bit.h
	gcc -o i2cgpio8bit  i2cgpio8bit.c
i2cledsoff: i2cledsoff.c i2cgpio8bit.h
	gcc -o i2cledsoff  i2cledsoff.c

readtemp: readtemp.c readtemp.h
	gcc -g -o readtemp  readtemp.c


read_ds3231: read_ds3231.c read_ds3231.h
	gcc -g -o read_ds3231  read_ds3231.c

read_adxl1326: read_adxl1326.c read_adxl1326.h
	gcc -g -o read_adxl1326  read_adxl1326.c



clean:
	rm -rf  i2cgpio8bit i2cledsoff readtemp read_ds3231  read_adxl1326 *.o

