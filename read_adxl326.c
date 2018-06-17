//
//
// This program reads the ads1115 a/d converter 4 channel a/d converter
// which is connected to adxl 326 accellerometer
// The temperature sensor is at 
//
#include <stdio.h>
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>              //Needed for I2C port
#include <string.h>
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <time.h>
#include "read_adxl236.h"

inline short swap_short(unsigned short a)
{
    return ((a & 0xff00)>>8) | ((a & 0xff)<<8);
}
//
//
//
short start_conversion(int file_i2c, int channel)
    //
    //This routine starts a conversion, 
{
    unsigned char buffer[3];
    unsigned length = 1;
    unsigned short base;

    buffer[0] = CONFIG_REG ; // the index register 0, config register.
    buffer[1] = 0x81;
    buffer[2] = 0x83
    channel = channel & 3;
    buffer[0] = buffer[0] | channel << 4
    
	if (write(file_i2c, buffer, length) != length)			
    {
		printf("Failed to write to the i2c bus.\n");
        return -1;

	}
    return 0;
    
}

short read_conversion(int file_i2c)
    //
    //This file reads a result of a conversion from the analog to digital conversion
    //the msb is read firt and the lsb is read second.  This is big endian, so it has to 
    //be swapped.
{
    unsigned char buffer[2];
    unsigned length = 1;
    buffer[0] = CONVERSION_REG ; // the index register 0, conversion register.
    
    
	if (write(file_i2c, buffer, length) != length)			
    {
		printf("Failed to write to the i2c bus.\n");
        return -1;

	}
    //read value of the conversion register.
	length = 2;			//<<< Number of bytes to read
	if (read(file_i2c, buffer , length) != length)
    {
		printf("Failed to read from the i2c bus.\n");
        return -1;
	}
    //acording to the spec, the msb is first and put in buffer[0] and the lsb is put in buffer[1]
    //This is a little endian machine, so lsb must be in byte0, so swap the bytes.
    return swap_short(*( ( signed short *) buffer) );
    
}
   
int main(int argc, char * argv[])
{
	int file_i2c;
	int length;
    int rc;
    int i;
    char t12_24;
   
	unsigned char time[60] = {0};
    int milliseconds;
    struct timespec ts;

	
	//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
    int addr = I2C_ADXL_ADDRESS  ;          //<<<<<The I2C address of the slave

	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
		return 1;
	}
	
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
		return 1;
	}
   //
   // read acceleration.
   //
   //now read the byes
    rc = read_time(file_i2c,  time , 0);
    //if 12_24 is set then it is a 12 hour clock
    //if 12_24 is not set, then it a 24 hour clock
    ampm[0] = 0;
    t12_24 = (time[2] & 0x40);
    if (t12_24 ){
        //check for am/pm
        if (time[2]&0x20){
            strcpy(ampm,"AM");
        }
        else{
            strcpy(ampm,PM);
        }
        hour = time[2] & 0x1f;
    }
    else{
        hour = time[2] & 0x3f;
        ampm[0]=0;
    }
    day = time[3];
    date = time[4];
    month = time[5] & 0x1f;
    year = time[6];
    
    printf("h:m:s=%02x:%02x:%02x %s %2x %2x %2x\n",hour,time[1],time[0],ampm,day,month,year);
    
    
    //convert the temperature to a floating point number.

}


