//
//
// This program reads the DS3231 temperature sensor
// The temperature sensor is at 
//
#include <stdio.h>
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>              //Needed for I2C port
#include <string.h>
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <time.h>
#include "read_ds3231.h"

short swap_short(unsigned short a)
{
    return ((a&0xff00)>>8) | ((a &0xff)<<8);
}
//
//read 6 bytes from ds3231 rtc
//
short read_time(int file_i2c, unsigned char *time, unsigned char deviceRegister )
{
    unsigned char buffer[2];
    unsigned length = 1;
    buffer[0] = deviceRegister;
    
    
	if (write(file_i2c, buffer, length) != length)			
    {
		printf("Failed to write to the i2c bus.\n");
        return -1;

	}
    //read the time
	length = 6;			//<<< Number of bytes to read
	if (read(file_i2c, time, length) != length)
    {
		printf("Failed to read from the i2c bus.\n");
        return -1;
	}
    return 0 ;
    
}
//
//this will write byte to the gpio register of the MPC230008
//



//writebyte
//routine will write a byte to the i2c port.
#if 0
int writeshort_to_MCP9808(int file_i2c,unsigned char deviceRegister, unsigned short data){
    int length;
    int bytes_written;
    unsigned char buffer[2];

    length = 1;
    buffer[0] = deviceRegister;
    // write the register to be written to the pointer register.
    if (write(file_i2c, buffer, length) != length)			
    {
		printf("Failed to write to the i2c bus.\n");
        return -1;

	}
    length = 2;
    buffer[0]=(data>>8) & 0xff;
    buffer[1]=data & 0xff ;
    if (write(file_i2c, buffer, length) != length)			
    {
		printf("Failed to write to the i2c bus.\n");
        return -1;

	}
    return 0;

}
#endif
   
char lower(unsigned char digit)
{
    return(digit &0x0f);
}
char upper(unsigned char digit)
{
    return ( (digit>>4)&0xf );
}
int main(int argc, char * argv[])
{
	int file_i2c;
	int length;
    int rc;
    int i;
    char t12_24;
    int seconds;
    int tenseconds;
    int minutes;
    int tensminutes;
    int hours;
    int tenshours;
    int upper;
    int lower;
    int date;
    int day;
    int hour;
    int month;
    int year;
    char ampm[8];
    char *AM={"AM"}; 
    char *PM={"PM"}; 

	unsigned char time[60] = {0};
    int milliseconds;
    struct timespec ts;

	
	//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
    int addr = I2C_DS3231_ADDRESS ;          //<<<<<The I2C address of the slave

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
   // read the time.
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


