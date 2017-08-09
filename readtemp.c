//
//
// This program read the MPC9808 temperature sensor
// The temperature sensor is at 
//
#include <stdio.h>
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <time.h>
#include "readtemp.h"

typedef union _buffer {
        short i;
        unsigned char tr_buffer[2];
    } Buffer;

float temperture_to_celcius(unsigned short short_temperature)
{
    float temperature_in_celcius;
    short_temperature = short_temperature  & 0x1fff;
    //calculate the temperure in celcius 
    //if its negative
    if (( short_temperature & 0x1000 ) == 0x1000){
        short_temperature = short_temperature & 0xfff;
        temperature_in_celcius =  ((float)(short_temperature>>8 & 0xff ))*16.0 +  ((float)((short_temperature) & 0xff)) / 16.0;
        //change the temperature to a negative
        temperature_in_celcius = 0.0 -  temperature_in_celcius ;
        
    }
    else{
        temperature_in_celcius =  ((float)(short_temperature>>8 & 0xff ))*16.0 +  ((float)((short_temperature) & 0xff)) / 16.0;    
    }
    return temperature_in_celcius;

}
short swap_short(a)
{
    return ((a&0xff00)>>8) | ((a &0xff)<<8);
}
//
//read 2 bytes from mpc9808 temperature sensor
//
short readtemp(int file_i2c, unsigned char deviceRegister, unsigned int *rc)
{
    unsigned char buffer[2];
    unsigned length = 1;
    Buffer temp_buffer;
    buffer[0] = deviceRegister;
    
    
	if (write(file_i2c, buffer, length) != length)			
    {
		printf("Failed to write to the i2c bus.\n");
        return -1;

	}
 	//set the temperature to zero before the read
    temp_buffer.i = 0;
    //read the temperature
	length = 2;			//<<< Number of bytes to read
	if (read(file_i2c, &temp_buffer.tr_buffer[0], length) != length)
    {
		printf("Failed to read from the i2c bus.\n");
        return -1;
	}
    *rc = 0;
    return swap_short(temp_buffer.i);
    
}
//
//this will write byte to the gpio register of the MPC230008
//
#if 0


//writebyte
//routine will write a byte to the i2c port.
int writebyte_to_MCP23008(int file_i2c,unsigned char register_address, unsigned char data){
   int length;
   int bytes_written;
   unsigned char buffer[2];
   struct timespec time_structure;
   
   //make half the bits outputs.
	buffer[0] = register_address;  //write 0's to the upper 4 bits making the outputs.
	buffer[1] = data;  //	
	length = 2;			//<<< Number of bytes to write
   //write() returns the number of bytes actually written,
    //if it doesn't match then an error occurred (e.g. no response from the device)
	if ( (bytes_written = (write(file_i2c, buffer, length) )) != length)
   {
		/* ERROR HANDLING: i2c transaction failed */
		printf("Failed to write to the i2c bus.\n");
      return -1;
	}
   return bytes_written;
}
#endif
int main(int argc, char * argv[])
{
	int file_i2c;
	int length;
    int rc;
	unsigned char buffer[60] = {0};
    unsigned char io_connection_con;
    unsigned char io_direction;
    unsigned char upper_nibble;
    unsigned char counter;
    int milliseconds;
    struct timespec ts;
    unsigned int temp_byte;
    unsigned short temperature;
    float temperature_in_celcuis;

	
	//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
		return 1;
	}
	
	int addr = I2C_TEMP_PROBE_ADDRESS ;          //<<<<<The I2C address of the slave
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
		return 1;
	}
   //
   // read the temperature.
   //
   //
    temperature =readtemp(file_i2c,  TREG , &rc);
    //convert the temperature to a floating point number.
    temperature_in_celcuis =  temperture_to_celcius(temperature);
 
    printf("temp = %f\n",temperature_in_celcuis );

}


