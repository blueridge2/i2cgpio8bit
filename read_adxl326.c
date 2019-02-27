//
//
// This program reads the ads1115 a/d converter 4 channel a/d converter
// which is connected to adxl 326 accellerometer
//
#include <stdio.h>
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>              //Needed for I2C port
#include <string.h>
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <time.h>
#include <wiringPi.h>
#include "read_adxl326.h"

#define VOLTS_PER_BIT (6.144/(float) 0x7fff)

static inline short swap_short(unsigned short a)
{
    return ((a & 0xff00)>>8) | ((a & 0xff)<<8);
}
//
//
short read_hi_threshold(int file_i2c)
    //
    //This routine will read the hi threshold register.
{
    unsigned char buffer[3];
    unsigned length = 1;
    unsigned short base;
    unsigned short ret_value;

    buffer[0] = HI_THRESHOLD ; // the index register 0, config register.
    
	if (write(file_i2c, buffer, length) != length)			
    {
		printf("Failed to write to the i2c bus.\n");
        return -1;

    }
    length = 2;
    if (read(file_i2c, buffer, 2) != length)
    {
            printf("Failed to read the i2c");
            return -1;
    }            
    ret_value = swap_short(*( ( signed short *) buffer) );

    return ret_value;
}

//
//

short read_lo_threshold(int file_i2c)
    //
    //This routine will lo threshold register
{
    unsigned char buffer[3];
    unsigned length = 1;
    unsigned short base;
    unsigned short ret_value;

    buffer[0] = LO_THRESHOLD ; // the index register 0, config register.
    
	if (write(file_i2c, buffer, length) != length)			
    {
		printf("Failed to write to the i2c bus.\n");
        return -1;

    }
    length = 2;
    if (read(file_i2c, buffer, 2) != length)
    {
            printf("Failed to read the i2c");
            return -1;
    }            
    ret_value = swap_short(*( ( signed short *) buffer) );

    return ret_value;
}

short read_config_register(int file_i2c)
    //
    //This routine will read the config register.
{
    unsigned char buffer[3];
    unsigned length = 1;
    unsigned short base;
    unsigned short ret_value;

    buffer[0] = CONFIG_REG ; // the index register 0, config register.
    
	if (write(file_i2c, buffer, length) != length)			
    {
		printf("Failed to write to the i2c bus.\n");
        return -1;

    }
    length = 2;
    if (read(file_i2c, buffer, 2) != length)
    {
            printf("Failed to read the i2c");
            return -1;
    }            
    ret_value = swap_short(*( ( signed short *) buffer) );

    return ret_value;
}
///
//
short write_lo_threshold(int file_i2c, short value)
    //
    //This routine starts a conversion, 
{
    unsigned char buffer[3];
    unsigned length = 3;

    buffer[0] = LO_THRESHOLD ; // the index register 0, config register.
    buffer[1] = value >> 8 & 0xff;  // the msb gets transmitted first
    buffer[2] = value & 0xff ;
    
	if (write(file_i2c, buffer, length) != length)			
    {
		printf("Failed to write to the i2c bus.\n");
        return -1;

	}
    return 0;
    
}
//
short write_hi_threshold(int file_i2c, short value)
    //
    //This writes the hi threshold 
{
    unsigned char buffer[3];
    unsigned length = 3;

    buffer[0] = HI_THRESHOLD ; // the index register 0, config register.
    buffer[1] = value >> 8 & 0xff;  // the msb gets transmitted first
    buffer[2] = value & 0xff ;
    
	if (write(file_i2c, buffer, length) != length)			
    {
		printf("Failed to write to the i2c bus.\n");
        return -1;

	}
    return 0;
    
}

short start_conversion(int file_i2c, int channel)
    //
    //This routine starts a conversion, 
{
    unsigned char buffer[3];
    unsigned length = 3;
    unsigned short base;

    buffer[0] = CONFIG_REG ; // the index register 0, config register.
    buffer[1] = 0x81;       // this goes out first to the most sigficant bytes of the conversion register.
    buffer[2] = 0x80 ;
    channel = channel & 3 | 0x4;  // do not used differential inputs
    //printf(" channel = %04x\n", buffer[1]);

    buffer[1] = buffer[1] | (channel << 4) ;
    //printf(" buffer1 = %04x\n", buffer[1]);
    
	if (write(file_i2c, buffer, length) != length)			
    {
		printf("Failed to write to the i2c bus.\n");
        return -1;

	}
    return 0;
    
}
void wait_for_ready(int file_i2c)
{
    unsigned short config_register = 0;
    unsigned char gpio_21;

    config_register = read_config_register(file_i2c);
    //printf("config reg = 0x%02x\n", config_register);
    gpio_21 = digitalRead(21);
    while ((config_register & 0x8000) == 0 )
    {
        config_register = read_config_register(file_i2c) & 0x8000;
        //printf("config reg = 0x%02x\n", config_register);
        gpio_21 = digitalRead(21);
        printf("gpio pin 21 = %x\n",gpio_21);


    }
    gpio_21 = digitalRead(5);
    printf("gpio pin 21 = %x\n",gpio_21);
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
    unsigned short config_register;
    unsigned short conversion;
    int channel;


	//  set up wiring pi
    wiringPiSetup();
    pinMode(22, INPUT);
    
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
    write_hi_threshold(file_i2c, 0x8000);
    write_lo_threshold(file_i2c, 0x0000);

    config_register = read_lo_threshold(file_i2c);
    printf("lo_threshold = 0x%02x\n", config_register);
    config_register = read_hi_threshold(file_i2c);
    printf("hi_threshold = 0x%02x\n", config_register);
    config_register = read_config_register(file_i2c);
    printf("config reg = 0x%02x\n", config_register);
    start_conversion(file_i2c, channel);

    config_register = read_config_register(file_i2c);
    printf("config reg = 0x%02x\n", config_register);
    while(1)
    {
        for (channel=0; channel <3; channel++)
        {
            start_conversion(file_i2c, channel);

            //config_register = read_config_register(file_i2c);
            //printf("config reg = 0x%02x\n", config_register);
            wait_for_ready(file_i2c); 
            //config_register = read_config_register(file_i2c);
            //printf("config reg = 0x%02x\n", config_register);

            conversion = read_conversion(file_i2c);
            printf("chan = %d value = %f \n",channel, (float) conversion * VOLTS_PER_BIT  );
        }
    }

}


