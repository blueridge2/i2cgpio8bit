//
//
//found on the web as an i2c read write example
//no copyright attached.
//this program writes to the git remote MCP23008 8 bit 
// i2c io expander. it turns the gpio a into to outputs and wrtes to them.
// i2cdetect puts it at address 0x20 on the 
//
#include <stdio.h>
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <time.h>
#include "i2cgpio8bit.h"
//
//this will read a byte from the gpio register the mpc230008
//
//
int readgpio(int file_i2c, unsigned char deviceRegister, unsigned char *gpioByte)
{
    unsigned char buffer[4];
    unsigned length = 1;
    buffer[0] = deviceRegister;

	if (write(file_i2c, buffer, length) != length)			
    {
		printf("Failed to write to the i2c bus.\n");
        return -1;

	}
 	//read the io direction register, this just read from the previously written address
	length = 1;			//<<< Number of bytes to read
	if (read(file_i2c, gpioByte, length) != length)
    {
		printf("Failed to read from the i2c bus.\n");
        return -1;
	}
    return 0;
}
//
//this will write byte to the gpio register of the MPC230008
//
int writegpio(int file_i2c,unsigned char device_register,unsigned char gpio_byte)
{   
    unsigned char buffer[4];
    int length=2;
    buffer[0] = device_register ;  //set the internal address of the register of the GPIO address expander
    buffer[1] = gpio_byte;  //making them an output.

    if (write(file_i2c, buffer, length) != length)			
    {
		/* ERROR HANDLING: i2c transaction failed */
		printf("Failed to write to the i2c bus.\n");
      return -1;
	}
    return 0;

}
unsigned char read_byte_from_MCP23008(int file_i2c,unsigned char register_address, int * rc){
   //
	//read the register  
   //
   unsigned char buffer[4];
   int length = 1;			//<<< Number of bytes to write
   buffer[0] = register_address;  //write the MCP23008 register to read
	if (write(file_i2c, buffer, length) != length)			
   {
	   /* ERROR HANDLING: i2c transaction failed */
		printf("Failed to write to the i2c bus.\n");
      *rc = -1;
      return -1;
	}
 	//read the io direction register, this just read from the previously written address
	length = 1;			//<<< Number of bytes to read
	if (read(file_i2c, buffer, length) != length)
   {
      //read() returns the number of bytes actually read,
      // if it doesn't match then an error occurred (e.g. no response from the device)
      //ERROR HANDLING: i2c transaction failed
		printf("Failed to read from the i2c bus.\n");
      *rc = -1;
      return -1;
	}
   *rc = 0;
	return buffer[0];
	


}

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

	
	//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
		return 1;
	}
	
	int addr = 0x20;          //<<<<<The I2C address of the slave
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
		return 1;
	}
   //
	//first read the CONFIGURATION (IOCON) REGISTER 
   //
   //
   io_connection_con = read_byte_from_MCP23008( file_i2c,INTCON, & rc);
   printf("iocon = 0x%02x rc = %d\n",(int)io_connection_con,rc);

   //turn off the address increment bit.
   rc = writebyte_to_MCP23008(file_i2c,INTCON, 0x20);

   io_connection_con = read_byte_from_MCP23008( file_i2c,INTCON, & rc);
   printf("iocon = 0x%02x rc = %d\n",(int)io_connection_con,rc);
   //
   //1 =  Pin is configured as an input.
   //0=  Pin is configured as an output.
   //io pins 8-4 are outputs.
   rc = writebyte_to_MCP23008(file_i2c,IODIR, 0x0f);
   io_direction = read_byte_from_MCP23008( file_i2c,IODIR, & rc);
   printf("iodir = 0x%02x rc = %d\n",(int)io_direction,rc);


   counter = 1;
   milliseconds = 5;
   ts.tv_sec = milliseconds / 1000;
   ts.tv_nsec = ( milliseconds % 1000) * 1000000;
   
   while (1)
   {   
      upper_nibble = counter<<4;
      rc = writebyte_to_MCP23008(file_i2c,GPIO, upper_nibble);
      //printf("sleeping counter= 0x%x\n",counter &0xf);
      sleep(.5);
      nanosleep(&ts, NULL); 
      counter +=1;
      //counter =counter <<1;
      //if ( counter == 0x10 ){
      //   counter = 0x01;
      // }
       
        
    }


}


