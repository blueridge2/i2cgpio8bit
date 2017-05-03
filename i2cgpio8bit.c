//
//
//found on the web as an i2c read write example
//no copyright attached.
//this program writes to the git remote MCP23003 8 bit 
// i2c io expander. it turns the gpio a into to outputs and wrtes to them.
// i2cdetect puts it at address 0x20 on the 
//
#include <stdio.h>
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include "i2cgpio8bit.h"
//
//this will read a byte from the gpio register the mpc230008
//
//
int readgpio(int fileI2c, unsigned char deviceRegister, unsigned char *gpioByte)
{
    unsigned char buffer[4];
    unsigned length = 1;
    buffer[0] = deviceRegister;

	if (write(fileI2c, buffer, length) != length)			
    {
		printf("Failed to write to the i2c bus.\n");
        return -1;

	}
 	//read the io direction register, this just read from the previously written address
	length = 1;			//<<< Number of bytes to read
	if (read(fileI2c, gpioByte, length) != length)
    {
		printf("Failed to read from the i2c bus.\n");
        return -1;
	}
    return 0;
}
//
//this will write byte to the gpio register of the MPC230008
//
int writegpio(int fileI2c,unsigned char deviceRegister,unsigned char gpioByte)
{   
    unsigned char buffer[4];
    int length=2;
    buffer[0] = deviceRegister ;  //set the internal address of the register of the GPIO address expander
    buffer[1] = gpioByte;  //making them an output.

    if (write(fileI2c, buffer, length) != length)			
    {
		/* ERROR HANDLING: i2c transaction failed */
		printf("Failed to write to the i2c bus.\n");
        return 0;
	}
    return -1;

}
int main(int argc, char * argv[])
{
	int file_i2c;
	int length;
	unsigned char buffer[60] = {0};

	
	//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
		return;
	}
	
	int addr = 0x20;          //<<<<<The I2C address of the slave
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
		return;
	}
		
	//first make half the bits outputs.
	buffer[0] = IODIR;  //write 0's to the upper 4 bits making the outputs.
	buffer[1] = 0x0f;  //	
	length = 2;			//<<< Number of bytes to write
    //write() returns the number of bytes actually written,
    //if it doesn't match then an error occurred (e.g. no response from the device)
	if (write(file_i2c, buffer, length) != length)
    {
		/* ERROR HANDLING: i2c transaction failed */
		printf("Failed to write to the i2c bus.\n");
        return -1;
	}
    //now read the IODIR register.  This takes a 1 byte write to write the address
    //into the address into the i2c device so it can be read.
	length = 1;			//<<< Number of bytes to write
	if (write(file_i2c, buffer, length) != length)			
    {
		/* ERROR HANDLING: i2c transaction failed */
		printf("Failed to write to the i2c bus.\n");
	}
 	//read the io direction register, this just read from the previously written address
	length = 1;			//<<< Number of bytes to read
	if (read(file_i2c, buffer, length) != length)
    {
        //read() returns the number of bytes actually read,
        // if it doesn't match then an error occurred (e.g. no response from the device)
        //ERROR HANDLING: i2c transaction failed
		printf("Failed to read from the i2c bus.\n");
        return -1;
	}
	else
	{
		printf("iodirreg0 0x%02X\n", buffer[0]);
	}

    //
    //now loop through and write 
    //
    while (1)
    {
#if 0
        buffer[0] =  GPIO ;  //set the internal address of the register of the GPIO address expander
        if (buffer[1])     //if the buffer is true.
        {
	        buffer[1] = 0x00;  //making them an output.
        }
        else
        {
	        buffer[1] = 0xf0;  //write the upper nibble
        }
       
	    length = 2;			//<<< Number of bytes to write
        if (write(file_i2c, buffer, length) != length)			
        {
		    /* ERROR HANDLING: i2c transaction failed */
		    printf("Failed to write to the i2c bus.\n");
	    }
        printf("sleeping\n");
        sleep(4);
#endif         
    }


}


