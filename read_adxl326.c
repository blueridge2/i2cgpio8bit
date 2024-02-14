/* Copyright 2024 by Ralph Blach under the gpl3 public license. see https://www.gnu.org/licenses/gpl-3.0.en.html#license-text
for the entire text
 *  
 *  @file  read_adxl326.c
    @author Ralph Blach
    @date April 15, 2019
  
    This program reads the adafruit ads1115 a/d converter 4 channel a/d converter
    which is connected to adxl 326 accellerometer
    the data sheet for the ads1115 is here https://cdn-shop.adafruit.com/datasheets/ads1115.pdf
*/

    
//
#include <stdio.h>
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>              //Needed for I2C port
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <limits.h>
#include <math.h>
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <time.h>
#include <errno.h>
#include <pigpio.h>

#include "read_adxl326.h"

#define VOLTS_PER_BIT (4.096/(float) 0x7fff)
#define CHANNEL1_OFFSET  0
#define CHANNEL2_OFFSET  0
#define CHANNEL3_OFFSET  0

static inline short swap_short(unsigned short a)
/** @brief This will inline byte swap a short

    @param short value
    @returns byte swapped short
**/
{
    return ((a & 0xff00)>>8) | ((a & 0xff)<<8);
}
static float volts_to_g(float voltage)
/** @breif this routine takes a voltage and returns the accelleration in g's
    
    @voltage the voltage from the a2d represting g's
    @return the g as a float
    
**/
{
    float gees = GEES_PER_VOLT * voltage - 16.0 ;
    return gees;
}

short read_hi_threshold(int file_i2c)
/** @brief read the high threshold register from the ads1115 a/d converter

    @param i2c device file handle
    @returns the threshold value
**/
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


short read_lo_threshold(int file_i2c)
/** @breif read the ads1115 low threshold register

    @param file_i2c the i2c file handle
    @return low threshold register
**/
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
/** @brief read the ads1115 config register

    @param i2c file handle
    @return the config register value
**/
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
/** @breif writes the low threshold ads1115 register
    @param file_i2c the i2c file handle
    @param value the value to write
    @return 0
*/
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
/** @brief writes the ads1115 low threshold register
    @param file_i2c the i2c device file handle
    @param value the value to write
    @return 0
**/ 
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
/** @brief this will start the conversion on the remote i2c device

    @param file_i2c the file handle for the i2c device,
    @param channel a2d converter has 4 channels this is the channel number
    @returns 0/False
**/
{
    unsigned char buffer[3];
    unsigned length = 3;
    unsigned short base;

    buffer[0] = CONFIG_REG ; // the index register 0, config register.
    buffer[1] = 0x83;       // this goes out first to the most sigficant bytes of the conversion register.
                            // start, conversion, bit 15, pga bits = 1, bits(11-9) full volt=4.096, 
                            // bit 8 = single shot
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
void wait_for_ready(int file_i2c, int gpio_pin)
{
/**  @breif wait_for_ready  Wait for the gpio to to low
    
    @param file_i2c the interger that contains the file handle for the i2c device driver
    @param gpio the gpio to be used reading the alert from the raspberry pi
**/
    unsigned short config_register = 0;
    unsigned char gpio_value;

    config_register = read_config_register(file_i2c);
    //printf("config reg = 0x%02x\n", config_register);
    //gpio_21 = digitalRead(21);
    //printf("gpio pin 21 = %x\n",gpio_21);
    //while ((config_register & 0x8000) == 0 )
    while ((gpio_value = gpioRead(GPIO22)) == 1 )
    {
        config_register = read_config_register(file_i2c) & 0x8000;
        //printf("config reg = 0x%02x\n", config_register);
        //    gpio_21 = digitalRead(21);
        //    printf("gpio pin 21 = %x\n",gpio_21);
    }
    gpio_value = gpioRead(GPIO22);
    // printf("gpio pin 21 = %x\n",gpio_21);
}
short read_conversion(int file_i2c)
/** @breif This routine reads a result of a conversion from the analog to digital conversion
       the msb is read firt and the lsb is read second.  This is big endian, so it has to 
       be swapped.
    @param file_i2c the file handle of the i2c device
    @return the conversion
**/
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
double string_to_double(char *p_string, int *p_errno)
/** @breif converts a string to a double(float)
    if the number does not contain any a valid floating point,  returns EINVAL since errno will not be set
    
    @param p_string a pointer to a character string
    @param errno a pointer to an int
    @return double if the conversion succeeds, a zero errno, if errno is not zero, the  convers failed
**/
{
    char *p_end;
    errno = 0;
    double value;
    value = strtod(p_string, &p_end);
    if (p_string == p_end)
    {
        fprintf(stderr, "sting : %s  invalid floating point  (no digits found, 0 returned)\n", p_string);
        errno = EINVAL;
    }
    else if (errno == !0)
    {
        fprintf(stderr,"%s is an illegal floating errno = %d\n", p_string, errno);
    }
    else if (errno == 0 && p_string && *p_end != 0)
    {
        printf ("value : %s valid float  (but additional characters remain, so count as invalid)\n", p_string);
        errno = EINVAL;
    }
    *p_errno = errno; // return the error number
    return value;
}


long string_to_long(char *p_string, int *p_errno)
/** @breif this converts a string to a long with error recovery
        if the number does not contain any digits return EINVAL since errno will not be set
    
    @param p_string a pointer to the string
    @parm p_errno pointer to the errno
    @return long, p_errno
**/
{
    long value;
    char *p_end;
    errno = 0;
    value = strtol(p_string, &p_end, 10);
    /* test return to number and errno values */
    if (p_string == p_end)
    {
        fprintf (stderr, " value : %lu  invalid  (no digits found, 0 returned)\n", value);
        errno = EINVAL;
    }
    else if (errno == ERANGE)
    {
        fprintf (stderr, " value : %lu either underflow or overflow\n", value);
    }
    else if (errno == EINVAL) 
    {
        fprintf (stderr, " value : %lu invalid base\n", value);
    }
    else if (errno != 0 && value == 0)
    {
        fprintf (stderr, " value : %lu  invalid\n", value);
    }
    else if (errno == 0 && p_string && *p_end != 0)
    {
        printf (" number : %lu    valid  (but additional characters remain, so count as invalid)\n", value);
        errno = EINVAL;
    }
     
    *p_errno = errno; // return the error number
    return value;
}
    
int main(int argc, char * argv[])
/** @brief This will run the i2c analog to digitial converter to conver the analog for
    accelerometer to digital.

    @param argc the number of arguments
    @param * argv[] and array of character pointers

    There the valid arguments are -c -d integer
    the -c is not used, -d is the delay between loops to read out the accerations between
    axis of the accerometer
    */
{
	int file_i2c;
	int length;
    int rc;
    unsigned short config_register;
    unsigned short conversion;
    int channel;
    float chan1_voltage = 0;
    float chan2_voltage = 0;
    float chan3_voltage = 0;
    float chan1_zeroed =0;
    float chan2_zeroed =0;
    float chan3_zeroed =0;
    float voltage;
    int count = 0;
    int cflag = 0;
    int nflag = 0;
    int dflag = 0;
    char i2c_file_name[MAX_STR_LEN] = I2C_DEVICE_NAME_BASE;
    char *pdelay_value = NULL;
    char *pdevice_number = NULL;
    long i2c_number;
    long device_number_long;
    char *p_end;
    float delay_float;
    long index;
    long c;
    double int_part;
    float gees;  // the gees
    int fractional_part_in_nano_seconds;
    struct timespec time, remaining;

    
    opterr = 0;
    if (gpioInitialise() < 0) return 1;

    // -d the delay in the loop
    // -n the i2c device number
    // -c cflag optional no uses
    while ((c = getopt (argc, argv, "cn:d:")) != -1)
    {
        switch (c)
        {
            case 'c':
                cflag = 1;
                break;
            case 'd':
                pdelay_value = optarg;  // The delay for the loop
                break;
            case 'n':
                nflag = 1;
                pdevice_number = optarg; // the i2c device number
                break;
            case '?':
                if (optopt == 'c')
                    fprintf (stderr, "Option -%c requires an argument.\n", optopt);
                else if (isprint (optopt))
                    fprintf (stderr, "Unknown option `-%c'.\n", optopt);
                else
                    fprintf (stderr, "Unknown option character `\\x%x'.\n",optopt);
                    return 1;
            default:
                ;
        }
    }
    if (pdelay_value == NULL)
    {
        fprintf(stderr,"no value specified for -d, specify a floating point number\n");
        exit(-1);
    }
    
    //convert the delay from a string to a double
    errno = 0;
    delay_float = string_to_double(pdelay_value, &errno);
    if (errno)
    {
        fprintf(stderr,"conversion of %s to a float failed with errno = %d\n", pdelay_value, errno);
        exit(-1);
    }

    printf("d=%f\n",delay_float); 

    // convert the 
    fractional_part_in_nano_seconds = (int)round(modf(delay_float, &int_part) * 1000000000.0);
    printf("int part = %d, fractonal part =%d nanoseconds\n",int_part, fractional_part_in_nano_seconds);
    time.tv_sec = int_part;
    time.tv_nsec = fractional_part_in_nano_seconds;
    
    gpioSetMode(GPIO22, PI_INPUT);
    gpioSetPullUpDown(GPIO22, PI_PUD_UP);
    
    // now get the i2c device number
    if (nflag==1)
    {   
        char device_number_str[MAX_STR_LEN];
        errno = 0;
        i2c_number = string_to_long(pdevice_number, &errno);
        if (errno != 0)
        {
            fprintf(stderr, "conversion to double failed, %s is not a integer\n", pdevice_number);
            exit(1);
        }
        sprintf(device_number_str,"%d", i2c_number); 
        strncat(i2c_file_name, device_number_str, MAX_STR_LEN-1);
    }
    else 
    {
        strncat(i2c_file_name, "1", MAX_STR_LEN-1);
    }
    printf("the i2c_device_driver name = %s\n", i2c_file_name);
	//----- OPEN THE I2C BUS -----
    
	
    int addr = I2C_ADXL_ADDRESS  ;          //<<<<<The I2C address of the slave
    
	if ((file_i2c = open(i2c_file_name, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus with file name %s\n", i2c_file_name);
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
            wait_for_ready(file_i2c, GPIO22); 
            //config_register = read_config_register(file_i2c);
            //printf("config reg = 0x%02x\n", config_register);

            conversion = read_conversion(file_i2c);
            voltage = conversion * VOLTS_PER_BIT;
            gees = volts_to_g(voltage); 
            
            printf("chan = %d value = %f gees = %f\n",channel, (float) voltage,gees );
    
            switch (channel)
            {
                case 0:
                    chan1_voltage += voltage;
                    chan1_zeroed = voltage-CHANNEL1_OFFSET;
                    break;
                case 1:
                    chan2_voltage += voltage;
                    chan2_zeroed = voltage-CHANNEL2_OFFSET;
                    break;
                case 2:
                    chan3_voltage += voltage;
                    chan3_zeroed = voltage-CHANNEL3_OFFSET;
                    break;
                default:
                    printf("error");
                    return -1;
            }
        }
        count ++;
        printf("average chan1 = %f, chan2=%f, chan3=%f\n", chan1_voltage/(float)count,
                                                                chan2_voltage/(float)count,
                                                                chan3_voltage/(float)count);
        printf("chan1 = %f, chan2=%f, chan3=%f\n\n", chan1_zeroed,
                                                                chan2_zeroed,
                                                                chan3_zeroed);
        if(nanosleep(&time , &remaining) < 0 )   
        {
            fprintf(stderr, "Nano sleep system call failed \n");
            return -1;
        }
    }
   gpioTerminate();
    

}


