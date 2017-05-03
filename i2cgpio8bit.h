#ifndef I2CGPIO8BIT_H
#define I2CGPIO8BIT_H
//define the i2c address for the 8 bit expander
#define IODIR       0X00        //IODIRECTION
#define IOPOL       0x01        //IPOL INPUT POLARITY PORT REGISTER (ADDR 0x01)   
#define GPINTEN     0X02        //INTERRUPT-ON-CHANGE CONTROL (GPINTEN) REGISTER
#define DEFVAL      0x03        //DEFAULT COMPARE
#define INTCON      0x04        //INTCON INTERRUPT-ON-CHANGE CONTROL REGISTER
#define IOCON       0x05        //IOCON CONFIGURATION REGISTER
#define GPPU        0x06        //GPPU GPIO PULL-UP RESISTOR REGISTER
#define INTF        0x07        //INTF  INTERRUPT FLAG REGISTER
#define INTCAP      0x08        //INTCAP INTERRUPT CAPTURED VALUE FOR PORT REGISTER
#define GPIO        0x09        //GPIO reflect the value of the gpio register
#define OLAT        0x0A        //OUTPUT LATCH REGISTER (OLAT)

#endif

