// Copyright 2024 by Ralph Blach under the terms of the GPL 3
#ifndef ADXL236_H
#define ADXL236_H
#define I2C_DEVICE_NAME_BASE  "/dev/i2c-"
#define MAX_STR_LEN 50
#define GPIO22  22
// g defines
#define M_16G  0.0
#define ZERO_G 1.65
#define P_16G  3.3
#define GEES_PER_VOLT  32.0/3.3
//define the i2c address for the a2d
#define I2C_ADXL_ADDRESS     0x48
#define POINTER_REG          0x00
#define CONVERSION_REG       0x00
#define CONFIG_REG           0x01
#define LO_THRESHOLD         0x02
#define HI_THRESHOLD         0x03
#endif

